//! HAL interface to the SPIM peripheral
//!
//! See product specification, chapter 31.
use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

pub use crate::target::spim0::frequency::FREQUENCYW as Frequency;
pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

use crate::target::{spim0, SPIM0};
use core::iter::repeat_with;

#[cfg(any(feature = "52832", feature = "52840"))]
use crate::target::{SPIM1, SPIM2};

use crate::gpio::{Floating, Input, Output, Pin, PushPull};
use crate::prelude::*;
use crate::target_constants::{EASY_DMA_SIZE, FORCE_COPY_BUFFER_SIZE};
use crate::{slice_in_ram, DmaSlice};

pub trait SpimExt: Deref<Target = spim0::RegisterBlock> + Sized {
    fn constrain(self, pins: Pins, frequency: Frequency, mode: Mode, orc: u8) -> Spim<Self>;
}

macro_rules! impl_spim_ext {
    ($($spim:ty,)*) => {
        $(
            impl SpimExt for $spim {
                fn constrain(self, pins: Pins, frequency: Frequency, mode: Mode, orc: u8) -> Spim<Self> {
                    Spim::new(self, pins, frequency, mode, orc)
                }
            }
        )*
    }
}

impl_spim_ext!(SPIM0,);

#[cfg(any(feature = "52832", feature = "52840"))]
impl_spim_ext!(SPIM1, SPIM2,);

/// Interface to a SPIM instance
///
/// This is a very basic interface that comes with the following limitations:
/// - The SPIM instances share the same address space with instances of SPIS,
///   SPI, TWIM, TWIS, and TWI. You need to make sure that conflicting instances
///   are disabled before using `Spim`. See product specification, section 15.2.
pub struct Spim<T>(T);

impl<T> embedded_hal::blocking::spi::Transfer<u8> for Spim<T>
where
    T: SpimExt,
{
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        // If the slice isn't in RAM, we can't write back to it at all
        ram_slice_check(words)?;

        for chunk in words.chunks(EASY_DMA_SIZE) {
            self.do_spi_dma_transfer(
                DmaSlice::from_slice(chunk),
                DmaSlice::from_slice(chunk),
                |_| {},
            )?;
        }

        Ok(words)
    }
}

impl<T> embedded_hal::blocking::spi::Write<u8> for Spim<T>
where
    T: SpimExt,
{
    type Error = Error;

    fn write<'w>(&mut self, words: &'w [u8]) -> Result<(), Error> {
        // Mask on segment where Data RAM is located on nrf52840 and nrf52832
        // Upper limit is choosen to entire area where DataRam can be placed
        let needs_copy = !slice_in_ram(words);
        let chunk_sz = if needs_copy {
            EASY_DMA_SIZE
        } else {
            FORCE_COPY_BUFFER_SIZE
        };

        for chunk in words.chunks(chunk_sz) {
            if needs_copy {
                // We don't really need to initialize this...
                let mut buf = [0u8; FORCE_COPY_BUFFER_SIZE];
                buf[..chunk.len()].copy_from_slice(chunk);

                self.do_spi_dma_transfer(
                    DmaSlice::from_slice(&buf[..chunk.len()]),
                    DmaSlice::null(),
                    |_| {},
                )?;
            } else {
                self.do_spi_dma_transfer(DmaSlice::from_slice(chunk), DmaSlice::null(), |_| {})?;
            }
        }

        Ok(())
    }
}
impl<T> Spim<T>
where
    T: SpimExt,
{
    pub fn new(spim: T, pins: Pins, frequency: Frequency, mode: Mode, orc: u8) -> Self {
        // Select pins
        spim.psel.sck.write(|w| {
            let w = unsafe { w.pin().bits(pins.sck.pin) };
            #[cfg(feature = "52840")]
            let w = w.port().bit(pins.sck.port);
            w.connect().connected()
        });

        match pins.mosi {
            Some(mosi) => spim.psel.mosi.write(|w| {
                let w = unsafe { w.pin().bits(mosi.pin) };
                #[cfg(feature = "52840")]
                let w = w.port().bit(mosi.port);
                w.connect().connected()
            }),
            None => spim.psel.mosi.write(|w| w.connect().disconnected()),
        }
        match pins.miso {
            Some(miso) => spim.psel.miso.write(|w| {
                let w = unsafe { w.pin().bits(miso.pin) };
                #[cfg(feature = "52840")]
                let w = w.port().bit(miso.port);
                w.connect().connected()
            }),
            None => spim.psel.miso.write(|w| w.connect().disconnected()),
        }

        // Enable SPIM instance
        spim.enable.write(|w| w.enable().enabled());

        // Configure mode
        spim.config.write(|w| {
            // Can't match on `mode` due to embedded-hal, see https://github.com/rust-embedded/embedded-hal/pull/126
            if mode == MODE_0 {
                w.order().msb_first().cpol().active_high().cpha().leading()
            } else if mode == MODE_1 {
                w.order().msb_first().cpol().active_high().cpha().trailing()
            } else if mode == MODE_2 {
                w.order().msb_first().cpol().active_low().cpha().leading()
            } else {
                w.order().msb_first().cpol().active_low().cpha().trailing()
            }
        });

        // Configure frequency
        spim.frequency.write(|w| w.frequency().variant(frequency));

        // Set over-read character to `0`
        spim.orc.write(|w|
            // The ORC field is 8 bits long, so `0` is a valid value to write
            // there.
            unsafe { w.orc().bits(orc) });

        Spim(spim)
    }

    /// Internal helper function to setup and execute SPIM DMA transfer
    fn do_spi_dma_transfer<CSFun>(
        &mut self,
        tx: DmaSlice,
        rx: DmaSlice,
        mut cs_n: CSFun,
    ) -> Result<(), Error>
    where
        CSFun: FnMut(bool),
    {
        let (tx_data_ptr, tx_len) = (tx.ptr, tx.len);
        let (rx_data_ptr, rx_len) = (rx.ptr, rx.len);

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(SeqCst);
        // set CS inactive
        cs_n(false);
        // Set up the DMA write
        self.0
            .txd
            .ptr
            .write(|w| unsafe { w.ptr().bits(tx_data_ptr) });
        self.0.txd.maxcnt.write(|w|
            // Note that that nrf52840 maxcnt is a wider
            // type than a u8, so we use a `_` cast rather than a `u8` cast.
            // The MAXCNT field is thus at least 8 bits wide and accepts the full
            // range of values that fit in a `u8`.
            unsafe { w.maxcnt().bits(tx_len as _ ) });

        // Set up the DMA read
        self.0.rxd.ptr.write(|w|
            // This is safe for the same reasons that writing to TXD.PTR is
            // safe. Please refer to the explanation there.
            unsafe { w.ptr().bits(rx_data_ptr ) });
        self.0.rxd.maxcnt.write(|w|
            // This is safe for the same reasons that writing to TXD.MAXCNT is
            // safe. Please refer to the explanation there.
            unsafe { w.maxcnt().bits(rx_len as _) });

        // Set CS active
        cs_n(true);
        // Start SPI transaction
        self.0.tasks_start.write(|w|
            // `1` is a valid value to write to task registers.
            unsafe { w.bits(1) });

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);

        // Wait for END event
        //
        // This event is triggered once both transmitting and receiving are
        // done.
        while self.0.events_end.read().bits() == 0 {}

        // Reset the event, otherwise it will always read `1` from now on.
        self.0.events_end.write(|w| w);

        // Transfer done - set cs inactive
        cs_n(false);
        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // after all possible DMA actions have completed
        compiler_fence(SeqCst);

        if self.0.txd.amount.read().bits() != tx_len {
            return Err(Error::Transmit);
        }
        if self.0.rxd.amount.read().bits() != rx_len {
            return Err(Error::Receive);
        }
        Ok(())
    }

    /// Read from an SPI slave
    ///
    /// This method is deprecated. Consider using `transfer` or `transfer_split`
    #[inline(always)]
    pub fn read(
        &mut self,
        chip_select: &mut Pin<Output<PushPull>>,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.transfer_split_uneven(chip_select, tx_buffer, rx_buffer)
    }

    /// Read and write from a SPI slave, using a single buffer
    ///
    /// This method implements a complete read transaction, which consists of
    /// the master transmitting what it wishes to read, and the slave responding
    /// with the requested data.
    ///
    /// Uses the provided chip select pin to initiate the transaction. Transmits
    /// all bytes in `buffer`, then receives an equal number of bytes.
    pub fn transfer(
        &mut self,
        chip_select: &mut Pin<Output<PushPull>>,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        ram_slice_check(buffer)?;

        for chunk in buffer.chunks(EASY_DMA_SIZE) {
            self.do_spi_dma_transfer(
                DmaSlice::from_slice(chunk),
                DmaSlice::from_slice(chunk),
                |cs| {
                    if cs {
                        chip_select.set_low()
                    } else {
                        chip_select.set_high()
                    }
                },
            )?;
        }

        Ok(())
    }

    /// Read and write from a SPI slave, using separate read and write buffers
    ///
    /// This method implements a complete read transaction, which consists of
    /// the master transmitting what it wishes to read, and the slave responding
    /// with the requested data.
    ///
    /// Uses the provided chip select pin to initiate the transaction. Transmits
    /// all bytes in `tx_buffer`, then receives bytes until `rx_buffer` is full.
    ///
    /// If `tx_buffer.len() != rx_buffer.len()`, the transaction will stop at the
    /// smaller of either buffer.
    pub fn transfer_split_even(
        &mut self,
        chip_select: &mut Pin<Output<PushPull>>,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
    ) -> Result<(), Error> {
        ram_slice_check(tx_buffer)?;
        ram_slice_check(rx_buffer)?;

        let txi = tx_buffer.chunks(EASY_DMA_SIZE);
        let rxi = rx_buffer.chunks_mut(EASY_DMA_SIZE);
        let iter = txi.zip(rxi);

        for (t, r) in iter {
            self.do_spi_dma_transfer(DmaSlice::from_slice(t), DmaSlice::from_slice(r), |cs| {
                if cs {
                    chip_select.set_low()
                } else {
                    chip_select.set_high()
                }
            })?;
        }

        Ok(())
    }

    /// Read and write from a SPI slave, using separate read and write buffers
    ///
    /// This method implements a complete read transaction, which consists of
    /// the master transmitting what it wishes to read, and the slave responding
    /// with the requested data.
    ///
    /// Uses the provided chip select pin to initiate the transaction. Transmits
    /// all bytes in `tx_buffer`, then receives bytes until `rx_buffer` is full.
    ///
    /// This method is more complicated than the other `transfer` methods because
    /// it is allowed to perform transactions where `tx_buffer.len() != rx_buffer.len()`.
    /// If this occurs, extra incoming bytes will be discarded, OR extra outgoing bytes
    /// will be filled with the `orc` value.
    pub fn transfer_split_uneven(
        &mut self,
        chip_select: &mut Pin<Output<PushPull>>,
        tx_buffer: &[u8],
        rx_buffer: &mut [u8],
    ) -> Result<(), Error> {
        ram_slice_check(tx_buffer)?;
        ram_slice_check(rx_buffer)?;
        // For the tx and rx, we want to return Some(chunk)
        // as long as there is data to send. We then chain a repeat to
        // the end so once all chunks have been exhausted, we will keep
        // getting Nones out of the iterators
        let txi = tx_buffer
            .chunks(EASY_DMA_SIZE)
            .map(|c| Some(c))
            .chain(repeat_with(|| None));

        let rxi = rx_buffer
            .chunks_mut(EASY_DMA_SIZE)
            .map(|c| Some(c))
            .chain(repeat_with(|| None));

        // We then chain the iterators together, and once BOTH are feeding
        // back Nones, then we are done sending and receiving
        let iters = txi
            .zip(rxi)
            .take_while(|(t, r)| t.is_some() && r.is_some())
            // We also turn the slices into either a DmaSlice (if there was data), or a null
            // DmaSlice (if there is no data)
            .map(|(t, r)| {
                (
                    t.map(|t| DmaSlice::from_slice(t))
                        .unwrap_or_else(|| DmaSlice::null()),
                    r.map(|r| DmaSlice::from_slice(r))
                        .unwrap_or_else(|| DmaSlice::null()),
                )
            });

        for (t, r) in iters {
            self.do_spi_dma_transfer(t, r, |cs| {
                if cs {
                    chip_select.set_low()
                } else {
                    chip_select.set_high()
                }
            })?;
        }

        Ok(())
    }

    /// Write to an SPI slave
    ///
    /// This method uses the provided chip select pin to initiate the
    /// transaction, then transmits all bytes in `tx_buffer`. All incoming
    /// bytes are discarded.
    pub fn write(
        &mut self,
        chip_select: &mut Pin<Output<PushPull>>,
        tx_buffer: &[u8],
    ) -> Result<(), Error> {
        ram_slice_check(tx_buffer)?;
        self.transfer_split_uneven(chip_select, tx_buffer, &mut [0u8; 0])
    }

    /// Return the raw interface to the underlying SPIM peripheral
    pub fn free(self) -> T {
        self.0
    }
}

/// GPIO pins for SPIM interface
pub struct Pins {
    /// SPI clock
    pub sck: Pin<Output<PushPull>>,

    /// MOSI Master out, slave in
    /// None if unused
    pub mosi: Option<Pin<Output<PushPull>>>,

    /// MISO Master in, slave out
    /// None if unused
    pub miso: Option<Pin<Input<Floating>>>,
}

#[derive(Debug)]
pub enum Error {
    TxBufferTooLong,
    RxBufferTooLong,
    /// EasyDMA can only read from data memory, read only buffers in flash will fail
    DMABufferNotInDataMemory,
    Transmit,
    Receive,
}

fn ram_slice_check(slice: &[u8]) -> Result<(), Error> {
    if slice_in_ram(slice) {
        Ok(())
    } else {
        Err(Error::DMABufferNotInDataMemory)
    }
}
