//! HAL interface to the TWIS peripheral.
//!
//! See product specification:
//!
//! - nrf52832: Section 34
//! - nrf52840: Section 6.32

use crate::gpio::{Floating, Input, Pin};
use crate::target::{twis0, P0, TWIS0};
use crate::target_constants::EASY_DMA_SIZE;
use core::ops::Deref;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

#[cfg(feature = "52840")]
use crate::target::TWIS1;

/// Interface to a TWIS instance.
pub struct Twis<T>(T);

impl<T> Twis<T>
where
    T: Instance,
{
    /// Interface to a TWIS instance.
    ///
    /// Currently, this only supports one address per instance,
    /// although more may be supported by the hardware. Additionally,
    /// the TWIS instances share the same address space as other
    /// peripherals, such as SPIM, SPIS, and TWIM, so a given numbered
    /// instance of a TWIS may not be used at the same time as the
    /// same numbered instance of a conflicting peripheral. For
    /// example, TWIM1 may not be used at the same time as SPIM1, as
    /// they share the same address space.
    pub fn new(address: u8, twis: T, pins: Pins) -> Self {
        // Configure PSEL.SCL, PSEL.SDA, CONFIG, and ADDRESS[n]
        // registers before enabling through ENABLE rigster.

        for &pin in &[pins.scl.pin, pins.sda.pin] {
            unsafe { &*P0::ptr() }.pin_cnf[pin as usize].write(|w| {
                w.dir()
                    .input()
                    .input()
                    .connect()
                    .pull()
                    .pullup()
                    .drive()
                    .s0d1()
                    .sense()
                    .disabled()
            });
        }

        twis.psel.scl.write(|w| {
            let w = unsafe { w.pin().bits(pins.scl.pin) };
            #[cfg(feature = "52840")]
            let w = w.port().bit(pins.scl.port);
            w.connect().connected()
        });
        twis.psel.sda.write(|w| {
            let w = unsafe { w.pin().bits(pins.sda.pin) };
            #[cfg(feature = "52840")]
            let w = w.port().bit(pins.sda.port);
            w.connect().connected()
        });

        // TWIS only supports 7 bit addressing.
        debug_assert!(address & !(0x7f) == 0);
        twis.address[0].write(|w| unsafe { w.bits(address.into()) });
        twis.config.write(|w| w.address0().enabled());
        twis.enable.write(|w| w.enable().enabled());

        // TWIS instance can have up to two addresses.
        Self(twis)
    }

    /// Allow `event` to generate an interrupt.
    pub fn enable_interrupt(&mut self, event: TwisInterrupt) {
        match event {
            TwisInterrupt::Stopped => self.0.intenset.write(|w| w.stopped().set_bit()),
            TwisInterrupt::Error => self.0.intenset.write(|w| w.error().set_bit()),
            TwisInterrupt::RxStarted => self.0.intenset.write(|w| w.rxstarted().set_bit()),
            TwisInterrupt::TxStarted => self.0.intenset.write(|w| w.txstarted().set_bit()),
            TwisInterrupt::Write => self.0.intenset.write(|w| w.write().set_bit()),
            TwisInterrupt::Read => self.0.intenset.write(|w| w.read().set_bit()),
        }
    }

    /// Prevent `event` from generating an interrupt.
    pub fn disable_interrupt(&mut self, event: TwisInterrupt) {
        match event {
            TwisInterrupt::Stopped => self.0.intenclr.write(|w| w.stopped().set_bit()),
            TwisInterrupt::Error => self.0.intenclr.write(|w| w.error().set_bit()),
            TwisInterrupt::RxStarted => self.0.intenclr.write(|w| w.rxstarted().set_bit()),
            TwisInterrupt::TxStarted => self.0.intenclr.write(|w| w.txstarted().set_bit()),
            TwisInterrupt::Write => self.0.intenclr.write(|w| w.write().set_bit()),
            TwisInterrupt::Read => self.0.intenclr.write(|w| w.read().set_bit()),
        }
    }

    pub fn get_event_triggered(&mut self, event: TwisInterrupt, clear_on_read: bool) -> bool {
        let mut orig = 0;
        let set_val = if clear_on_read { 0 } else { 1 };
        match event {
            TwisInterrupt::Stopped => self.0.events_stopped.modify(|r, w| {
                orig = r.bits();
                unsafe { w.bits(set_val) }
            }),
            TwisInterrupt::Error => self.0.events_error.modify(|r, w| {
                orig = r.bits();
                unsafe { w.bits(set_val) }
            }),
            TwisInterrupt::RxStarted => self.0.events_rxstarted.modify(|r, w| {
                orig = r.bits();
                unsafe { w.bits(set_val) }
            }),
            TwisInterrupt::TxStarted => self.0.events_txstarted.modify(|r, w| {
                orig = r.bits();
                unsafe { w.bits(set_val) }
            }),
            TwisInterrupt::Write => self.0.events_write.modify(|r, w| {
                orig = r.bits();
                unsafe { w.bits(set_val) }
            }),
            TwisInterrupt::Read => self.0.events_read.modify(|r, w| {
                orig = r.bits();
                unsafe { w.bits(set_val) }
            }),
        };

        orig == 1
    }

    // TODO: this probably shouldn't be pub? Interrupt interfaces are
    // tough.
    pub fn prepare_read(&mut self, buffer: &mut [u8]) {
        // Give a pointer to `buffer` and its length to the DMA system
        // and start the read.
        self.0
            .rxd
            .ptr
            .write(|w| unsafe { w.ptr().bits(buffer.as_mut_ptr() as _) });
        self.0
            .rxd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(buffer.len() as _) });
        self.0
            .tasks_preparerx
            .write(|w| w.tasks_preparerx().set_bit());
    }

    /// Read from the I²C master.
    ///
    /// On success, returns the number of bytes read.
    ///
    /// The buffer length is limited by the DMA transfer size.
    ///  - 255 bytes on the nRF52832
    ///  - 65535 bytes on the nRF52840
    pub fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if buffer.len() > EASY_DMA_SIZE {
            return Err(Error::RxBufferTooLong);
        }

        // We can only DMA out of RAM
        if !crate::slice_in_ram(buffer) {
            return Err(Error::BufferNotInRAM);
        }

        // TODO: Can make this non-blocking by encoding current state
        // and dispatching accordingly:
        //  - prepare buffer for dma
        //  - wait for event_stopped
        //
        // but what happens on abort? we still have the pointer in the
        // dma system and it can be written to outside of our
        // control. can't rely on `Drop`, either.

        // Conservative compiler fence to prevent optimizations that
        // do not take in to account actions by DMA. The fence has
        // been placed here, before any DMA action has started
        compiler_fence(SeqCst);

        self.prepare_read(buffer);

        // Give a pointer to `buffer` and its length to the DMA system
        // and start the read.
        self.0
            .rxd
            .ptr
            .write(|w| unsafe { w.ptr().bits(buffer.as_mut_ptr() as _) });
        self.0
            .rxd
            .maxcnt
            .write(|w| unsafe { w.maxcnt().bits(buffer.len() as _) });
        self.0
            .tasks_preparerx
            .write(|w| w.tasks_preparerx().set_bit());

        // Wait for the read to end.
        while self.0.events_stopped.read().events_stopped().bit_is_clear() {}

        // Conservative compiler fence to prevent optimizations that
        // do not take in to account actions by DMA. The fence has
        // been placed here, after all possible DMA actions have
        // completed
        compiler_fence(SeqCst);

        if self.0.events_error.read().events_error().bit_is_set() {
            if self.0.errorsrc.read().overflow().bit_is_set() {
                Err(Error::Overflow)
            } else if self.0.errorsrc.read().dnack().bit_is_set() {
                Err(Error::DataNack)
            } else if self.0.errorsrc.read().overread().bit_is_set() {
                Err(Error::OverRead)
            } else {
                Err(Error::Receive)
            }
        } else {
            Ok(self.0.rxd.amount.read().bits() as _)
        }
    }

    // TODO: this is such a hack. It's starting to look like
    // interrupt-driven stuff isn't going to happen in this framework.
    pub fn amount(&self) -> usize {
        self.0.rxd.amount.read().bits() as _
    }

    // TODO: read_then_write. Not sure how this should work? maybe
    // take a closure? Not sure this is even necessary if there's
    // already a write method, tbh.
}

/// The pins used by the TWIS peripheral.
pub struct Pins {
    /// Serial Clock Line.
    pub scl: Pin<Input<Floating>>,

    /// Serial Data Line.
    pub sda: Pin<Input<Floating>>,
}

/// Events that may be generated by a Twis instance.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TwisInterrupt {
    Stopped,
    Error,
    RxStarted,
    TxStarted,
    Write,
    Read,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Error {
    BufferNotInRAM,
    TxBufferTooLong,
    RxBufferTooLong,
    Transmit,
    Receive,
    Overflow,
    DataNack,
    OverRead,
}
/// Implemented by all TWIS instances.
pub trait Instance: Deref<Target = twis0::RegisterBlock> {}
impl Instance for TWIS0 {}

#[cfg(feature = "52840")]
impl Instance for TWIS1 {}
