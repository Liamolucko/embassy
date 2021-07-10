#![macro_use]

use core::future::Future;
use core::marker::PhantomData;

use embassy::interrupt::Interrupt;
use embassy::interrupt::InterruptExt;
use embassy::traits::uart;
use embassy::util::Unborrow;
use embassy_extras::unborrow;
use futures::FutureExt;

use crate::gpio;
use crate::gpio::sealed::Pin;
use crate::gpio::OptionalPin as GpioOptionalPin;
use crate::gpio::Pin as GpioPin;
use crate::pac;
use crate::util::irq_read;
use crate::util::irq_write;
use crate::util::IrqRead;
use crate::util::IrqWrite;

// Re-export SVD variants to allow user to directly set values.
pub use pac::uart0::baudrate::BAUDRATE_A as Baudrate;
pub use pac::uart0::config::PARITY_A as Parity;

use super::Config;

/// Interface to the UART peripheral
pub struct Uart<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Uart<'d, T> {
    /// Creates the interface to a UART instance.
    /// Sets the baud rate, parity and assigns the pins to the UART peripheral.
    ///
    /// # Safety
    ///
    /// The returned API is safe unless you use `mem::forget` (or similar safe mechanisms)
    /// on stack allocated buffers which which have been passed to [`send()`](Uarte::send)
    /// or [`receive`](Uarte::receive).
    #[allow(unused_unsafe)]
    pub unsafe fn new(
        _uarte: impl Unborrow<Target = T> + 'd,
        irq: impl Unborrow<Target = T::Interrupt> + 'd,
        rxd: impl Unborrow<Target = impl GpioPin> + 'd,
        txd: impl Unborrow<Target = impl GpioPin> + 'd,
        cts: impl Unborrow<Target = impl GpioOptionalPin> + 'd,
        rts: impl Unborrow<Target = impl GpioOptionalPin> + 'd,
        config: Config,
    ) -> Self {
        unborrow!(irq, rxd, txd, cts, rts);

        let r = T::regs();

        assert!(r.enable.read().enable().is_disabled());

        rxd.conf().write(|w| w.input().connect().drive().h0h1());
        r.pselrxd.write(|w| unsafe { w.bits(rxd.psel_bits()) });

        txd.set_high();
        txd.conf().write(|w| w.dir().output().drive().h0h1());
        r.pseltxd.write(|w| unsafe { w.bits(txd.psel_bits()) });

        if let Some(pin) = rts.pin_mut() {
            pin.set_high();
            pin.conf().write(|w| w.dir().output().drive().h0h1());
        }
        r.pselcts.write(|w| unsafe { w.bits(cts.psel_bits()) });

        if let Some(pin) = cts.pin_mut() {
            pin.conf().write(|w| w.input().connect().drive().h0h1());
        }
        r.pselrts.write(|w| unsafe { w.bits(rts.psel_bits()) });

        // Configure
        let hardware_flow_control = match (rts.pin().is_some(), cts.pin().is_some()) {
            (false, false) => false,
            (true, true) => true,
            _ => panic!("RTS and CTS pins must be either both set or none set."),
        };
        r.config.write(|w| {
            w.hwfc().bit(hardware_flow_control);
            w.parity().variant(config.parity);
            w
        });
        r.baudrate.write(|w| w.baudrate().variant(config.baudrate));

        // Disable all interrupts
        r.intenclr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });

        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        // Enable
        r.enable.write(|w| w.enable().enabled());

        Self {
            phantom: PhantomData,
        }
    }

    fn on_interrupt(_: *mut ()) {
        trace!("uart: interrupt fired");

        let r = T::regs();

        if r.events_rxdrdy.read().bits() != 0 {
            // Defer to `IrqRead` (it handles clearing the event).
            <Self as IrqRead>::on_irq();
        }

        if r.events_txdrdy.read().bits() != 0 {
            <Self as IrqWrite>::on_irq();
        }

        // This interrupt is just enabled to wake the CPU from WFE,
        // so we don't need to do anything other than disable the interrupt
        // so it doesn't repeatedly fire.
        if r.events_rxto.read().bits() != 0 {
            r.intenclr.write(|w| w.rxto().clear());
        }
    }
}

impl<'a, T: Instance> Drop for Uart<'a, T> {
    fn drop(&mut self) {
        info!("uart drop");

        let r = T::regs();

        // Wait for rxto, if needed.
        // (The interrupt firing will wake the CPU from WFE)
        r.intenset.write(|w| w.rxto().set());
        while r.events_rxto.read().bits() == 0 {
            info!("uart drop: wfe");
            cortex_m::asm::wfe();
        }

        cortex_m::asm::sev();

        // Finally we can disable!
        r.enable.write(|w| w.enable().disabled());

        gpio::deconfigure_pin(r.pselrxd.read().bits());
        gpio::deconfigure_pin(r.pseltxd.read().bits());
        gpio::deconfigure_pin(r.pselrts.read().bits());
        gpio::deconfigure_pin(r.pselcts.read().bits());

        info!("uart drop: done");
    }
}

impl<'d, T: Instance> IrqRead for Uart<'d, T> {
    #[inline]
    fn state() -> &'static crate::util::IrqIoState {
        T::rx_state()
    }

    #[inline]
    fn enable_irq(&self) {
        T::regs().intenset.write(|w| w.rxdrdy().set())
    }

    #[inline]
    fn disable_irq() {
        T::regs().intenclr.write(|w| w.rxdrdy().clear())
    }

    #[inline]
    fn start(&self) {
        T::regs().tasks_startrx.write(|w| unsafe { w.bits(1) })
    }

    #[inline]
    fn stop(&self) {
        T::regs().tasks_stoprx.write(|w| unsafe { w.bits(1) })
    }

    #[inline]
    fn clear_event() {
        T::regs().events_rxdrdy.reset()
    }

    #[inline]
    fn next_byte() -> u8 {
        T::regs().rxd.read().rxd().bits()
    }
}

impl<'d, T: Instance> uart::Read for Uart<'d, T> {
    #[rustfmt::skip]
    type ReadFuture<'a> where 'd: 'a = impl Future<Output = Result<(), uart::Error>> + 'a;

    #[inline]
    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
        // SAFETY: The safety contract of `irq_read` is forwarded to `Uart::new`.
        unsafe { irq_read(self, buf) }.map(|_| Ok(()))
    }
}

impl<'d, T: Instance> IrqWrite for Uart<'d, T> {
    #[inline]
    fn state() -> &'static crate::util::IrqIoState {
        T::tx_state()
    }

    #[inline]
    fn enable_irq(&self) {
        T::regs().intenset.write(|w| w.txdrdy().set())
    }

    #[inline]
    fn disable_irq() {
        T::regs().intenclr.write(|w| w.txdrdy().clear())
    }

    #[inline]
    fn start(&self) {
        T::regs().tasks_starttx.write(|w| unsafe { w.bits(1) })
    }

    #[inline]
    fn stop(&self) {
        T::regs().tasks_stoptx.write(|w| unsafe { w.bits(1) })
    }

    #[inline]
    fn event_fired(&self) -> bool {
        T::regs().events_txdrdy.read().bits() != 0
    }

    #[inline]
    fn clear_event() {
        T::regs().events_txdrdy.reset()
    }

    #[inline]
    fn set_next_byte(byte: u8) {
        T::regs().txd.write(|w| unsafe { w.txd().bits(byte) })
    }
}

impl<'d, T: Instance> uart::Write for Uart<'d, T> {
    #[rustfmt::skip]
    type WriteFuture<'a> where 'd: 'a = impl Future<Output = Result<(), uart::Error>>;

    #[inline]
    fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
        // SAFETY: `irq_write`'s safety contract is forwarded to `Uart::new`.
        unsafe { irq_write(self, buf) }.map(|_| Ok(()))
    }
}

pub(crate) mod sealed {
    use crate::util::IrqIoState;

    use super::*;

    pub trait Instance {
        fn regs() -> &'static pac::uart0::RegisterBlock;
        fn rx_state() -> &'static IrqIoState;
        fn tx_state() -> &'static IrqIoState;
    }
}

pub trait Instance: Unborrow<Target = Self> + sealed::Instance + 'static {
    type Interrupt: Interrupt;
}

macro_rules! impl_uart {
    ($type:ident, $pac_type:ident, $irq:ident) => {
        impl crate::uart::sealed::Instance for peripherals::$type {
            fn regs() -> &'static pac::uart0::RegisterBlock {
                unsafe { &*pac::$pac_type::ptr() }
            }
            fn rx_state() -> &'static crate::util::IrqIoState {
                static STATE: crate::util::IrqIoState = crate::util::IrqIoState::new();
                &STATE
            }
            fn tx_state() -> &'static crate::util::IrqIoState {
                static STATE: crate::util::IrqIoState = crate::util::IrqIoState::new();
                &STATE
            }
        }
        impl crate::uart::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::$irq;
        }
    };
}
