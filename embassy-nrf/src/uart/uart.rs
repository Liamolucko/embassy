#![macro_use]

use core::future::Future;
use core::marker::PhantomData;

use embassy::interrupt::Interrupt;
use embassy::traits::uart;
use embassy::util::Unborrow;
use embassy_hal_common::unborrow;

use crate::gpio::sealed::Pin;
use crate::gpio::{self, OptionalPin as GpioOptionalPin, Pin as GpioPin};
use crate::pac;
use crate::util::io::{read, write, ByteRead, ByteWrite};

// Re-export SVD variants to allow user to directly set values.
pub use pac::uart0::{baudrate::BAUDRATE_A as Baudrate, config::PARITY_A as Parity};

use super::Config;

/// Interface to the UART peripheral
pub struct Uart<'d, T: Instance> {
    irq: T::Interrupt,
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Uart<'d, T> {
    /// Creates the interface to a UART instance.
    /// Sets the baud rate, parity and assigns the pins to the UART peripheral.
    pub fn new(
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

        // Enable
        r.enable.write(|w| w.enable().enabled());

        Self {
            irq,
            phantom: PhantomData,
        }
    }
}

impl<'a, T: Instance> Drop for Uart<'a, T> {
    fn drop(&mut self) {
        info!("uart drop");

        let r = T::regs();

        // Wait for rxto, if needed.
        while r.events_rxto.read().bits() == 0 {}

        // Finally we can disable!
        r.enable.write(|w| w.enable().disabled());

        gpio::deconfigure_pin(r.pselrxd.read().bits());
        gpio::deconfigure_pin(r.pseltxd.read().bits());
        gpio::deconfigure_pin(r.pselrts.read().bits());
        gpio::deconfigure_pin(r.pselcts.read().bits());

        info!("uart drop: done");
    }
}

impl<'d, T: Instance> ByteRead for Uart<'d, T> {
    type Interrupt = T::Interrupt;

    #[inline]
    fn irq(&mut self) -> &mut Self::Interrupt {
        &mut self.irq
    }

    #[inline]
    fn enable_irq(&self) {
        T::regs().intenset.write(|w| w.rxdrdy().set())
    }

    #[inline]
    fn disable_irq(&self) {
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
    fn clear_event(&self) {
        T::regs().events_rxdrdy.reset()
    }

    #[inline]
    fn next_byte(&self) -> u8 {
        T::regs().rxd.read().rxd().bits()
    }
}

impl<'d, T: Instance> uart::Read for Uart<'d, T> {
    #[rustfmt::skip]
    type ReadFuture<'a> where 'd: 'a = impl Future<Output = Result<(), uart::Error>> + 'a;

    #[inline]
    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
        // SAFETY: The safety contract of `irq_read` is forwarded to `Uart::new`.
        async move {
            unsafe {
                read(self, buf).await;
            }
            Ok(())
        }
    }
}

impl<'d, T: Instance> ByteWrite for Uart<'d, T> {
    type Interrupt = T::Interrupt;

    #[inline]
    fn irq(&mut self) -> &mut Self::Interrupt {
        &mut self.irq
    }

    #[inline]
    fn enable_irq(&self) {
        T::regs().intenset.write(|w| w.txdrdy().set())
    }

    #[inline]
    fn disable_irq(&self) {
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
    fn clear_event(&self) {
        T::regs().events_txdrdy.reset()
    }

    #[inline]
    fn set_next_byte(&self, byte: u8) {
        T::regs().txd.write(|w| unsafe { w.txd().bits(byte) })
    }
}

impl<'d, T: Instance> uart::Write for Uart<'d, T> {
    #[rustfmt::skip]
    type WriteFuture<'a> where Self: 'a = impl Future<Output = Result<(), uart::Error>> + 'a;

    #[inline]
    fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
        // SAFETY: `irq_write`'s safety contract is forwarded to `Uart::new`.
        async move {
            unsafe {
                write(self, buf).await;
            }
            Ok(())
        }
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        fn regs() -> &'static pac::uart0::RegisterBlock;
    }
}

pub trait Instance: Unborrow<Target = Self> + sealed::Instance + 'static + Send + Sync {
    type Interrupt: Interrupt + Unpin + Sync;
}

macro_rules! impl_uart {
    ($type:ident, $pac_type:ident, $irq:ident) => {
        impl crate::uart::sealed::Instance for peripherals::$type {
            fn regs() -> &'static pac::uart0::RegisterBlock {
                unsafe { &*pac::$pac_type::ptr() }
            }
        }
        impl crate::uart::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::$irq;
        }
    };
}
