#![macro_use]

use core::future::Future;
use core::marker::PhantomData;

use embassy::interrupt::Interrupt;
use embassy::interrupt::InterruptExt;
use embassy::traits::uart::{self, Error};
use embassy::util::{Unborrow, WakerRegistration};
use embassy_hal_common::peripheral::{PeripheralMutex, StateStorage};
use embassy_hal_common::unborrow;
use futures::FutureExt;

use crate::gpio::sealed::Pin;
use crate::gpio::{self, OptionalPin as GpioOptionalPin, Pin as GpioPin};
use crate::pac;
use crate::ppi::{AnyConfigurableChannel, ConfigurableChannel, Event, Ppi, Task};
use crate::timer::{Frequency, Instance as TimerInstance, SupportsBitmode, Timer};
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

        // Set an interrupt handler which will just disable itself when RXTO is fired.
        self.irq.set_handler(|_| {
            let r = T::regs();
            r.intenclr.write(|w| w.rxto().clear());
        });

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
        unsafe { read(self, buf) }.map(|_| Ok(()))
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
        unsafe { write(self, buf) }.map(|_| Ok(()))
    }
}

/// Interface to an UARTE peripheral that uses an additional timer and two PPI channels,
/// allowing it to implement the ReadUntilIdle trait.
pub struct UartWithIdle<'d, U: Instance, T: TimerInstance + SupportsBitmode<u16>> {
    uart: Uart<'d, U>,
    timer: Timer<'d, T, u16>,
    ppi_ch1: Ppi<'d, AnyConfigurableChannel>,
    _ppi_ch2: Ppi<'d, AnyConfigurableChannel>,
}

impl<'d, U: Instance, T: TimerInstance + SupportsBitmode<u16>> UartWithIdle<'d, U, T> {
    /// Creates the interface to a UARTE instance.
    /// Sets the baud rate, parity and assigns the pins to the UARTE peripheral.
    ///
    /// # Safety
    ///
    /// The returned API is safe unless you use `mem::forget` (or similar safe mechanisms)
    /// on stack allocated buffers which which have been passed to [`send()`](Uarte::send)
    /// or [`receive`](Uarte::receive).
    #[allow(unused_unsafe)]
    pub unsafe fn new(
        uarte: impl Unborrow<Target = U> + 'd,
        timer: impl Unborrow<Target = T> + 'd,
        ppi_ch1: impl Unborrow<Target = impl ConfigurableChannel> + 'd,
        ppi_ch2: impl Unborrow<Target = impl ConfigurableChannel> + 'd,
        irq: impl Unborrow<Target = U::Interrupt> + 'd,
        rxd: impl Unborrow<Target = impl GpioPin> + 'd,
        txd: impl Unborrow<Target = impl GpioPin> + 'd,
        cts: impl Unborrow<Target = impl GpioOptionalPin> + 'd,
        rts: impl Unborrow<Target = impl GpioOptionalPin> + 'd,
        config: Config,
    ) -> Self {
        let baudrate = config.baudrate;
        let uart = Uart::new(uarte, irq, rxd, txd, cts, rts, config);
        let mut timer: Timer<T, u16> = Timer::new_irqless(timer);

        unborrow!(ppi_ch1, ppi_ch2);

        let r = U::regs();

        // BAUDRATE register values are `baudrate * 2^32 / 16000000`
        // source: https://devzone.nordicsemi.com/f/nordic-q-a/391/uart-baudrate-register-values
        //
        // We want to stop RX if line is idle for 2 bytes worth of time
        // That is 20 bits (each byte is 1 start bit + 8 data bits + 1 stop bit)
        // This gives us the amount of 16M ticks for 20 bits.
        let timeout = (0x8000_0000 / (baudrate as u32 / 40)) as u16;

        timer.set_frequency(Frequency::F16MHz);
        timer.cc(0).write(timeout);
        timer.cc(0).short_compare_clear();
        timer.cc(0).short_compare_stop();

        let mut ppi_ch1 = Ppi::new(ppi_ch1.degrade_configurable());
        ppi_ch1.set_event(Event::from_reg(&r.events_rxdrdy));
        ppi_ch1.set_task(timer.task_clear());
        ppi_ch1.enable();

        let mut ppi_ch2 = Ppi::new(ppi_ch2.degrade_configurable());
        ppi_ch2.set_event(timer.cc(0).event_compare());
        ppi_ch2.set_task(Task::from_reg(&r.tasks_stoprx));
        ppi_ch2.enable();

        Self {
            uart,
            timer,
            ppi_ch1,
            _ppi_ch2: ppi_ch2,
        }
    }
}

impl<'d, U, T> uart::ReadUntilIdle for UartWithIdle<'d, U, T>
where
    U: Instance,
    T: TimerInstance + SupportsBitmode<u16>,
{
    #[rustfmt::skip]
    type ReadUntilIdleFuture<'a> where Self: 'a = impl Future<Output = Result<usize, Error>> + 'a;
    fn read_until_idle<'a>(&'a mut self, rx_buffer: &'a mut [u8]) -> Self::ReadUntilIdleFuture<'a> {
        if rx_buffer.len() == 0 {
            return; // Nothing to fill
        }

        // SAFETY: This future will only live as long as the reference to the original `irq`;
        // we only need ownership to pass it to `PeripheralMutex`.
        let irq = unsafe { (&mut self.uart.irq).unborrow() };

        let mut storage = StateStorage::new();

        // SAFETY: `UartWithIdle::new`'s safety contract makes sure the destructor will be run.
        let mut mutex = unsafe {
            PeripheralMutex::new_unchecked(irq, &mut storage, || ReadState {
                reader: &*self,
                buf_iter: rx_buffer.iter_mut(),
                waker: WakerRegistration::new(),
            })
        };

        let r = U::regs();

        r.intenset.write(|w| w.rxdrdy().set());
        r.tasks_startrx.write(|w| unsafe { w.bits(1) });

        let on_drop = OnDrop::new(|| {
            // Disable the interrupt first, so we don't waste its time with events it'll just ignore.
            reader.disable_irq();
            reader.stop();
        });

        poll_fn(|cx| {
            mutex.with(|state| {
                state.waker.register(cx.waker());

                if state.buf_iter.len() == 0 {
                    // We're done.
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await;

        // Trigger the teardown
        drop(on_drop);
    }
}

impl<'d, U: Instance, T: TimerInstance + SupportsBitmode<u16>> uart::Read
    for UartWithIdle<'d, U, T>
{
    #[rustfmt::skip]
    type ReadFuture<'a> where Self: 'a = impl Future<Output = Result<(), Error>> + 'a;
    fn read<'a>(&'a mut self, rx_buffer: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            self.ppi_ch1.disable();
            let result = self.uart.read(rx_buffer).await;
            self.ppi_ch1.enable();
            result
        }
    }
}

impl<'d, U: Instance, T: TimerInstance + SupportsBitmode<u16>> uart::Write
    for UartWithIdle<'d, U, T>
{
    #[rustfmt::skip]
    type WriteFuture<'a> where Self: 'a = impl Future<Output = Result<(), Error>> + 'a;

    fn write<'a>(&'a mut self, tx_buffer: &'a [u8]) -> Self::WriteFuture<'a> {
        self.uart.write(tx_buffer)
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
