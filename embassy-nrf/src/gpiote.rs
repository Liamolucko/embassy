use core::convert::Infallible;
use core::task::Poll;
use embassy::interrupt::{Interrupt, InterruptExt};
use embassy::waitqueue::AtomicWaker;
use embassy_hal_common::unsafe_impl_unborrow;
use embedded_hal::digital::v2::{InputPin, StatefulOutputPin};
use futures::future::poll_fn;

use crate::gpio::{Input, Output, Pin as GpioPin};
use crate::pac;
use crate::ppi::{Event, Task};
use crate::{interrupt, peripherals};

pub const CHANNEL_COUNT: usize = 8;

#[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
pub const PIN_COUNT: usize = 48;
#[cfg(not(any(feature = "nrf52833", feature = "nrf52840")))]
pub const PIN_COUNT: usize = 32;

#[allow(clippy::declare_interior_mutable_const)]
const NEW_AW: AtomicWaker = AtomicWaker::new();
static CHANNEL_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [NEW_AW; CHANNEL_COUNT];
#[cfg(not(feature = "nrf51"))]
static PORT_WAKERS: [AtomicWaker; PIN_COUNT] = [NEW_AW; PIN_COUNT];

pub enum InputChannelPolarity {
    None,
    HiToLo,
    LoToHi,
    Toggle,
}

/// Polarity of the `task out` operation.
pub enum OutputChannelPolarity {
    Set,
    Clear,
    Toggle,
}

fn regs() -> &'static pac::gpiote::RegisterBlock {
    cfg_if::cfg_if! {
        if #[cfg(any(feature="nrf5340-app-s", feature="nrf9160-s"))] {
            unsafe { &*pac::GPIOTE0::ptr() }
        } else if #[cfg(any(feature="nrf5340-app-ns", feature="nrf9160-ns"))] {
            unsafe { &*pac::GPIOTE1::ptr() }
        } else {
            unsafe { &*pac::GPIOTE::ptr() }
        }
    }
}

pub(crate) fn init(irq_prio: crate::interrupt::Priority) {
    #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
    let ports = unsafe { &[&*pac::P0::ptr(), &*pac::P1::ptr()] };
    #[cfg(not(any(feature = "nrf51", feature = "nrf52833", feature = "nrf52840")))]
    let ports = unsafe { &[&*pac::P0::ptr()] };

    #[cfg(not(feature = "nrf51"))]
    for &p in ports {
        // Enable latched detection
        p.detectmode.write(|w| w.detectmode().ldetect());
        // Clear latch
        p.latch.write(|w| unsafe { w.bits(0xFFFFFFFF) })
    }

    // Enable interrupts
    cfg_if::cfg_if! {
        if #[cfg(any(feature="nrf5340-app-s", feature="nrf9160-s"))] {
            let irq = unsafe { interrupt::GPIOTE0::steal() };
        } else if #[cfg(any(feature="nrf5340-app-ns", feature="nrf9160-ns"))] {
            let irq = unsafe { interrupt::GPIOTE1::steal() };
        } else {
            let irq = unsafe { interrupt::GPIOTE::steal() };
        }
    }

    irq.unpend();
    irq.set_priority(irq_prio);
    irq.enable();

    let g = regs();
    g.events_port.write(|w| w);
    g.intenset.write(|w| w.port().set());
}

cfg_if::cfg_if! {
    if #[cfg(any(feature="nrf5340-app-s", feature="nrf9160-s"))] {
        #[interrupt]
        fn GPIOTE0() {
            unsafe { handle_gpiote_interrupt() };
        }
    } else if #[cfg(any(feature="nrf5340-app-ns", feature="nrf9160-ns"))] {
        #[interrupt]
        fn GPIOTE1() {
            unsafe { handle_gpiote_interrupt() };
        }
    } else {
        #[interrupt]
        fn GPIOTE() {
            unsafe { handle_gpiote_interrupt() };
        }
    }
}

unsafe fn handle_gpiote_interrupt() {
    let g = regs();

    for i in 0..CHANNEL_COUNT {
        if g.events_in[i].read().bits() != 0 {
            g.intenclr.write(|w| unsafe { w.bits(1 << i) });
            CHANNEL_WAKERS[i].wake();
        }
    }

    #[cfg(not(feature = "nrf51"))]
    if g.events_port.read().bits() != 0 {
        g.events_port.write(|w| w);

        #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
        let ports = &[&*pac::P0::ptr(), &*pac::P1::ptr()];
        #[cfg(not(any(feature = "nrf52833", feature = "nrf52840")))]
        let ports = &[&*pac::P0::ptr()];

        for (port, &p) in ports.iter().enumerate() {
            let bits = p.latch.read().bits();
            for pin in BitIter(bits) {
                p.pin_cnf[pin as usize].modify(|_, w| w.sense().disabled());
                PORT_WAKERS[port * 32 + pin as usize].wake();
            }
            p.latch.write(|w| w.bits(bits));
        }
    }
}

struct BitIter(u32);

impl Iterator for BitIter {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            32 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b)
            }
        }
    }
}

/// GPIOTE channel driver in input mode
pub struct InputChannel<'d, C: Channel, T: GpioPin> {
    ch: C,
    pin: Input<'d, T>,
}

impl<'d, C: Channel, T: GpioPin> Drop for InputChannel<'d, C, T> {
    fn drop(&mut self) {
        let g = regs();
        let num = self.ch.number();
        g.config[num].write(|w| w.mode().disabled());
        g.intenclr.write(|w| unsafe { w.bits(1 << num) });
    }
}

impl<'d, C: Channel, T: GpioPin> InputChannel<'d, C, T> {
    pub fn new(ch: C, pin: Input<'d, T>, polarity: InputChannelPolarity) -> Self {
        let g = regs();
        let num = ch.number();

        g.config[num].write(|w| {
            match polarity {
                InputChannelPolarity::HiToLo => w.mode().event().polarity().hi_to_lo(),
                InputChannelPolarity::LoToHi => w.mode().event().polarity().lo_to_hi(),
                InputChannelPolarity::None => w.mode().event().polarity().none(),
                InputChannelPolarity::Toggle => w.mode().event().polarity().toggle(),
            };
            #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
            w.port().bit(match pin.pin.port() {
                crate::gpio::Port::Port0 => false,
                crate::gpio::Port::Port1 => true,
            });
            unsafe { w.psel().bits(pin.pin.pin()) }
        });

        g.events_in[num].reset();

        InputChannel { ch, pin }
    }

    pub async fn wait(&self) {
        let g = regs();
        let num = self.ch.number();

        // Enable interrupt
        g.events_in[num].reset();
        g.intenset.write(|w| unsafe { w.bits(1 << num) });

        poll_fn(|cx| {
            CHANNEL_WAKERS[num].register(cx.waker());

            if g.events_in[num].read().bits() != 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    /// Returns the IN event, for use with PPI.
    pub fn event_in(&self) -> Event {
        let g = regs();
        Event::from_reg(&g.events_in[self.ch.number()])
    }
}

impl<'d, C: Channel, T: GpioPin> InputPin for InputChannel<'d, C, T> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
}

/// GPIOTE channel driver in output mode
pub struct OutputChannel<'d, C: Channel, T: GpioPin> {
    ch: C,
    _pin: Output<'d, T>,
}

impl<'d, C: Channel, T: GpioPin> Drop for OutputChannel<'d, C, T> {
    fn drop(&mut self) {
        let g = regs();
        let num = self.ch.number();
        g.config[num].write(|w| w.mode().disabled());
        g.intenclr.write(|w| unsafe { w.bits(1 << num) });
    }
}

impl<'d, C: Channel, T: GpioPin> OutputChannel<'d, C, T> {
    pub fn new(ch: C, pin: Output<'d, T>, polarity: OutputChannelPolarity) -> Self {
        let g = regs();
        let num = ch.number();

        g.config[num].write(|w| {
            w.mode().task();
            match pin.is_set_high().unwrap() {
                true => w.outinit().high(),
                false => w.outinit().low(),
            };
            match polarity {
                OutputChannelPolarity::Set => w.polarity().lo_to_hi(),
                OutputChannelPolarity::Clear => w.polarity().hi_to_lo(),
                OutputChannelPolarity::Toggle => w.polarity().toggle(),
            };
            #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
            w.port().bit(match pin.pin.port() {
                crate::gpio::Port::Port0 => false,
                crate::gpio::Port::Port1 => true,
            });
            unsafe { w.psel().bits(pin.pin.pin()) }
        });

        OutputChannel { ch, _pin: pin }
    }

    /// Triggers `task out` (as configured with task_out_polarity, defaults to Toggle).
    pub fn out(&self) {
        let g = regs();
        g.tasks_out[self.ch.number()].write(|w| unsafe { w.bits(1) });
    }

    /// Triggers `task set` (set associated pin high).
    #[cfg(not(feature = "nrf51"))]
    pub fn set(&self) {
        let g = regs();
        g.tasks_set[self.ch.number()].write(|w| unsafe { w.bits(1) });
    }

    /// Triggers `task clear` (set associated pin low).
    #[cfg(not(feature = "nrf51"))]
    pub fn clear(&self) {
        let g = regs();
        g.tasks_clr[self.ch.number()].write(|w| unsafe { w.bits(1) });
    }

    /// Returns the OUT task, for use with PPI.
    pub fn task_out(&self) -> Task {
        let g = regs();
        Task::from_reg(&g.tasks_out[self.ch.number()])
    }

    /// Returns the CLR task, for use with PPI.
    #[cfg(not(feature = "nrf51"))]
    pub fn task_clr(&self) -> Task {
        let g = regs();
        Task::from_reg(&g.tasks_clr[self.ch.number()])
    }

    /// Returns the SET task, for use with PPI.
    #[cfg(not(feature = "nrf51"))]
    pub fn task_set(&self) -> Task {
        let g = regs();
        Task::from_reg(&g.tasks_set[self.ch.number()])
    }
}

// The nrf51 has no latch register, so we can't reliably tell which pins might have caused the PORT event to fire.
// Some of the pins may go back to low before they are checked.
//
// TODO: expose the PORT event in some other way on the nrf51
#[cfg(not(feature = "nrf51"))]
mod port_input {
    use core::convert::Infallible;
    use core::future::Future;
    use core::marker::PhantomData;
    use core::task::{Context, Poll};

    use embassy::traits::gpio::{WaitForAnyEdge, WaitForHigh, WaitForLow};
    use embedded_hal::digital::v2::InputPin;

    use crate::gpio::sealed::Pin;
    use crate::gpio::{AnyPin, Input, Pin as GpioPin};

    use super::PORT_WAKERS;

    /// GPIOTE port input driver
    pub struct PortInput<'d, T: GpioPin> {
        pin: Input<'d, T>,
    }

    impl<'d, T: GpioPin> Unpin for PortInput<'d, T> {}

    impl<'d, T: GpioPin> PortInput<'d, T> {
        pub fn new(pin: Input<'d, T>) -> Self {
            Self { pin }
        }
    }

    impl<'d, T: GpioPin> InputPin for PortInput<'d, T> {
        type Error = Infallible;

        fn is_high(&self) -> Result<bool, Self::Error> {
            self.pin.is_high()
        }

        fn is_low(&self) -> Result<bool, Self::Error> {
            self.pin.is_low()
        }
    }

    impl<'d, T: GpioPin> WaitForHigh for PortInput<'d, T> {
        type Future<'a>
        where
            Self: 'a,
        = PortInputFuture<'a>;

        fn wait_for_high<'a>(&'a mut self) -> Self::Future<'a> {
            self.pin.pin.conf().modify(|_, w| w.sense().high());

            PortInputFuture {
                pin_port: self.pin.pin.pin_port(),
                phantom: PhantomData,
            }
        }
    }

    impl<'d, T: GpioPin> WaitForLow for PortInput<'d, T> {
        type Future<'a>
        where
            Self: 'a,
        = PortInputFuture<'a>;

        fn wait_for_low<'a>(&'a mut self) -> Self::Future<'a> {
            self.pin.pin.conf().modify(|_, w| w.sense().low());

            PortInputFuture {
                pin_port: self.pin.pin.pin_port(),
                phantom: PhantomData,
            }
        }
    }

    impl<'d, T: GpioPin> WaitForAnyEdge for PortInput<'d, T> {
        type Future<'a>
        where
            Self: 'a,
        = PortInputFuture<'a>;

        fn wait_for_any_edge<'a>(&'a mut self) -> Self::Future<'a> {
            if self.is_high().ok().unwrap() {
                self.pin.pin.conf().modify(|_, w| w.sense().low());
            } else {
                self.pin.pin.conf().modify(|_, w| w.sense().high());
            }
            PortInputFuture {
                pin_port: self.pin.pin.pin_port(),
                phantom: PhantomData,
            }
        }
    }

    pub struct PortInputFuture<'a> {
        pin_port: u8,
        phantom: PhantomData<&'a mut AnyPin>,
    }

    impl<'a> Drop for PortInputFuture<'a> {
        fn drop(&mut self) {
            let pin = unsafe { AnyPin::steal(self.pin_port) };
            pin.conf().modify(|_, w| w.sense().disabled());
        }
    }

    impl<'a> Future for PortInputFuture<'a> {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            PORT_WAKERS[self.pin_port as usize].register(cx.waker());

            let pin = unsafe { AnyPin::steal(self.pin_port) };
            if pin.conf().read().sense().is_disabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}
#[cfg(not(feature = "nrf51"))]
pub use port_input::*;

mod sealed {
    pub trait Channel {}
}

pub trait Channel: sealed::Channel + Sized {
    fn number(&self) -> usize;
    fn degrade(self) -> AnyChannel {
        AnyChannel {
            number: self.number() as u8,
        }
    }
}

pub struct AnyChannel {
    number: u8,
}
unsafe_impl_unborrow!(AnyChannel);
impl sealed::Channel for AnyChannel {}
impl Channel for AnyChannel {
    fn number(&self) -> usize {
        self.number as usize
    }
}

macro_rules! impl_channel {
    ($type:ident, $number:expr) => {
        impl sealed::Channel for peripherals::$type {}
        impl Channel for peripherals::$type {
            fn number(&self) -> usize {
                $number as usize
            }
        }
    };
}

impl_channel!(GPIOTE_CH0, 0);
impl_channel!(GPIOTE_CH1, 1);
impl_channel!(GPIOTE_CH2, 2);
impl_channel!(GPIOTE_CH3, 3);
#[cfg(not(feature = "nrf51"))]
impl_channel!(GPIOTE_CH4, 4);
#[cfg(not(feature = "nrf51"))]
impl_channel!(GPIOTE_CH5, 5);
#[cfg(not(feature = "nrf51"))]
impl_channel!(GPIOTE_CH6, 6);
#[cfg(not(feature = "nrf51"))]
impl_channel!(GPIOTE_CH7, 7);
