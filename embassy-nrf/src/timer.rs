#![macro_use]

use core::convert::TryFrom;
use core::convert::TryInto;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::task::Poll;

use embassy::interrupt::Interrupt;
use embassy::interrupt::InterruptExt;
use embassy::util::OnDrop;
use embassy::util::Unborrow;
use embassy_extras::unborrow;
use futures::future::poll_fn;
use pac::timer0::bitmode;
use pac::timer0::bitmode::BITMODE_W;

use crate::pac;
use crate::ppi::Event;
use crate::ppi::Task;

pub(crate) mod sealed {
    use embassy::util::AtomicWaker;

    use super::*;

    pub trait Instance {
        /// The number of CC registers this instance has.
        const CCS: usize;
        fn regs() -> &'static pac::timer0::RegisterBlock;
        /// Storage for the waker for CC register `n`.
        fn waker(n: usize) -> &'static AtomicWaker;
    }
    pub trait ExtendedInstance {}

    pub trait SupportsBitmode<T>
    where
        T: Bitmode,
        // I'm not sure why this isn't just implicit
        <T as TryFrom<u32>>::Error: Debug,
    {
    }
}

pub trait SupportsBitmode<T>: sealed::SupportsBitmode<T>
where
    T: Bitmode,
    <T as TryFrom<u32>>::Error: Debug,
{
}

pub trait Instance: Unborrow<Target = Self> + sealed::Instance + 'static {
    type Interrupt: Interrupt;
}
pub trait ExtendedInstance: Instance + sealed::ExtendedInstance {}

macro_rules! impl_timer {
    ($type:ident, $pac_type:ident, $irq:ident, [$($bitmode:path),*], $ccs:literal) => {
        impl crate::timer::sealed::Instance for peripherals::$type {
            const CCS: usize = $ccs;
            fn regs() -> &'static pac::timer0::RegisterBlock {
                unsafe { &*(pac::$pac_type::ptr() as *const pac::timer0::RegisterBlock) }
            }
            fn waker(n: usize) -> &'static ::embassy::util::AtomicWaker {
                use ::embassy::util::AtomicWaker;
                const NEW_AW: AtomicWaker = AtomicWaker::new();
                static WAKERS: [AtomicWaker; $ccs] = [NEW_AW; $ccs];
                &WAKERS[n]
            }
        }
        impl crate::timer::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::$irq;
        }
        $(
            impl crate::timer::sealed::SupportsBitmode<$bitmode> for peripherals::$type {}
        )*
    };
    ($type:ident, $pac_type:ident, $irq:ident, [$($bitmode:path),*]) => {
        impl_timer!($type, $pac_type, $irq, [$($bitmode),*], 4);
    };
    ($type:ident, $pac_type:ident, $irq:ident, [$($bitmode:path),*], extended) => {
        impl_timer!($type, $pac_type, $irq, [$($bitmode),*], 6);
        impl crate::timer::sealed::ExtendedInstance for peripherals::$type {}
        impl crate::timer::ExtendedInstance for peripherals::$type {}
    };
}

#[repr(u8)]
pub enum Frequency {
    // I'd prefer not to prefix these with `F`, but Rust identifiers can't start with digits.
    F16MHz = 0,
    F8MHz = 1,
    F4MHz = 2,
    F2MHz = 3,
    F1MHz = 4,
    F500kHz = 5,
    F250kHz = 6,
    F125kHz = 7,
    F62500Hz = 8,
    F31250Hz = 9,
}

pub trait Bitmode: TryFrom<u32> + Into<u32>
where
    <Self as TryFrom<u32>>::Error: Debug,
{
    fn config(w: BITMODE_W) -> &mut bitmode::W;
}

impl Bitmode for u8 {
    fn config(w: BITMODE_W) -> &mut bitmode::W {
        w._08bit()
    }
}

impl Bitmode for u16 {
    fn config(w: BITMODE_W) -> &mut bitmode::W {
        w._16bit()
    }
}

// TODO: support 24-bit bitmodes. Maybe use a `U24` wrapper type or something?

impl Bitmode for u32 {
    fn config(w: BITMODE_W) -> &mut bitmode::W {
        w._32bit()
    }
}

/// nRF Timer driver.
///
/// The timer has an internal counter, which is incremented for every tick of the timer.
/// The counter will wrap around when it overflows.
///
/// The size of the counter can vary. On most timers, it goes up to 32 bits, except for TIMER1 and TIMER2 on nrf51 chips.
/// It can be specified through `Timer`'s second generic parameter, as `u8` for 8 bits, `u16` for 16 bits, etc.
/// It defaults to 32 bits.
///
/// It has either 4 or 6 Capture/Compare registers, which can be used to capture the current state of the counter
/// or trigger an event when the counter reaches a certain value.
pub struct Timer<'d, T, B = u32>
// TODO: should `B` default to `u16` instead for timers which don't support 32-bit bitmode?
where
    T: Instance,
    B: Bitmode,
    T: SupportsBitmode<B>,
    <B as TryFrom<u32>>::Error: Debug,
{
    phantom: PhantomData<(&'d mut T, B)>,
}

impl<'d, T, B> Timer<'d, T, B>
where
    T: Instance,
    B: Bitmode,
    T: SupportsBitmode<B>,
    <B as TryFrom<u32>>::Error: Debug,
{
    pub fn new(
        timer: impl Unborrow<Target = T> + 'd,
        irq: impl Unborrow<Target = T::Interrupt> + 'd,
    ) -> Self {
        unborrow!(irq);

        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        Self::new_irqless(timer)
    }

    /// Create a `Timer` without an interrupt, meaning `Cc::wait` won't work.
    ///
    /// This is used by `Uarte` internally.
    pub(crate) fn new_irqless(_timer: impl Unborrow<Target = T> + 'd) -> Self {
        let regs = T::regs();

        let mut this = Self {
            phantom: PhantomData,
        };

        // Stop the timer before doing anything else,
        // since changing BITMODE while running can cause 'unpredictable behaviour' according to the specification.
        this.stop();

        // Set the instance to timer mode.
        regs.mode.write(|w| w.mode().timer());

        // Configure bitmode
        regs.bitmode.write(|w| B::config(w.bitmode()));

        // Initialize the counter at 0.
        this.clear();

        // Default to the max frequency of the lower power clock
        this.set_frequency(Frequency::F1MHz);

        for n in 0..T::CCS {
            let cc = this.cc(n);
            // Initialize all the shorts as disabled.
            cc.unshort_compare_clear();
            cc.unshort_compare_stop();
            // Initialize the CC registers as 0.
            cc.write(0u32.try_into().unwrap()); // 0 is valid in any bitmode
        }

        this
    }

    /// Starts the timer.
    pub fn start(&self) {
        T::regs().tasks_start.write(|w| unsafe { w.bits(1) })
    }

    /// Stops the timer.
    pub fn stop(&self) {
        T::regs().tasks_stop.write(|w| unsafe { w.bits(1) })
    }

    /// Reset the timer's counter to 0.
    pub fn clear(&self) {
        T::regs().tasks_clear.write(|w| unsafe { w.bits(1) })
    }

    /// Returns the START task, for use with PPI.
    ///
    /// When triggered, this task starts the timer.
    pub fn task_start(&self) -> Task {
        Task::from_reg(&T::regs().tasks_start)
    }

    /// Returns the STOP task, for use with PPI.
    ///
    /// When triggered, this task stops the timer.
    pub fn task_stop(&self) -> Task {
        Task::from_reg(&T::regs().tasks_stop)
    }

    /// Returns the CLEAR task, for use with PPI.
    ///
    /// When triggered, this task resets the timer's counter to 0.
    pub fn task_clear(&self) -> Task {
        Task::from_reg(&T::regs().tasks_clear)
    }

    /// Change the timer's frequency.
    ///
    /// This will stop the timer if it isn't already stopped,
    /// because the timer may exhibit 'unpredictable behaviour' if it's frequency is changed while it's running.
    pub fn set_frequency(&self, frequency: Frequency) {
        self.stop();

        T::regs()
            .prescaler
            // SAFETY: `frequency` is a variant of `Frequency`,
            // whose values are all in the range of 0-9 (the valid range of `prescaler`).
            .write(|w| unsafe { w.prescaler().bits(frequency as u8) })
    }

    fn on_interrupt(_: *mut ()) {
        let regs = T::regs();
        for n in 0..T::CCS {
            if regs.events_compare[n].read().bits() != 0 {
                // Clear the interrupt, otherwise the interrupt will be repeatedly raised as soon as the interrupt handler exits.
                // We can't clear the event, because it's used to poll whether the future is done or still pending.
                regs.intenclr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << (16 + n))) });
                T::waker(n).wake();
            }
        }
    }

    /// Returns this timer's `n`th CC register.
    ///
    /// # Panics
    /// Panics if `n` >= the number of CC registers this timer has (4 for a normal timer, 6 for an extended timer).
    pub fn cc(&mut self, n: usize) -> Cc<T, B> {
        if n >= T::CCS {
            panic!(
                "Cannot get CC register {} of timer with {} CC registers.",
                n,
                T::CCS
            );
        }
        Cc {
            n,
            phantom: PhantomData,
        }
    }
}

/// A representation of a timer's Capture/Compare (CC) register.
///
/// A CC register holds a 32-bit value.
/// This is used either to store a capture of the timer's current count, or to specify the value for the timer to compare against.
///
/// The timer will fire the register's COMPARE event when its counter reaches the value stored in the register.
/// When the register's CAPTURE task is triggered, the timer will store the current value of its counter in the register
pub struct Cc<'a, T, B>
where
    T: Instance,
    B: Bitmode,
    T: SupportsBitmode<B>,
    <B as TryFrom<u32>>::Error: Debug,
{
    n: usize,
    phantom: PhantomData<(&'a mut T, B)>,
}

impl<'a, T, B> Cc<'a, T, B>
where
    T: Instance,
    B: Bitmode,
    T: SupportsBitmode<B>,
    <B as TryFrom<u32>>::Error: Debug,
{
    /// Get the current value stored in the register.
    pub fn read(&self) -> B {
        // We initialize all the CC registers to 0, and then only ever write values of the correct bitmode to it.
        // So, there should be no way an invalid value could be in there.
        T::regs().cc[self.n].read().bits().try_into().unwrap()
    }

    /// Set the value stored in the register.
    ///
    /// `event_compare` will fire when the timer's counter reaches this value.
    pub fn write(&self, value: B) {
        // SAFETY: there are no invalid values for the CC register.
        T::regs().cc[self.n].write(|w| unsafe { w.bits(value.into()) })
    }

    /// Capture the current value of the timer's counter in this register, and return it.
    pub fn capture(&self) -> B {
        T::regs().tasks_capture[self.n].write(|w| unsafe { w.bits(1) });
        self.read()
    }

    /// Returns this CC register's CAPTURE task, for use with PPI.
    ///
    /// When triggered, this task will capture the current value of the timer's counter in this register.
    pub fn task_capture(&self) -> Task {
        Task::from_reg(&T::regs().tasks_capture[self.n])
    }

    /// Returns this CC register's COMPARE event, for use with PPI.
    ///
    /// This event will fire when the timer's counter reaches the value in this CC register.
    pub fn event_compare(&self) -> Event {
        Event::from_reg(&T::regs().events_compare[self.n])
    }

    /// Enable the shortcut between this CC register's COMPARE event and the timer's CLEAR task.
    ///
    /// This means that when the COMPARE event is fired, the CLEAR task will be triggered.
    ///
    /// So, when the timer's counter reaches the value stored in this register, the timer's counter will be reset to 0.
    pub fn short_compare_clear(&self) {
        T::regs()
            .shorts
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.n)) })
    }

    /// Disable the shortcut between this CC register's COMPARE event and the timer's CLEAR task.
    pub fn unshort_compare_clear(&self) {
        T::regs()
            .shorts
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.n)) })
    }

    /// Enable the shortcut between this CC register's COMPARE event and the timer's STOP task.
    ///
    /// This means that when the COMPARE event is fired, the STOP task will be triggered.
    ///
    /// So, when the timer's counter reaches the value stored in this register, the timer will stop counting up.
    pub fn short_compare_stop(&self) {
        T::regs()
            .shorts
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << (8 + self.n))) })
    }

    /// Disable the shortcut between this CC register's COMPARE event and the timer's STOP task.
    pub fn unshort_compare_stop(&self) {
        T::regs()
            .shorts
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << (8 + self.n))) })
    }

    /// Wait until the timer's counter reaches the value stored in this register.
    ///
    /// This requires a mutable reference so that this task's waker cannot be overwritten by a second call to `wait`.
    pub async fn wait(&mut self) {
        let regs = T::regs();

        // Enable the interrupt for this CC's COMPARE event.
        regs.intenset
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << (16 + self.n))) });

        // Disable the interrupt if the future is dropped.
        let on_drop = OnDrop::new(|| {
            regs.intenclr
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << (16 + self.n))) });
        });

        poll_fn(|cx| {
            T::waker(self.n).register(cx.waker());

            if regs.events_compare[self.n].read().bits() != 0 {
                // Reset the register for next time
                regs.events_compare[self.n].reset();
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        // The interrupt was already disabled in the interrupt handler, so there's no need to disable it again.
        on_drop.defuse();
    }
}
