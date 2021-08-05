use core::convert::Infallible;
use core::future::Future;
use core::marker::PhantomData;

use embassy::interrupt::InterruptExt;
use embassy::traits;
use embassy::util::Unborrow;
use embassy_hal_common::unborrow;
use futures::FutureExt;
use rand_core::RngCore;

use crate::interrupt;
use crate::pac;
use crate::peripherals::RNG;
use crate::util::io::{read, ByteRead, State};

impl RNG {
    fn regs() -> &'static pac::rng::RegisterBlock {
        unsafe { &*pac::RNG::ptr() }
    }
}

static STATE: State = State::new();

/// A wrapper around an nRF RNG peripheral.
///
/// It has a non-blocking API, through `embassy::traits::Rng`, and a blocking api through `rand`.
pub struct Rng<'d> {
    irq: interrupt::RNG,
    phantom: PhantomData<(&'d mut RNG, &'d mut interrupt::RNG)>,
}

impl<'d> ByteRead for Rng<'d> {
    #[inline]
    fn state() -> &'static crate::util::io::State {
        &STATE
    }

    #[inline]
    fn enable_irq(&self) {
        RNG::regs().intenset.write(|w| w.valrdy().set());
    }

    #[inline]
    fn disable_irq() {
        RNG::regs().intenclr.write(|w| w.valrdy().clear());
    }

    #[inline]
    fn start(&self) {
        // SAFETY: 1 is a valid variant of `tasks_start`.
        RNG::regs().tasks_start.write(|w| unsafe { w.bits(1) })
    }

    #[inline]
    fn stop(&self) {
        // SAFETY: 1 is a valid variant of `tasks_start`.
        RNG::regs().tasks_stop.write(|w| unsafe { w.bits(1) })
    }

    #[inline]
    fn clear_event() {
        RNG::regs().events_valrdy.reset()
    }

    #[inline]
    fn next_byte() -> u8 {
        RNG::regs().value.read().value().bits()
    }
}

impl<'d> Rng<'d> {
    /// Creates a new RNG driver from the `RNG` peripheral and interrupt.
    ///
    /// # Safety
    /// The future returned from `fill_bytes` must not have its lifetime end without running its destructor,
    /// e.g. using `mem::forget`.
    ///
    /// The synchronous API is safe.
    pub unsafe fn new(
        _rng: impl Unborrow<Target = RNG> + 'd,
        irq: impl Unborrow<Target = interrupt::RNG> + 'd,
    ) -> Self {
        unborrow!(irq);

        let this = Self {
            irq,
            phantom: PhantomData,
        };

        this.stop();
        Self::disable_irq();

        this.irq.set_handler(Self::on_interrupt);
        this.irq.unpend();
        this.irq.enable();

        this
    }

    fn on_interrupt(_: *mut ()) {
        // Defer to `IrqRead`.
        Self::on_irq();
    }

    /// Enable or disable the RNG's bias correction.
    ///
    /// Bias correction removes any bias towards a '1' or a '0' in the bits generated.
    /// However, this makes the generation of numbers slower.
    ///
    /// Defaults to disabled.
    #[inline]
    pub fn bias_correction(&self, enable: bool) {
        RNG::regs().config.write(|w| w.dercen().bit(enable))
    }
}

impl<'d> Drop for Rng<'d> {
    fn drop(&mut self) {
        self.irq.disable()
    }
}

impl<'d> traits::rng::Rng for Rng<'d> {
    type Error = Infallible;

    #[rustfmt::skip] // For some reason rustfmt removes the where clause
    type RngFuture<'a> where 'd: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    #[inline]
    fn fill_bytes<'a>(&'a mut self, dest: &'a mut [u8]) -> Self::RngFuture<'a> {
        // SAFETY: `irq_read`'s safety contract is forwarded to `Rng::new`.
        unsafe { read(self, dest) }.map(|_| Ok(()))
    }
}

impl<'d> RngCore for Rng<'d> {
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.start();

        for byte in dest.iter_mut() {
            let regs = RNG::regs();
            while regs.events_valrdy.read().bits() == 0 {}
            regs.events_valrdy.reset();
            *byte = regs.value.read().value().bits();
        }

        self.stop();
    }

    #[inline]
    fn next_u32(&mut self) -> u32 {
        let mut bytes = [0; 4];
        self.fill_bytes(&mut bytes);
        // We don't care about the endianness, so just use the native one.
        u32::from_ne_bytes(bytes)
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0; 8];
        self.fill_bytes(&mut bytes);
        u64::from_ne_bytes(bytes)
    }

    #[inline]
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

// TODO: Should `Rng` implement `CryptoRng`? It's 'suitable for cryptographic purposes' according to the specification.
