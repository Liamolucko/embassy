use core::ptr;
use core::sync::atomic::AtomicPtr;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy::util::AtomicWaker;
use embassy::util::OnDrop;
use futures::future::poll_fn;

const SRAM_LOWER: usize = 0x2000_0000;
const SRAM_UPPER: usize = 0x3000_0000;

/// Does this slice reside entirely within RAM?
pub(crate) fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= SRAM_LOWER && (ptr + slice.len()) < SRAM_UPPER
}

/// Return an error if slice is not in RAM.
#[cfg(not(feature = "nrf51"))]
pub(crate) fn slice_in_ram_or<T>(slice: &[u8], err: T) -> Result<(), T> {
    if slice.len() == 0 || slice_in_ram(slice) {
        Ok(())
    } else {
        Err(err)
    }
}

pub(crate) struct IrqIoState {
    pub ptr: AtomicPtr<u8>,
    pub end: AtomicPtr<u8>,
    pub waker: AtomicWaker,
}

impl IrqIoState {
    pub const fn new() -> Self {
        Self {
            ptr: AtomicPtr::new(ptr::null_mut()),
            end: AtomicPtr::new(ptr::null_mut()),
            waker: AtomicWaker::new(),
        }
    }
}

/// A helper to implement drivers which read from a register one byte at a time in an interrupt.
pub(crate) trait IrqRead {
    fn state() -> &'static IrqIoState;
    fn enable_irq(&self);
    fn disable_irq(&self);
    fn start(&self);
    fn stop(&self);
    fn next_value() -> u8;

    /// This should be called in an interrupt handler, if the corresponding event has been fired and after clearing said event.
    fn on_irq() {
        // Mutate the slice within a critical section,
        // so that the future isn't dropped in between us loading the pointer and actually dereferencing it.
        let (ptr, end) = critical_section::with(|_| {
            let ptr = Self::state().ptr.load(Ordering::Relaxed);
            // We need to make sure we haven't already filled the whole slice,
            // in case the interrupt fired again before the executor got back to the future.
            let end = Self::state().end.load(Ordering::Relaxed);
            if !ptr.is_null() && ptr != end {
                // If the future was dropped, the pointer would have been set to null,
                // so we're still good to mutate the slice.
                // The safety contract of `irq_read` means that the future can't have been dropped
                // without calling its destructor.
                unsafe {
                    *ptr = Self::next_value();
                }
            }
            (ptr, end)
        });

        if ptr.is_null() || ptr == end {
            // If the future was dropped, there's nothing to do.
            // If `ptr == end`, we were called by mistake, so return.
            return;
        }

        let new_ptr = unsafe { ptr.add(1) };
        match Self::state()
            .ptr
            .compare_exchange(ptr, new_ptr, Ordering::Relaxed, Ordering::Relaxed)
        {
            Ok(_) => {
                let end = Self::state().end.load(Ordering::Relaxed);
                // It doesn't matter if `end` was changed under our feet, because then this will just be false.
                if new_ptr == end {
                    Self::state().waker.wake();
                }
            }
            Err(_) => {
                // If the future was dropped or finished, there's no point trying to wake it.
            }
        }
    }
}

/// The `read` function for `IrqRead` which you can more or less reexport.
///
/// It can't be implemented as a provided method because `impl Future` can't be used as the return type.
///
/// # Safety
/// The future's lifetime must not end without triggering its destructor,
/// e.g. using `mem::forget`.
pub(crate) async unsafe fn irq_read<T: IrqRead>(this: &mut T, dest: &mut [u8]) {
    if dest.len() == 0 {
        return; // Nothing to fill
    }

    let range = dest.as_mut_ptr_range();
    // Even if we've preempted the interrupt, it can't preempt us again,
    // so we don't need to worry about the order we write these in.
    T::state().ptr.store(range.start, Ordering::Relaxed);
    T::state().end.store(range.end, Ordering::Relaxed);

    this.enable_irq();
    this.start();

    let on_drop = OnDrop::new(|| {
        this.stop();
        this.disable_irq();

        // The interrupt is now disabled and can't preempt us anymore, so the order doesn't matter here.
        T::state().ptr.store(ptr::null_mut(), Ordering::Relaxed);
        T::state().end.store(ptr::null_mut(), Ordering::Relaxed);
    });

    poll_fn(|cx| {
        T::state().waker.register(cx.waker());

        // The interrupt will never modify `end`, so load it first and then get the most up-to-date `ptr`.
        let end = T::state().end.load(Ordering::Relaxed);
        let ptr = T::state().ptr.load(Ordering::Relaxed);

        if ptr == end {
            // We're done.
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;

    // Trigger the teardown
    drop(on_drop);
}
