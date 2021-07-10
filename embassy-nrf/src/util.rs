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

pub struct IrqIoState {
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
pub trait IrqRead {
    /// Some state which should be stored in a `static` somewhere.
    fn state() -> &'static IrqIoState;

    /// Enable an interrupt to fire when a byte is ready to be read.
    fn enable_irq(&self);

    /// Disable an interrupt to fire when a byte is ready to be read.
    fn disable_irq();

    /// Start the peripheral generating bytes.
    fn start(&self);

    /// Stop the peripheral generating bytes.
    fn stop(&self);

    /// Clear the event which is triggering the interrupt.
    fn clear_event();

    /// Read the next byte from the peripheral.
    fn next_byte() -> u8;

    /// This should be called in an interrupt handler, if the corresponding event has been fired and after clearing said event.
    fn on_irq() {
        trace!("IrqRead::on_irq");

        // Mutate the slice within a critical section,
        // so that the future isn't dropped in between us loading the pointer and actually dereferencing it.
        critical_section::with(|_| {
            let ptr = Self::state().ptr.load(Ordering::Relaxed);
            // We need to make sure we haven't already filled the whole slice,
            // in case the interrupt fired again before the executor got back to the future.
            let end = Self::state().end.load(Ordering::Relaxed);
            trace!("ptr: {}, end: {}", ptr, end);
            if !ptr.is_null() && ptr != end {
                // We only want to clear the event if we're not done,
                // so that if we're already done the event will be handled immediately next time the peripheral is used.
                //
                // For example, in UART we want to first read out bytes which were already buffered by the peripheral,
                // so we leave the `rxdrdy` event set to immediately trigger the interrupt next time it's enabled.
                Self::clear_event();

                // If the future was dropped, the pointer would have been set to null,
                // so we're still good to mutate the slice.
                // The safety contract of `irq_read` means that the future can't have been dropped
                // without calling its destructor.
                unsafe {
                    *ptr = Self::next_byte();
                }

                let new_ptr = unsafe { ptr.add(1) };
                if new_ptr == end {
                    Self::state().waker.wake();
                }
                Self::state().ptr.store(new_ptr, Ordering::Relaxed);
            } else {
                // If we're not clearing the event, we have to disable the interrupt,
                // otherwise the interrupt gets re-fired as soon as it exits.
                Self::disable_irq();
            }
        });
    }
}

/// The `read` function for `IrqRead` which you can more or less reexport.
///
/// It can't be implemented as a provided method because `impl Future` can't be used as the return type.
///
/// # Safety
/// The future's lifetime must not end without triggering its destructor,
/// e.g. using `mem::forget`.
pub async unsafe fn irq_read<T: IrqRead>(this: &mut T, dest: &mut [u8]) {
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
        // Disable the interrupt first, so we don't waste its time with events it'll just ignore.
        T::disable_irq();
        this.stop();

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

pub trait IrqWrite {
    fn state() -> &'static IrqIoState;

    fn enable_irq(&self);
    fn disable_irq();

    fn start(&self);
    fn stop(&self);

    fn event_fired(&self) -> bool;
    fn clear_event();

    fn set_next_byte(byte: u8);

    fn on_irq() {
        trace!("IrqWrite::on_irq");

        // Actually dereference `ptr` in a critical section,
        // so that the future isn't dropped in between us loading the pointer and actually dereferencing it.
        critical_section::with(|_| {
            let old_ptr = Self::state().ptr.load(Ordering::Relaxed) as *const u8;
            // We need to check if this is the `txdrdy` fired after the last byte has been sent,
            // so that we don't attempt to send another byte and dereference an invalid pointer.
            let end = Self::state().end.load(Ordering::Relaxed) as *const u8;
            trace!("  old_ptr: {}, end: {}", old_ptr, end);
            if old_ptr.is_null() {
                // If we aren't going to clear the event, we need to disable the interrupt to prevent it being immediately re-fired.
                Self::disable_irq();

                // If the future was dropped, there's nothing we need to do.
                return;
            }
            // Only clear the event if the future hasn't been dropped,
            // because it's used to detect when the last byte has been sent before finishing dropping.
            Self::clear_event();

            // `ptr` is 1 less than the thing we actually want to dereference,
            // so that `ptr == end` only becomes true after the final byte is actually sent (and fires `txdrdy` again).
            let ptr = unsafe { old_ptr.add(1) };
            // If `ptr == end`, it's a pointer to just after the end of the slice, so we can't dereference it.
            if ptr != end {
                trace!("  sending byte {}", unsafe { *ptr });
                // If the future was dropped, the pointer would have been set to null,
                // so we're still good to read from the slice.
                // The safety contract of `irq_write` means that the future can't have been dropped
                // without calling its destructor.
                Self::set_next_byte(unsafe { *ptr })
            } else {
                trace!("  waking");
                // This was the final `txdrdy` after sending the last byte, time to wake the future.
                Self::state().waker.wake();
            }
            Self::state().ptr.store(ptr as *mut u8, Ordering::Relaxed);
        });
    }
}

/// # Safety
/// The future's lifetime must not end without triggering its destructor,
/// e.g. using `mem::forget`.
pub async unsafe fn irq_write<T: IrqWrite>(this: &mut T, buf: &[u8]) {
    if buf.len() == 0 {
        return; // Nothing to send
    }

    this.start();

    T::set_next_byte(buf[0]);

    let range = buf.as_ptr_range();

    // `AtomicPtr` can only store `*mut` pointers, so we have to cast them for storage.
    // We're storing a pointer to the byte _before_ the next byte the interrupt will write.
    T::state()
        .ptr
        .store(range.start as *mut _, Ordering::Relaxed);
    T::state().end.store(range.end as *mut _, Ordering::Relaxed);

    this.enable_irq();

    let on_drop = OnDrop::new(|| {
        trace!("irq_write: dropping");

        // Don't stop yet, because at least for UART triggering `tasks_stoptx` will cause the transmission to be 'stopped immediately' according to the reference manual.
        // So wait for the last byte to finish.

        // Set the pointers to null, so the interrupt doesn't go on to send another byte.
        // Set `ptr` first because it's the one the interrupt checks.
        T::state().ptr.store(ptr::null_mut(), Ordering::Relaxed);
        T::state().end.store(ptr::null_mut(), Ordering::Relaxed);

        while !this.event_fired() {
            trace!("  wfe");
            cortex_m::asm::wfe();
        }
        cortex_m::asm::sev();

        // Now we can safely stop the transmission.
        this.stop();
        T::disable_irq();
    });

    poll_fn(|cx| {
        trace!("irq_write: poll_fn");

        T::state().waker.register(cx.waker());

        // The interrupt will never modify `end`, so load it first and then get the most up-to-date `ptr`.
        let end = T::state().end.load(Ordering::Relaxed);
        let ptr = T::state().ptr.load(Ordering::Relaxed);

        trace!("  ptr: {}, end: {}", ptr, end);

        if ptr == end {
            trace!("  ready");
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;

    on_drop.defuse();

    this.stop();
    T::disable_irq();

    T::state().ptr.store(ptr::null_mut(), Ordering::Relaxed);
    T::state().end.store(ptr::null_mut(), Ordering::Relaxed);
}
