// Some of the nrf51's peripherals do use DMA, but none of them are supported yet.
#[cfg(not(feature = "nrf51"))]
pub mod dma {
    const SRAM_LOWER: usize = 0x2000_0000;
    const SRAM_UPPER: usize = 0x3000_0000;

    /// Does this slice reside entirely within RAM?
    pub(crate) fn slice_in_ram(slice: &[u8]) -> bool {
        let ptr = slice.as_ptr() as usize;
        ptr >= SRAM_LOWER && (ptr + slice.len()) < SRAM_UPPER
    }

    /// Return an error if slice is not in RAM.
    pub(crate) fn slice_in_ram_or<T>(slice: &[u8], err: T) -> Result<(), T> {
        if slice.len() == 0 || slice_in_ram(slice) {
            Ok(())
        } else {
            Err(err)
        }
    }
}

// `IrqWrite` isn't used outside of the nrf51.
#[allow(dead_code)]
pub mod io {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};
    use core::{mem, slice};

    use embassy::interrupt::Interrupt;
    use embassy::util::WakerRegistration;
    use embassy::util::{OnDrop, Unborrow};
    use embassy_hal_common::peripheral::{PeripheralMutex, PeripheralState, StateStorage};
    use futures::future::poll_fn;

    pub struct ReadState<'a, T> {
        reader: &'a T,
        buf_iter: slice::IterMut<'a, u8>,
        waker: WakerRegistration,
    }

    pub struct WriteState<'a, T> {
        reader: &'a T,
        // This gets set to `None` after the last `txdrdy` is recieved, since the iterator having finished isn't enough to know we're done.
        buf_iter: Option<slice::Iter<'a, u8>>,
        waker: WakerRegistration,
    }

    impl<'a, T> PeripheralState for ReadState<'a, T>
    where
        T: ByteRead,
    {
        type Interrupt = T::Interrupt;
        fn on_interrupt(&mut self) {
            if let Some(byte) = self.buf_iter.next() {
                // We only want to clear the event if we're not done,
                // so that if we're already done the event will be handled immediately next time the peripheral is used.
                //
                // For example, in UART we want to first read out bytes which were already buffered by the peripheral,
                // so we leave the `rxdrdy` event set to immediately trigger the interrupt next time it's enabled.
                self.reader.clear_event();

                *byte = self.reader.next_byte();

                // TODO: Use `is_empty` instead when it gets stabilised, since it's a bit more efficient.
                // Hopefully this'll compile down to the same thing.
                if self.buf_iter.len() == 0 {
                    // We've filled the buffer, so it's time to wake the task.
                    self.waker.wake();
                }
            } else {
                // If we're not clearing the event, we have to disable the interrupt,
                // otherwise the interrupt gets re-fired as soon as it exits.
                self.reader.disable_irq();
            }
        }
    }

    impl<'a, T> PeripheralState for WriteState<'a, T>
    where
        T: ByteWrite,
    {
        type Interrupt = T::Interrupt;
        fn on_interrupt(&mut self) {
            if let Some(iter) = &mut self.buf_iter {
                self.reader.clear_event();

                if let Some(&byte) = iter.next() {
                    self.reader.set_next_byte(byte);
                } else {
                    // This is the `txdrdy` fired after the last byte was sent,
                    // so it's time to wake the waker.

                    // First, set the iterator to `None` so the future knows we're done.
                    self.buf_iter = None;

                    self.waker.wake();
                }
            } else {
                // The only time when this interrupt should be fired with the iterator set to `None` is when the future is being dropped.
                // We can't clear the event, since it relies on that to know when it's done,
                // so instead disable the interrupt to stop it being repeatedly fired.
                self.reader.disable_irq();
            }
        }
    }

    /// A helper to implement drivers which read from a register one byte at a time in an interrupt.
    pub trait ByteRead: Sync {
        /// This is only needed because `PeripheralState` requires it. TODO: Should `PeripheralState` require it in the first place?
        type Interrupt: Interrupt;

        fn irq(&mut self) -> &mut Self::Interrupt;

        /// Enable an interrupt to fire when a byte is ready to be read.
        fn enable_irq(&self);

        /// Disable an interrupt to fire when a byte is ready to be read.
        fn disable_irq(&self);

        /// Start the peripheral generating bytes.
        fn start(&self);

        /// Stop the peripheral generating bytes.
        fn stop(&self);

        /// Clear the event which is triggering the interrupt.
        fn clear_event(&self);

        /// Read the next byte from the peripheral.
        fn next_byte(&self) -> u8;
    }

    /// The `read` function for `IrqRead` which you can more or less reexport.
    ///
    /// It can't be implemented as a provided method because `impl Future` can't be used as the return type.
    ///
    /// # Safety
    /// The future's lifetime must not end without triggering its destructor,
    /// e.g. using `mem::forget`.
    pub async unsafe fn read<T: ByteRead>(reader: &mut T, dest: &mut [u8]) {
        // SAFETY: This future will only live as long as the reference to the original `irq`;
        // we only need ownership to pass it to `PeripheralMutex`.
        let irq = unsafe { reader.irq().unborrow() };

        if dest.len() == 0 {
            return; // Nothing to fill
        }

        let mut storage = StateStorage::new();

        let mut mutex = PeripheralMutex::new_unchecked(irq, &mut storage, || ReadState {
            reader: &*reader,
            buf_iter: dest.iter_mut(),
            waker: WakerRegistration::new(),
        });

        reader.enable_irq();
        reader.start();

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

    pub trait ByteWrite: Sync {
        type Interrupt: Interrupt + Unpin;

        fn irq(&mut self) -> &mut Self::Interrupt;

        fn enable_irq(&self);
        fn disable_irq(&self);

        fn start(&self);
        fn stop(&self);

        fn event_fired(&self) -> bool;
        fn clear_event(&self);

        fn set_next_byte(&self, byte: u8);
    }

    /// # Safety
    /// The future's lifetime must not end without triggering its destructor,
    /// e.g. using `mem::forget`.
    pub async unsafe fn write<T: ByteWrite>(reader: &mut T, buf: &[u8]) {
        // SAFETY: This future will only live as long as the reference to the original `irq`;
        // we only need ownership to pass it to `PeripheralMutex`.
        let irq = unsafe { reader.irq().unborrow() };

        if buf.len() == 0 {
            return; // Nothing to send
        }

        reader.start();

        reader.set_next_byte(buf[0]);

        let mut storage = StateStorage::new();

        let mutex = PeripheralMutex::new_unchecked(irq, &mut storage, || WriteState {
            reader: &*reader,
            buf_iter: Some(buf[1..].iter()),
            waker: WakerRegistration::new(),
        });

        reader.enable_irq();

        // This is needed because if we keep mutable references to `mutex` in both `OnDrop` and `poll_fn`,
        // it's aliasing mutable references, so we have to use an actual intermediate `Future`.
        struct InnerFuture<'a, T: ByteWrite> {
            reader: &'a T,
            mutex: PeripheralMutex<'a, WriteState<'a, T>>,
        }

        impl<'a, T: ByteWrite> Future for InnerFuture<'a, T> {
            type Output = ();

            fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
                trace!("irq_write: poll");

                self.mutex.with(|state| {
                    state.waker.register(cx.waker());

                    // The interrupt sets `buf_iter` to `None` when it's done.
                    if state.buf_iter.is_none() {
                        trace!("  ready");
                        Poll::Ready(())
                    } else {
                        Poll::Pending
                    }
                })
            }
        }

        impl<'a, T: ByteWrite> Drop for InnerFuture<'a, T> {
            fn drop(&mut self) {
                trace!("irq_write: dropping");

                // Don't stop yet, because at least for UART triggering `tasks_stoptx` will cause the transmission to be 'stopped immediately' according to the reference manual.
                // So wait for the last byte to finish.

                // Set `buf_iter` to `None`, to tell the interrupt it's done and stop it from sending another byte.
                self.mutex.with(|state| {
                    state.buf_iter = None;
                });

                while !self.reader.event_fired() {
                    trace!("  wfe");
                    cortex_m::asm::wfe();
                }
                cortex_m::asm::sev();

                // Now we can safely stop the transmission.
                self.reader.stop();
                self.reader.disable_irq();
            }
        }

        let mut fut = InnerFuture { reader, mutex };
        Pin::new(&mut fut).await;

        // Since the future completed properly, we don't need to wait for the last byte.
        mem::forget(fut);

        reader.stop();
        reader.disable_irq();
    }
}
