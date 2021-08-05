use core::cmp::min;
use core::marker::PhantomData;
use core::mem;
use core::pin::Pin;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::{Context, Poll};
use embassy::interrupt::InterruptExt;
use embassy::io::{AsyncBufRead, AsyncWrite, Result};
use embassy::util::{Unborrow, WakerRegistration};
use embassy_hal_common::peripheral::{PeripheralMutex, PeripheralState, StateStorage};
use embassy_hal_common::ring_buffer::RingBuffer;
use embassy_hal_common::{low_power_wait_until, unborrow};

use crate::gpio::sealed::Pin as _;
use crate::gpio::{self, OptionalPin as GpioOptionalPin, Pin as GpioPin};
use crate::ppi::{AnyConfigurableChannel, ConfigurableChannel, Event, Ppi, Task};
use crate::timer::Frequency;
use crate::timer::Instance as TimerInstance;
use crate::timer::SupportsBitmode;
use crate::timer::Timer;
use crate::uart::{Config, Instance as UartInstance};

// Re-export SVD variants to allow user to directly set values
pub use crate::uart::{Baudrate, Parity};

#[derive(Copy, Clone, Debug, PartialEq)]
enum RxState {
    Idle,
    Receiving,
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum TxState {
    Idle,
    Transmitting(usize),
}

pub struct State<'d, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>>(
    StateStorage<StateInner<'d, U, T>>,
);
impl<'d, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>> State<'d, U, T> {
    pub fn new() -> Self {
        Self(StateStorage::new())
    }
}

struct StateInner<'d, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>> {
    phantom: PhantomData<&'d mut U>,
    timer: Timer<'d, T, u16>,
    _ppi_ch1: Ppi<'d, AnyConfigurableChannel>,
    _ppi_ch2: Ppi<'d, AnyConfigurableChannel>,

    rx: RingBuffer<'d>,
    rx_state: RxState,
    rx_waker: WakerRegistration,

    tx: RingBuffer<'d>,
    tx_state: TxState,
    tx_waker: WakerRegistration,
}

/// Interface to a UARTE instance
pub struct BufferedUart<'d, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>> {
    inner: PeripheralMutex<'d, StateInner<'d, U, T>>,
}

impl<'d, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>> Unpin
    for BufferedUart<'d, U, T>
{
}

impl<'d, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>> BufferedUart<'d, U, T> {
    /// unsafe: may not leak self or futures
    pub unsafe fn new(
        state: &'d mut State<'d, U, T>,
        _uarte: impl Unborrow<Target = U> + 'd,
        timer: impl Unborrow<Target = T> + 'd,
        ppi_ch1: impl Unborrow<Target = impl ConfigurableChannel> + 'd,
        ppi_ch2: impl Unborrow<Target = impl ConfigurableChannel> + 'd,
        irq: impl Unborrow<Target = U::Interrupt> + 'd,
        rxd: impl Unborrow<Target = impl GpioPin> + 'd,
        txd: impl Unborrow<Target = impl GpioPin> + 'd,
        cts: impl Unborrow<Target = impl GpioOptionalPin> + 'd,
        rts: impl Unborrow<Target = impl GpioOptionalPin> + 'd,
        config: Config,
        rx_buffer: &'d mut [u8],
        tx_buffer: &'d mut [u8],
    ) -> Self {
        unborrow!(ppi_ch1, ppi_ch2, irq, rxd, txd, cts, rts);

        let r = U::regs();

        let mut timer: Timer<T, u16> = Timer::new_irqless(timer);

        rxd.conf().write(|w| w.input().connect().drive().h0h1());
        #[cfg(not(feature = "nrf51"))]
        r.psel.rxd.write(|w| unsafe { w.bits(rxd.psel_bits()) });
        #[cfg(feature = "nrf51")]
        r.pselrxd.write(|w| unsafe { w.bits(rxd.psel_bits()) });

        txd.set_high();
        txd.conf().write(|w| w.dir().output().drive().h0h1());
        #[cfg(not(feature = "nrf51"))]
        r.psel.txd.write(|w| unsafe { w.bits(txd.psel_bits()) });
        #[cfg(feature = "nrf51")]
        r.pseltxd.write(|w| unsafe { w.bits(txd.psel_bits()) });

        if let Some(pin) = rts.pin_mut() {
            pin.set_high();
            pin.conf().write(|w| w.dir().output().drive().h0h1());
        }
        #[cfg(not(feature = "nrf51"))]
        r.psel.cts.write(|w| unsafe { w.bits(cts.psel_bits()) });
        #[cfg(feature = "nrf51")]
        r.pselcts.write(|w| unsafe { w.bits(cts.psel_bits()) });

        if let Some(pin) = cts.pin_mut() {
            pin.conf().write(|w| w.input().connect().drive().h0h1());
        }
        #[cfg(not(feature = "nrf51"))]
        r.psel.rts.write(|w| unsafe { w.bits(rts.psel_bits()) });
        #[cfg(feature = "nrf51")]
        r.pselrts.write(|w| unsafe { w.bits(rts.psel_bits()) });

        r.baudrate.write(|w| w.baudrate().variant(config.baudrate));
        r.config.write(|w| w.parity().variant(config.parity));

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

        // Disable the irq, let the Registration enable it when everything is set up.
        irq.disable();
        irq.pend();

        // Enable UARTE instance
        r.enable.write(|w| w.enable().enabled());

        // Enable interrupts
        #[cfg(not(feature = "nrf51"))]
        r.intenset.write(|w| w.endrx().set().endtx().set());
        #[cfg(feature = "nrf51")]
        r.intenset.write(|w| w.rxdrdy().set().txdrdy().set());

        // BAUDRATE register values are `baudrate * 2^32 / 16000000`
        // source: https://devzone.nordicsemi.com/f/nordic-q-a/391/uart-baudrate-register-values
        //
        // We want to stop RX if line is idle for 2 bytes worth of time
        // That is 20 bits (each byte is 1 start bit + 8 data bits + 1 stop bit)
        // This gives us the amount of 16M ticks for 20 bits.
        let timeout = (0x8000_0000 / (config.baudrate as u32 / 40)) as u16;

        timer.set_frequency(Frequency::F16MHz);
        timer.cc(0).write(timeout);
        timer.cc(0).short_compare_clear();
        timer.cc(0).short_compare_stop();

        let mut ppi_ch1 = Ppi::new(ppi_ch1.degrade_configurable());
        ppi_ch1.set_event(Event::from_reg(&r.events_rxdrdy));
        ppi_ch1.set_task(timer.task_clear());
        // The nrf51's PPI has no fork tasks,
        // so we instead just manually start the timer when the first `rxdrdy` of a reception occurs.
        #[cfg(not(feature = "nrf51"))]
        ppi_ch1.set_fork_task(timer.task_start());
        ppi_ch1.enable();

        let mut ppi_ch2 = Ppi::new(ppi_ch2.degrade_configurable());
        ppi_ch2.set_event(timer.cc(0).event_compare());
        ppi_ch2.set_task(Task::from_reg(&r.tasks_stoprx));
        ppi_ch2.enable();

        Self {
            inner: PeripheralMutex::new_unchecked(irq, &mut state.0, move || StateInner {
                phantom: PhantomData,
                timer,
                _ppi_ch1: ppi_ch1,
                _ppi_ch2: ppi_ch2,

                rx: RingBuffer::new(rx_buffer),
                rx_state: RxState::Idle,
                rx_waker: WakerRegistration::new(),

                tx: RingBuffer::new(tx_buffer),
                tx_state: TxState::Idle,
                tx_waker: WakerRegistration::new(),
            }),
        }
    }

    pub fn set_baudrate(&mut self, baudrate: Baudrate) {
        self.inner.with(|state| {
            let r = U::regs();

            let timeout = (0x8000_0000 / (baudrate as u32 / 40)) as u16;
            state.timer.cc(0).write(timeout);
            state.timer.clear();

            r.baudrate.write(|w| w.baudrate().variant(baudrate));
        });
    }
}

impl<'d, U, T> AsyncBufRead for BufferedUart<'d, U, T>
where
    U: UartInstance,
    T: TimerInstance + SupportsBitmode<u16>,
{
    fn poll_fill_buf(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<&[u8]>> {
        self.inner.with(|state| {
            // Conservative compiler fence to prevent optimizations that do not
            // take in to account actions by DMA. The fence has been placed here,
            // before any DMA action has started
            compiler_fence(Ordering::SeqCst);
            trace!("poll_read");

            // We have data ready in buffer? Return it.
            let buf = state.rx.pop_buf();
            if !buf.is_empty() {
                trace!("  got {:?} {:?}", buf.as_ptr() as u32, buf.len());
                let buf: &[u8] = buf;
                let buf: &[u8] = unsafe { mem::transmute(buf) };
                return Poll::Ready(Ok(buf));
            }

            trace!("  empty");
            state.rx_waker.register(cx.waker());
            Poll::<Result<&[u8]>>::Pending
        })
    }

    fn consume(mut self: Pin<&mut Self>, amt: usize) {
        self.inner.with(|state| {
            trace!("consume {:?}", amt);
            state.rx.pop(amt);
        });
        self.inner.pend();
    }
}

impl<'d, U, T> AsyncWrite for BufferedUart<'d, U, T>
where
    U: UartInstance,
    T: TimerInstance + SupportsBitmode<u16>,
{
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<Result<usize>> {
        let poll = self.inner.with(|state| {
            trace!("poll_write: {:?}", buf.len());

            let tx_buf = state.tx.push_buf();
            if tx_buf.is_empty() {
                trace!("poll_write: pending");
                state.tx_waker.register(cx.waker());
                return Poll::Pending;
            }

            let n = min(tx_buf.len(), buf.len());
            tx_buf[..n].copy_from_slice(&buf[..n]);
            state.tx.push(n);

            trace!("poll_write: queued {:?}", n);

            // Conservative compiler fence to prevent optimizations that do not
            // take in to account actions by DMA. The fence has been placed here,
            // before any DMA action has started
            compiler_fence(Ordering::SeqCst);

            Poll::Ready(Ok(n))
        });

        self.inner.pend();

        poll
    }
}

impl<'a, U: UartInstance, T: TimerInstance + SupportsBitmode<u16>> Drop for BufferedUart<'a, U, T> {
    fn drop(&mut self) {
        let r = U::regs();

        self.inner.with(|state| {
            state.timer.stop();

            if state.rx_state == RxState::Receiving {
                r.tasks_stoprx.write(|w| unsafe { w.bits(1) });

                // Mark the rx buffer as full so that we don't just immediately start recieving again.
                // We need to do it twice because `push_buf` only returns the first contiguous part of the buffer;
                // there can also be a second contiguous part after the buffer wraps around.
                for _ in 0..2 {
                    let len = state.rx.push_buf().len();
                    state.rx.push(len);
                }
            }
        });

        #[cfg(not(feature = "nrf51"))]
        r.intenset.write(|w| w.rxto().set());
        low_power_wait_until(|| {
            self.inner.with(|state| {
                // The only way that we could have reached this point without the interrupt firing and starting at least one reception
                // is if we were in a critical section all the way from construction to drop.
                //
                // If that's the case, the interrupt will never fire and this'll deadlock anyway, so it's not really a problem.
                r.events_rxto.read().bits() != 0 && state.tx_state == TxState::Idle
            })
        });

        r.enable.write(|w| w.enable().disabled());

        #[cfg(not(feature = "nrf51"))]
        {
            gpio::deconfigure_pin(r.psel.rxd.read().bits());
            gpio::deconfigure_pin(r.psel.txd.read().bits());
            gpio::deconfigure_pin(r.psel.rts.read().bits());
            gpio::deconfigure_pin(r.psel.cts.read().bits());
        }
        #[cfg(feature = "nrf51")]
        {
            gpio::deconfigure_pin(r.pselrxd.read().bits());
            gpio::deconfigure_pin(r.pseltxd.read().bits());
            gpio::deconfigure_pin(r.pselrts.read().bits());
            gpio::deconfigure_pin(r.pselcts.read().bits());
        }
    }
}

impl<'a, U, T> PeripheralState for StateInner<'a, U, T>
where
    U: UartInstance,
    T: TimerInstance + SupportsBitmode<u16>,
{
    type Interrupt = U::Interrupt;
    fn on_interrupt(&mut self) {
        trace!("irq: start");
        let r = U::regs();

        loop {
            match self.rx_state {
                RxState::Idle => {
                    trace!("  irq_rx: in state idle");

                    let buf = self.rx.push_buf();
                    if !buf.is_empty() {
                        trace!("  irq_rx: starting {:?}", buf.len());
                        self.rx_state = RxState::Receiving;

                        #[cfg(not(feature = "nrf51"))]
                        {
                            // Set up the DMA read
                            r.rxd.ptr.write(|w|
                                // The PTR field is a full 32 bits wide and accepts the full range
                                // of values.
                                unsafe { w.ptr().bits(buf.as_ptr() as u32) });
                            r.rxd.maxcnt.write(|w|
                                // We're giving it the length of the buffer, so no danger of
                                // accessing invalid memory. We have verified that the length of the
                                // buffer fits in an `u8`, so the cast to `u8` is also fine.
                                //
                                // The MAXCNT field is at least 8 bits wide and accepts the full
                                // range of values.
                                unsafe { w.maxcnt().bits(buf.len() as _) });
                            trace!("  irq_rx: buf {:?} {:?}", buf.as_ptr() as u32, buf.len());
                        }

                        // Only clear RXTO when we start RX, so that if RXTO has already occured when `BufferedUart` has dropped it remains triggered.
                        r.events_rxto.reset();

                        // Start UARTE Receive transaction
                        r.tasks_startrx.write(|w|
                            // `1` is a valid value to write to task registers.
                            unsafe { w.bits(1) });

                        #[cfg(feature = "nrf51")]
                        {
                            // There's no ENDRX on the nrf51,
                            // so the only way we can trigger the interrupt idle timer triggered STOPRX
                            // is by using RXTO.
                            // We want to leave the event set after reception is finished though,
                            // so that we can check for it in the destructor.
                            // So only have the interrupt for it enabled when we're currently recieving.
                            r.intenset.write(|w| w.rxto().set());

                            // Drain any bytes which were already buffered in the FIFO.
                            continue;
                        }
                    }
                    break;
                }
                RxState::Receiving => {
                    trace!("  irq_rx: in state receiving");
                    #[cfg(feature = "nrf51")]
                    // Make this a while loop to drain the FIFO when reception is restarted.
                    while r.events_rxdrdy.read().bits() != 0 {
                        // Start the idle timer if it hasn't been already.
                        // There's no way to check if it's already running.
                        self.timer.start();

                        let buf = self.rx.push_buf();
                        if let Some(byte) = buf.first_mut() {
                            r.events_rxdrdy.reset();
                            *byte = r.rxd.read().rxd().bits();
                            self.rx.push(1);
                            if self.rx.push_buf().is_empty() {
                                r.tasks_stoprx.write(|w| unsafe { w.bits(1) });
                                self.timer.stop();
                                self.rx_waker.wake();
                                self.rx_state = RxState::Idle;
                                continue;
                            }
                        }
                    }
                    #[cfg(feature = "nrf51")]
                    if r.events_rxto.read().bits() != 0 {
                        // We want RXTO to remain fired so we know RX is done in the destructor.
                        // So clear the interrupt instead of the event to stop it being re-fired.
                        r.intenclr.write(|w| w.rxto().clear());
                        self.rx_waker.wake();
                        self.rx_state = RxState::Idle;
                        continue;
                    }
                    #[cfg(not(feature = "nrf51"))]
                    if r.events_endrx.read().bits() != 0 {
                        self.timer.stop();

                        let n: usize = r.rxd.amount.read().amount().bits() as usize;
                        trace!("  irq_rx: endrx {:?}", n);
                        self.rx.push(n);

                        r.events_endrx.reset();

                        self.rx_waker.wake();
                        self.rx_state = RxState::Idle;
                        continue;
                    }
                    break;
                }
            }
        }

        loop {
            match self.tx_state {
                TxState::Idle => {
                    trace!("  irq_tx: in state Idle");
                    let buf = self.tx.pop_buf();
                    #[allow(unused_variables)]
                    if let Some(byte) = buf.first() {
                        trace!("  irq_tx: starting {:?}", buf.len());
                        self.tx_state = TxState::Transmitting(buf.len());

                        #[cfg(not(feature = "nrf51"))]
                        {
                            // Set up the DMA write
                            r.txd.ptr.write(|w|
                                // The PTR field is a full 32 bits wide and accepts the full range
                                // of values.
                                unsafe { w.ptr().bits(buf.as_ptr() as u32) });
                            r.txd.maxcnt.write(|w|
                                // We're giving it the length of the buffer, so no danger of
                                // accessing invalid memory. We have verified that the length of the
                                // buffer fits in an `u8`, so the cast to `u8` is also fine.
                                //
                                // The MAXCNT field is 8 bits wide and accepts the full range of
                                // values.
                                unsafe { w.maxcnt().bits(buf.len() as _) });
                        }

                        // Start UARTE Transmit transaction
                        r.tasks_starttx.write(|w|
                            // `1` is a valid value to write to task registers.
                            unsafe { w.bits(1) });

                        #[cfg(feature = "nrf51")]
                        // Set the initial byte to transmit.
                        {
                            r.txd.write(|w| unsafe { w.txd().bits(*byte) });
                            self.tx.pop(1);
                        }
                    }
                    break;
                }
                #[allow(unused_variables)]
                TxState::Transmitting(n) => {
                    trace!("  irq_tx: in state Transmitting");
                    #[cfg(feature = "nrf51")]
                    if r.events_txdrdy.read().bits() != 0 {
                        r.events_txdrdy.reset();
                        let buf = self.tx.pop_buf();
                        if let Some(byte) = buf.first() {
                            r.txd.write(|w| unsafe { w.txd().bits(*byte) });
                            self.tx.pop(1);
                        } else {
                            self.tx_state = TxState::Idle;
                            continue;
                        }
                    }

                    #[cfg(not(feature = "nrf51"))]
                    if r.events_endtx.read().bits() != 0 {
                        r.events_endtx.reset();

                        trace!("  irq_tx: endtx {:?}", n);
                        self.tx.pop(n);
                        self.tx_waker.wake();
                        self.tx_state = TxState::Idle;
                        continue;
                    }
                    break;
                }
            }
        }

        #[cfg(not(feature = "nrf51"))]
        if r.events_rxto.read().bits() != 0 {
            r.intenclr.write(|w| w.rxto().clear());
        }

        trace!("irq: end");
    }
}
