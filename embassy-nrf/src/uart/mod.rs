#![macro_use]

//! Async UART
//!
//! Async UART is provided in two flavors - this one and also [buffered_uart::BufferedUart].
//! The [Uart] here is useful for those use-cases where reading the UARTE peripheral is
//! exclusively awaited on. If the [Uart] is required to be awaited on with some other future,
//! for example when using `futures_util::future::select`, then you should consider
//! [buffered_uart::BufferedUart] so that reads may continue while processing these
//! other futures. If you do not then you may lose data between reads.
//!
//! An advantage that [Uart] has over [buffered_uart::BufferedUart] is that less
//! memory may be used given that buffers are passed in directly to its read and write
//! methods.

#[allow(clippy::module_inception)]
#[cfg(feature = "nrf51")]
mod uart;
#[cfg(feature = "nrf51")]
pub use uart::*;

#[cfg(not(feature = "nrf51"))]
mod uarte;
#[cfg(not(feature = "nrf51"))]
pub use uarte::*;

#[non_exhaustive]
pub struct Config {
    pub parity: Parity,
    pub baudrate: Baudrate,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            parity: Parity::EXCLUDED,
            baudrate: Baudrate::BAUD115200,
        }
    }
}
