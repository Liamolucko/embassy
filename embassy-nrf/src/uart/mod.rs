#![macro_use]

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
