#![macro_use]

mod uarte;
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
