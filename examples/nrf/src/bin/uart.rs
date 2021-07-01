#![no_std]
#![no_main]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

#[path = "../example_common.rs"]
mod example_common;
use example_common::*;

use defmt::panic;
use embassy::executor::Spawner;
use embassy::traits::uart::{Read, Write};
use embassy_nrf::gpio::NoPin;
use embassy_nrf::{interrupt, uart, Peripherals};

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let mut config = uart::Config::default();
    config.parity = uart::Parity::EXCLUDED;
    config.baudrate = uart::Baudrate::BAUD115200;

    let irq = interrupt::take!(UARTE0_UART0);
    let mut uart =
        unsafe { uart::Uart::new(p.UARTE0, irq, p.P0_08, p.P0_06, NoPin, NoPin, config) };

    info!("uart initialized!");

    // Message must be in SRAM
    let mut buf = [0; 8];
    buf.copy_from_slice(b"Hello!\r\n");

    unwrap!(uart.write(&buf).await);
    info!("wrote hello in uart!");

    loop {
        info!("reading...");
        unwrap!(uart.read(&mut buf).await);
        info!("writing...");
        unwrap!(uart.write(&buf).await);
    }
}
