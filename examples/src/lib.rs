#![no_std]

use embassy_nrf::{bind_interrupts, spim};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::spim::Spim;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;

use nrf70::bus::SpiBus;

bind_interrupts!(pub struct Irqs {
    SERIAL0 => spim::InterruptHandler<embassy_nrf::peripherals::SERIAL0>;
});

pub type Nrf70SpiBus = SpiBus<ExclusiveDevice<Spim<'static>, Output<'static>, Delay>>;

#[embassy_executor::task]
pub async fn nrf70_task(mut runner: nrf70::Runner<'static, Nrf70SpiBus, Input<'static>, Output<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, nrf70::NetDriver<'static>>) -> ! {
    runner.run().await
}
