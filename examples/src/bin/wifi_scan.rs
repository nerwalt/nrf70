#![no_std]
#![no_main]

use align_data::{Align16, include_aligned};
use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::spim::{self, Spim};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use nrf70::bus::SpiBus;
use nrf70::control::scan::*;
use nrf70_examples::*;
use static_cell::StaticCell;
use {defmt_rtt as _, embassy_nrf as _, panic_probe as _};

static FW: &[u8] = include_aligned!(
    Align16,
    "../../../third_party/nrf70-bm/sdk-nrfxlib/nrf_wifi/bin/ncs/scan_only/nrf70.bin"
);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Wifi Scan Example");

    let p = embassy_nrf::init(Default::default());

    info!("Setting up hardware");
    let bucken = Output::new(p.P0_12, Level::Low, OutputDrive::HighDrive);
    let iovdd_ctl = Output::new(p.P0_31, Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23, Pull::None);

    info!("Setting up SPI");
    let sck = p.P0_17;
    let mosi = p.P0_13;
    let miso = p.P0_14;
    let cs = p.P0_18;
    let config = spim::Config::default();
    let spi = Spim::new(p.SERIAL0, Irqs, sck, miso, mosi, config);
    let cs = Output::new(cs, Level::High, OutputDrive::HighDrive);
    let spi = ExclusiveDevice::new(spi, cs, Delay).unwrap();
    let bus = SpiBus::new(spi);

    info!("Setting up NRF70 State");
    static STATE: StaticCell<nrf70::State> = StaticCell::new();
    let state = STATE.init(nrf70::State::new());

    info!("Initializing the NRF70");
    let (_device, mut control, runner) = nrf70::new(state, bus, bucken, iovdd_ctl, host_irq).await;
    unwrap!(spawner.spawn(nrf70_task(runner)));

    info!("Initializing control");
    match control.init(FW).await {
        Ok(()) => (),
        Err(error) => error!("Failed to initialize {:?}", error),
    };

    let mut ssids = heapless::Vec::<heapless::String<32>, 2>::new();
    let mut ssid = heapless::String::<32>::new();
    let _ = ssid.push_str("slaphappy");
    ssids.push(ssid).unwrap();


    for n in 0..3 {

        info!("Triggering scan {}", n+1);
        let mut scan_options = ScanOptions::default();
        scan_options.scan_type = ScanType::Passive;
        scan_options.dwell_time = Some(Duration::from_millis(500));

        match control.scan(scan_options).await {
            Ok(()) => info!("Scan complete"),
            Err(error) => error!("Failed to perform scan {}", error),
        }

        match control.get_scan_results().await {
            Ok(results) => {
                info!("Scan results");
                for ap in results.aps {
                    info!(" {:?}", ap);
                }
            }
            Err(error) => error!("Failed to get scan results: {}", error),
        }
        info!("");

        if n < 2 {
            let wait = Duration::from_secs(30);
            info!("Waiting {} seconds before next scan", wait.as_secs());
            Timer::after(wait).await;
        }
    }

    info!("Blinky");
    let mut led = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
