use core::result::Result::Ok;

use embedded_hal::{blocking::delay, prelude::_embedded_hal_blocking_delay_DelayMs};
use shared_bus::{self, I2cProxy};

use async_broadcast::{broadcast, Receiver, Sender};
// use ens160::{AirqualityIndex, Ens160};
use esp_idf_hal::{i2c::*, peripherals::Peripherals, prelude::*};
use esp_idf_sys::{self as _, esp, esp_app_desc}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;
use sht4x::Sht4x;
use tokio::{self, runtime::Handle, task::block_in_place};

use esp32_threads_demo::ens160::{AirqualityIndex, Ens160};

use anyhow::Result;

esp_app_desc!();

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Setting up eventfd...");
    let config = esp_idf_sys::esp_vfs_eventfd_config_t { max_fds: 2 };
    esp! { unsafe { esp_idf_sys::esp_vfs_eventfd_register(&config) } }?;

    info!("Setting up board...");
    let peripherals = Peripherals::take().unwrap();

    info!("Starting async run loop...");
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(async_main(peripherals))?;

    info!("Shutting down...");
    Ok(())
}

async fn async_main(peripherals: Peripherals) -> Result<()> {
    let sda = peripherals.pins.gpio3;
    let scl = peripherals.pins.gpio2;
    let i2c = peripherals.i2c0;

    let config = I2cConfig::new().baudrate(100.kHz().into());
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;

    let bus: &'static _ = shared_bus::new_std!(I2cDriver = driver).unwrap();

    let (mut s1, r1) = broadcast(1);
    let (mut sender_ens160, _) = broadcast(1);
    let (sender_sht45, receiver_sht45) = broadcast(1);

    s1.set_overflow(true);
    sender_ens160.set_overflow(true);

    let t1 = tokio::spawn(sensor(s1));

    let t2 = tokio::spawn(sensor2(r1));

    // let sht45_handler: tokio::task::JoinHandle<()> =
    //     tokio::spawn(sht45_sensor(sender_sht45, bus.acquire_i2c()));

    let ens160_handler = tokio::spawn(ens160_sensor(
        sender_ens160,
        receiver_sht45,
        bus.acquire_i2c(),
    ));

    // let _ = t1.await?;
    // let _ = t2.await?;
    // sht45_handler.await?;
    match ens160_handler.await? {
        Ok(_) => info!("ens160_handler: finished"),
        Err(e) => warn!("ens160_handler: error: {:?}", e),
    };

    loop {
        info!("main: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

type Sht45Reading = (u16, f32);
type Ens160Reading = (u16, u16, AirqualityIndex);

async fn ens160_sensor(
    tx: Sender<Ens160Reading>,
    mut rx: Receiver<Sht45Reading>,
    driver: I2cProxy<'static, std::sync::Mutex<I2cDriver<'_>>>,
) -> Result<()> {
    info!("ens160_sensor: setup...");
    let mut sensor = tokio::time::timeout(
        std::time::Duration::from_secs(10),
        tokio::task::spawn_blocking(|| {
            let mut sensor = Ens160::new(driver, 0x53);
            info!("ens160_sensor: reseting...");
            sensor.reset();
            info!("ens160_sensor: device reset...");
            Handle::current().block_on(async {
                tokio::time::sleep(std::time::Duration::from_millis(250)).await;
            });
            info!("ens160_sensor: switch to operational mode...");
            sensor.operational();
            Handle::current().block_on(async {
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            });
            sensor
        }),
    )
    .await??;

    loop {
        info!("ens160_sensor: checking sensor status...");
        if let core::result::Result::Ok(status) = sensor.status() {
            // Wait for data to be ready
            if status.data_is_ready() {
                info!("ens160_sensor: check for correction values...");
                if let core::result::Result::Ok((hum, temp)) = rx.try_recv() {
                    sensor.set_hum(hum).ok();
                    sensor.set_temp((temp * 100_f32) as i16).ok();
                }

                info!("ens160_sensor: reading measurements...");
                let tvoc = sensor.tvoc().unwrap();
                let eco2 = sensor.eco2().unwrap();
                let aqi = sensor.airquality_index().unwrap();
                info!("ens160_sensor: Broadcasting...");
                tx.broadcast((tvoc, *eco2, aqi)).await.ok();
            }
        }

        info!("ens160_sensor: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

async fn sht45_sensor(
    tx: Sender<Sht45Reading>,
    driver: I2cProxy<'_, std::sync::Mutex<I2cDriver<'_>>>,
) {
    info!("sht45_sensor: setup...");
    let mut delay = CustomDelay {};
    let mut sensor = Sht4x::new(driver);

    loop {
        info!("sht45_sensor: taking measurement...");
        let measurement = sensor.measure(sht4x::Precision::Low, &mut delay);

        info!("sht45_sensor: evaluating measurands...");
        if let core::result::Result::Ok(val) = measurement {
            info!("sht45_sensor: Broadcasting measurements...");
            tx.broadcast((
                val.humidity_percent().to_num(),
                val.temperature_celsius().to_num(),
            ))
            .await
            .ok();
        }

        info!("sht45_sensor: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

struct CustomDelay {}

impl embedded_hal::blocking::delay::DelayMs<u16> for CustomDelay {
    fn delay_ms(&mut self, ms: u16) {
        info!("Delay provider: sleeping... {ms}");
        Handle::current().block_on(async {
            tokio::time::sleep(std::time::Duration::from_millis(ms as u64)).await;
        });
    }
}

async fn sensor(tx: Sender<String>) -> Result<()> {
    for i in 0..10 {
        info!("Broadcasting: {}", i);
        tx.broadcast(format!("{i}")).await?;
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
    Ok(())
}

async fn sensor2(mut rx: Receiver<String>) -> Result<()> {
    loop {
        let data = match rx.recv().await {
            core::result::Result::Ok(x) => x,
            core::result::Result::Err(e) => match e {
                async_broadcast::RecvError::Closed => {
                    warn!("Channel closed");
                    break;
                }
                async_broadcast::RecvError::Overflowed(num) => {
                    warn!("Channel overflow, missed {num} message(s)");
                    continue;
                }
            },
        };
        info!("Received: {}", data);
        tokio::time::sleep(std::time::Duration::from_secs(5)).await;
    }

    Ok(())
}
