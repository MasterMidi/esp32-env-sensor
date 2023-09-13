#![feature(async_closure)]

use core::result::Result::Ok;

use embedded_svc::{
    mqtt::client::{Event, QoS},
    wifi::{AuthMethod, ClientConfiguration, Configuration},
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    mqtt::client::{EspMqttClient, MqttClientConfiguration},
    nvs::EspDefaultNvsPartition,
    timer::EspTaskTimerService,
    wifi::{AsyncWifi, EspWifi},
};
use scd4x::scd4x::Scd4x;
use serde::Serialize;
use shared_bus::{self, BusManager, I2cProxy};

use async_broadcast::{broadcast, Receiver, Sender, TryRecvError};
use esp_idf_hal::{cpu::core, i2c::*, peripherals::Peripherals, prelude::*};
use esp_idf_sys::{self as _, esp, esp_app_desc, EspError}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;
use sht4x::Sht4x;
use tokio::{self, runtime::Handle, task};

use esp32_env_sensor::{
    ens160::{AirqualityIndex, Ens160},
    sensors::{i2c_sensor::I2CSensor, pmsa003i},
};

use esp32_env_sensor::sensors::pmsa003i::Pmsa003i;

use anyhow::{Error, Result};

use std::{self, fmt::Display, sync::Mutex};

#[toml_cfg::toml_config]
pub struct Config {
    #[default("localhost")]
    mqtt_host: &'static str,
    #[default("")]
    mqtt_user: &'static str,
    #[default("")]
    mqtt_pass: &'static str,
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

esp_app_desc!();

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Setting up board...");
    let config = esp_idf_sys::esp_vfs_eventfd_config_t { max_fds: 1 };
    esp! { unsafe { esp_idf_sys::esp_vfs_eventfd_register(&config) } }?;

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take()?;
    let timer = EspTaskTimerService::new()?;
    let nvs = EspDefaultNvsPartition::take()?;

    info!("Initializing Wi-Fi module...");
    let wifi = AsyncWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
        timer.clone(),
    )?;

    info!("Initializing I2C bus...");
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = driver).unwrap();

    info!("Starting program loop...");
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?
        .block_on(async_main(bus, wifi))
}

async fn async_main(
    i2c_bus: &'static BusManager<Mutex<I2cDriver<'static>>>,
    wifi: AsyncWifi<EspWifi<'static>>,
) -> Result<()> {
    info!("main async: running on core: {:?}", core());
    let cfg = CONFIG;

    info!("Setting up wifi...");
    let mut wifi_loop = WifiLoop { wifi };
    wifi_loop.configure(cfg.wifi_ssid, cfg.wifi_psk).await?;
    wifi_loop.initial_connect().await?;

    info!("setting up MQTT client...");
    let broker_url = format!("mqtt://{}", cfg.mqtt_host);
    let mqtt_config = MqttClientConfiguration {
        client_id: "esp32".into(),
        ..Default::default()
    };
    let mqtt = EspMqttClient::new(broker_url, &mqtt_config, move |msg| match msg {
        Ok(event) => match event {
            Event::Received(msg) => {
                info!(
                    "MQTT Message: {} -> {:?}",
                    msg.id(),
                    String::from_utf8(msg.data().to_vec())
                );
            }
            _ => info!("MQTT Message: {:?}", event),
        },
        Err(e) => info!("MQTT Message ERROR: {}", e),
    })?;

    info!("async_main: setup broadcast channels...");
    let (mut sender_ens160, receiver_ens160) = broadcast(1);
    let (mut sender_sht45, receiver_sht45) = broadcast(1);
    let (mut sender_scd4x, receiver_scd4x) = broadcast(1);
    let (mut sender_pmsa003i, receiver_pmsa003i) = broadcast(1);

    sender_ens160.set_overflow(true);
    sender_sht45.set_overflow(true);
    sender_scd4x.set_overflow(true);
    sender_pmsa003i.set_overflow(true);

    info!("starting sensor loops...");
    tokio::spawn(ens160_sensor(
        sender_ens160,
        receiver_sht45.clone(),
        i2c_bus.acquire_i2c(),
    ));
    tokio::spawn(sht45_sensor(sender_sht45, i2c_bus.acquire_i2c()));
    tokio::spawn(scd41_sensor(i2c_bus.acquire_i2c(), sender_scd4x));
    // tokio::spawn(pmsa003i_sensor(i2c_bus.acquire_i2c(), sender_pmsa003i));

    info!("starting MQTT publish loop...");
    tokio::spawn(publish_loop(
        mqtt,
        receiver_ens160,
        receiver_sht45,
        receiver_scd4x,
        receiver_pmsa003i,
    ));

    info!("Entering main Wi-Fi run loop...");
    wifi_loop.stay_connected().await?;

    Ok(())
}

fn next_message<T>(recv: &mut Receiver<T>) -> Result<T, TryRecvError>
where
    T: Clone,
{
    let res = recv.try_recv();
    if let Err(TryRecvError::Overflowed(_)) = res {
        return recv.try_recv();
    }
    res
}

async fn publish_loop(
    mut mqtt: EspMqttClient,
    mut receiver_ens160: Receiver<Ens160Reading>,
    mut receiver_sht45: Receiver<Sht4xReading>,
    mut receiver_scd4x: Receiver<Scd4xReading>,
    mut receiver_pmsa003i: Receiver<Pmsa003iReading>,
) -> Result<()> {
    loop {
        if let Ok(reading) = next_message(&mut receiver_ens160) {
            mqtt.publish(
                "esp32_testing/ens160",
                QoS::AtMostOnce,
                false,
                serde_json::to_string(&reading)?.as_bytes(),
            )?;
        }

        if let Ok(reading) = next_message(&mut receiver_sht45) {
            mqtt.publish(
                "esp32_testing/sht45",
                QoS::AtMostOnce,
                false,
                serde_json::to_string(&reading)?.as_bytes(),
            )?;
        }

        if let Ok(reading) = next_message(&mut receiver_scd4x) {
            mqtt.publish(
                "esp32_testing/scd4x",
                QoS::AtMostOnce,
                false,
                serde_json::to_string(&reading)?.as_bytes(),
            )?;
        }

        if let Ok(reading) = next_message(&mut receiver_pmsa003i) {
            mqtt.publish(
                "esp32_testing/pmsa003i",
                QoS::AtMostOnce,
                false,
                serde_json::to_string(&reading)?.as_bytes(),
            )?;
        }

        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

#[derive(Debug, Clone, Serialize)]
struct Ens160Reading {
    tvoc: u16,
    eco2: u16,
    aqi: AirqualityIndex,
}

#[derive(Debug, Clone, Serialize)]
struct Sht4xReading {
    humidity: u16,
    temperature: f32,
}

#[derive(Debug, Clone, Serialize)]
struct Scd4xReading {
    co2: u16,
    humidity: f32,
    temperature: f32,
}

#[derive(Debug, Clone, Serialize)]
struct Pmsa003iReading {
    pm1: u16,
    pm2_5: u16,
    pm10: u16,
}

struct Co2 {
    value: u16,
    unit: &'static str,
}

async fn pmsa003i_setup(
    sensor: &mut Pmsa003i<I2cProxy<'static, Mutex<I2cDriver<'_>>>>,
) -> Result<()> {
    info!(
        "pmsa003i_sensor: setup: running task: {:?} on {:?}",
        task::try_id().unwrap(),
        core()
    );

    sensor.version();

    Ok(())
}

async fn pmsa003i_sensor(
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
    tx: Sender<Pmsa003iReading>,
) -> Result<()> {
    let mut sensor = Pmsa003i::new(driver, pmsa003i::ADDRESS);

    while let Err(err) = pmsa003i_setup(&mut sensor).await {
        warn!("pmsa003i_sensor: error: {:?}", err);
        tokio::time::sleep(std::time::Duration::from_millis(50000)).await;
    }

    loop {
        info!("pmsa003i_sensor: reading measurements...");
        match sensor.read() {
            Ok(reading) => {
                info!("pmsa003i_sensor: Broadcasting...");
                tx.broadcast(Pmsa003iReading {
                    pm1: reading.pm1(),
                    pm2_5: reading.pm2_5(),
                    pm10: reading.pm10(),
                })
                .await?;
            }
            Err(e) => {
                warn!("pmsa003i_sensor: error: {:?}", e);
            }
        };
    }
}

async fn ens160_setup(sensor: &mut Ens160<I2cProxy<'static, Mutex<I2cDriver<'_>>>>) -> Result<()> {
    info!(
        "ens160_sensor: setup: running task: {:?} on {:?}",
        task::try_id(),
        core()
    );
    info!("ens160_sensor: reseting...");
    sensor.reset()?;
    tokio::time::sleep(std::time::Duration::from_millis(250)).await;

    info!("ens160_sensor: switch to operational mode...");
    sensor.operational()?;

    info!("ens160: wating for sensor to warm up...");

    Ok(())
}

async fn ens160_sensor(
    tx: Sender<Ens160Reading>,
    mut rx: Receiver<Sht4xReading>,
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
) -> Result<()> {
    let mut sensor = Ens160::new(driver, 0x53);

    while let Err(err) = ens160_setup(&mut sensor).await {
        warn!("ens160_sensor: error: {:?}", err);
        tokio::time::sleep(std::time::Duration::from_millis(50000)).await;
    }

    loop {
        info!("ens160_sensor: checking sensor status...");
        if let Ok(status) = sensor.status() {
            // Wait for data to be ready
            if status.data_is_ready() {
                if let Ok(reading) = next_message(&mut rx) {
                    info!("ens160_sensor: check for correction values...");
                    sensor.set_hum(reading.humidity * 100)?;
                    sensor.set_temp((reading.temperature * 100_f32) as i16)?;
                }

                info!("ens160_sensor: reading measurements...");
                let tvoc = sensor.tvoc()?;
                let eco2 = sensor.eco2()?;
                let aqi = sensor.airquality_index()?;
                info!("ens160_sensor: Broadcasting...");
                tx.broadcast(Ens160Reading {
                    tvoc,
                    eco2: *eco2,
                    aqi,
                })
                .await?;
            }
        }

        info!("ens160_sensor: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

async fn scd4x_setup(
    sensor: &mut Scd4x<I2cProxy<'_, Mutex<I2cDriver<'_>>>, CustomDelay>,
) -> Result<()> {
    info!(
        "scd41_sensor: setup: running task: {:?} on {:?}",
        task::try_id(),
        core()
    );
    info!("scd41_sensor: initializing...");
    sensor.wake_up();
    match sensor.stop_periodic_measurement() {
        Ok(_) => {}
        Err(e) => return Err(Error::msg(format!("{e:?}"))),
    }; // TODO: uses custom error type not implementing Error trait
    match sensor.reinit() {
        Ok(_) => {}
        Err(e) => return Err(Error::msg(format!("{e:?}"))),
    };

    let serial = match sensor.serial_number() {
        Ok(x) => x,
        Err(e) => return Err(Error::msg(format!("{e:?}"))),
    };
    info!("scd41_sensor: serial: {:#04x}", serial);

    match sensor.start_periodic_measurement() {
        Ok(_) => {}
        Err(e) => return Err(Error::msg(format!("{e:?}"))),
    };

    Ok(())
}

async fn scd41_sensor(
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
    tx: Sender<Scd4xReading>,
) -> Result<()> {
    let delay = CustomDelay {};
    let mut sensor = Scd4x::new(driver, delay);

    while let Err(err) = scd4x_setup(&mut sensor).await {
        warn!("scd41_sensor: error: {:?}", err);
        tokio::time::sleep(std::time::Duration::from_millis(50000)).await;
    }

    loop {
        info!("scd41_sensor: sleeping 5 secs...");
        tokio::time::sleep(std::time::Duration::from_secs(5)).await;

        info!("scd41_sensor: taking measurement...");
        if let Ok(measurement) = sensor.measurement() {
            info!("sht45_sensor: Broadcasting measurements...");
            tx.broadcast(Scd4xReading {
                co2: measurement.co2,
                humidity: measurement.humidity,
                temperature: measurement.temperature,
            })
            .await?;
        };
    }
}

async fn sht45_setup(
    sensor: &mut Sht4x<I2cProxy<'static, Mutex<I2cDriver<'static>>>, CustomDelay>,
) -> Result<()> {
    info!(
        "sht45_sensor: setup: running task: {:?} on {:?}",
        task::try_id(),
        core()
    );
    info!("sht45_sensor: verifying connection...");
    let mut delay = CustomDelay {};
    match sensor.serial_number(&mut delay) {
        Ok(serial) => info!("sht45_sensor: serial: {:#04x}", serial),
        Err(e) => return Err(Error::msg(format!("{e:?}"))),
    };
    Ok(())
}

async fn sht45_sensor(
    tx: Sender<Sht4xReading>,
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
) -> Result<()> {
    let mut delay = CustomDelay {};
    let mut sensor = Sht4x::new(driver);

    while let Err(err) = sht45_setup(&mut sensor).await {
        warn!("sht45_sensor: error: {:?}", err);
        tokio::time::sleep(std::time::Duration::from_millis(50000)).await;
    }

    loop {
        info!("sht45_sensor: taking measurement...");
        let result = sensor.measure(sht4x::Precision::High, &mut delay);

        info!("sht45_sensor: evaluating measurands...");
        match result {
            Ok(val) => {
                info!("sht45_sensor: Broadcasting measurements...");
                tx.broadcast(Sht4xReading {
                    humidity: val.humidity_percent().to_num(),
                    temperature: val.temperature_celsius().to_num(),
                })
                .await
                .ok();
            }
            Err(e) => {
                warn!("sht45_sensor: error: {:?}", e);
            }
        }

        info!("sht45_sensor: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

struct CustomDelay {}

impl<T> embedded_hal::blocking::delay::DelayMs<T> for CustomDelay
where
    T: Into<u64> + Display + Copy,
{
    fn delay_ms(&mut self, ms: T) {
        info!("Delay provider: sleeping... {ms}");
        Handle::current().block_on(async {
            tokio::time::sleep(std::time::Duration::from_millis(ms.into())).await;
        });
    }
}

pub struct WifiLoop<'a> {
    wifi: AsyncWifi<EspWifi<'a>>,
}

impl<'a> WifiLoop<'a> {
    pub async fn configure(&mut self, ssid: &str, psswd: &str) -> Result<(), EspError> {
        info!("Setting Wi-Fi credentials...");
        self.wifi
            .set_configuration(&Configuration::Client(ClientConfiguration {
                ssid: ssid.into(),
                password: psswd.into(),
                auth_method: AuthMethod::WPA2WPA3Personal,
                ..Default::default()
            }))?;

        info!("Starting Wi-Fi driver...");
        self.wifi.start().await
    }

    pub async fn initial_connect(&mut self) -> Result<(), EspError> {
        self.do_connect_loop(true).await
    }

    pub async fn stay_connected(mut self) -> Result<(), EspError> {
        self.do_connect_loop(false).await
    }

    async fn do_connect_loop(&mut self, exit_after_first_connect: bool) -> Result<(), EspError> {
        let wifi = &mut self.wifi;
        loop {
            // Wait for disconnect before trying to connect again.  This loop ensures
            // we stay connected and is commonly missing from trivial examples as it's
            // way too difficult to showcase the core logic of an example and have
            // a proper Wi-Fi event loop without a robust async runtime.  Fortunately, we can do it
            // now!
            wifi.wifi_wait(|| wifi.is_up(), None).await?;

            info!("Connecting to Wi-Fi...");
            wifi.connect().await?;

            info!("Waiting for association...");
            wifi.ip_wait_while(|| wifi.is_up().map(|s| !s), None)
                .await?;

            if exit_after_first_connect {
                return Ok(());
            }
        }
    }
}
