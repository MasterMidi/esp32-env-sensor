use core::result::Result::Ok;

use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::EspDefaultNvsPartition,
    timer::EspTaskTimerService,
    wifi::{AsyncWifi, EspWifi},
};
use scd4x::scd4x::Scd4x;
use shared_bus::{self, BusManager, I2cProxy};

use async_broadcast::{broadcast, Receiver, Sender};
// use ens160::{AirqualityIndex, Ens160};
use esp_idf_hal::{cpu::core, i2c::*, peripherals::Peripherals, prelude::*};
use esp_idf_sys::{self as _, esp, esp_app_desc, EspError}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;
use sht4x::Sht4x;
use tokio::{
    self,
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    runtime::Handle,
    task::{JoinError, JoinHandle},
    time::error::Elapsed,
};

use esp32_threads_demo::ens160::{AirqualityIndex, Ens160};

use anyhow::Result;

use std::{self, fmt::Display, sync::Mutex, thread::sleep};

// Edit these or provide your own way of provisioning...
const WIFI_SSID: &str = "Asus RT-AX86U";
const WIFI_PASS: &str = "3zyn2dY&Gp";

// To test, run `cargo run`, then when the server is up, use `nc -v espressif 12345` from
// a machine on the same Wi-Fi network.
const TCP_LISTENING_PORT: u16 = 12345;

esp_app_desc!();

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Setting up eventfd...");
    let config = esp_idf_sys::esp_vfs_eventfd_config_t { max_fds: 1 };
    esp! { unsafe { esp_idf_sys::esp_vfs_eventfd_register(&config) } }?;

    info!("main: running on core: {:?}", core());

    info!("Setting up board...");
    let peripherals = Peripherals::take().unwrap();

    info!("Setting up Wi-Fi...");
    let sysloop = EspSystemEventLoop::take()?;
    let timer = EspTaskTimerService::new()?;
    let nvs = EspDefaultNvsPartition::take()?;

    info!("Initializing Wi-Fi...");
    let wifi = AsyncWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
        timer.clone(),
    )?;

    info!("Setting up I2C bus...");
    let sda = peripherals.pins.gpio3;
    let scl = peripherals.pins.gpio2;
    let i2c = peripherals.i2c1;
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = driver).unwrap();

    info!("Starting async run loop...");
    match tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(async_main(bus, wifi))
    {
        Ok(_) => info!("async_main: finished..."),
        Err(e) => {
            info!("Shutting down...");
            panic!("async_main: error: {:?}", e)
        }
    };

    todo!();
}

async fn async_main(
    bus: &'static BusManager<Mutex<I2cDriver<'static>>>,
    wifi: AsyncWifi<EspWifi<'_>>,
) -> Result<()> {
    info!("main async: running on core: {:?}", core());
    let mut wifi_loop = WifiLoop { wifi };
    wifi_loop.configure().await?;
    wifi_loop.initial_connect().await?;

    info!("Preparing to launch echo server...");
    tokio::spawn(echo_server());

    info!("async_main: setup broadcast channels...");
    let (mut s1, r1) = broadcast(1);
    // let (mut sender_ens160, _) = broadcast(1);
    // let (mut sender_sht45, receiver_sht45) = broadcast(1);
    // let (mut sender_scd4x, receiver_scd4x) = broadcast(1);

    s1.set_overflow(true);
    // sender_ens160.set_overflow(true);
    // sender_sht45.set_overflow(true);
    // sender_scd4x.set_overflow(true);

    info!("async_main: spawn sensor threads...");
    let _ = tokio::spawn(sensor(s1));
    let _ = tokio::spawn(sensor2(r1));

    // tokio::spawn(sht45_setup(bus.acquire_i2c())).await;
    // tokio::spawn(ens160_setup(bus.acquire_i2c())).await;

    // let ens160_handler = tokio::spawn(ens160_sensor(
    //     sender_ens160,
    //     receiver_sht45,
    //     bus.acquire_i2c(),
    // ));

    // let sht45_handler = tokio::spawn(sht45_sensor(sender_sht45, bus.acquire_i2c()));

    // let scd4x_handler = scd41_sensor(bus.acquire_i2c(), sender_scd4x);

    // let _ = t1.await?;
    // let _ = t2.await?;
    // sht45_handler.await?;
    // match ens160_handler.await {
    //     Ok(_) => info!("ens160_handler: finished"),
    //     Err(e) => warn!("ens160_handler: error: {:?}", e),
    // };

    info!("Entering main Wi-Fi run loop...");
    wifi_loop.stay_connected().await?;

    loop {
        info!("main: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

type Sht4xReading = (u16, f32);
type Scd4xReading = (u16, f32, f32);
type Ens160Reading = (u16, u16, AirqualityIndex);

async fn ens160_setup(
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
) -> Result<Result<Ens160<I2cProxy<'static, Mutex<I2cDriver<'static>>>>, JoinError>, Elapsed> {
    tokio::time::timeout(
        std::time::Duration::from_secs(1),
        tokio::task::spawn_blocking(move || {
            info!("sync ens160_sensor: setup: running on core: {:?}", core());
            info!("sync ens160_sensor: setup...");
            let mut sensor = Ens160::new(driver, 0x53);

            info!("sync ens160_sensor: reseting...");
            sensor.reset().ok();
            info!("sync ens160_sensor: device reset...");
            // sleep(std::time::Duration::from_millis(250));
            Handle::current().block_on(async {
                tokio::time::sleep(std::time::Duration::from_millis(250)).await;
            });

            info!("sync ens160_sensor: switch to operational mode...");
            sensor.operational().ok();
            // sleep(std::time::Duration::from_millis(50));
            Handle::current().block_on(async {
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            });

            sensor
        }),
    )
    .await
}

async fn ens160_sensor(
    tx: Sender<Ens160Reading>,
    mut rx: Receiver<Sht4xReading>,
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
    // mut sensor: Ens160<I2cProxy<'static, Mutex<I2cDriver<'_>>>>,
) -> Result<()> {
    info!("ens160_sensor: running on core: {:?}", core());
    let mut sensor = tokio::time::timeout(
        std::time::Duration::from_secs(10),
        tokio::task::spawn_blocking(move || {
            info!("ens160_sensor: setup: running on core: {:?}", core());
            info!("ens160_sensor: setup...");
            let mut sensor = Ens160::new(driver, 0x53);

            info!("ens160_sensor: reseting...");
            sensor.reset().ok();
            info!("ens160_sensor: device reset...");
            // sleep(std::time::Duration::from_millis(250));
            Handle::current().block_on(async {
                tokio::time::sleep(std::time::Duration::from_millis(250)).await;
            });

            info!("ens160_sensor: switch to operational mode...");
            sensor.operational().ok();
            // sleep(std::time::Duration::from_millis(50));
            Handle::current().block_on(async {
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
            });

            sensor
        }),
    )
    .await??;

    loop {
        info!("ens160_sensor: checking sensor status...");
        if let Ok(status) = sensor.status() {
            // Wait for data to be ready
            if status.data_is_ready() {
                info!("ens160_sensor: check for correction values...");
                if let core::result::Result::Ok((hum, temp)) = rx.try_recv() {
                    sensor.set_hum(hum)?;
                    sensor.set_temp((temp * 100_f32) as i16)?;
                }

                info!("ens160_sensor: reading measurements...");
                let tvoc = sensor.tvoc()?;
                let eco2 = sensor.eco2()?;
                let aqi = sensor.airquality_index()?;
                info!("ens160_sensor: Broadcasting...");
                tx.broadcast((tvoc, *eco2, aqi)).await?;
            }
        }

        info!("ens160_sensor: sleeping...");
        tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    }
}

async fn scd41_sensor(
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
    tx: Sender<Scd4xReading>,
) -> Result<()> {
    let mut sensor = tokio::time::timeout(
        std::time::Duration::from_secs(10),
        tokio::task::spawn_blocking(|| {
            info!("scd41_sensor: setup...");
            let delay = CustomDelay {};
            let mut sensor = Scd4x::new(driver, delay);
            info!("scd41_sensor: initializing...");
            sensor.wake_up();
            sensor.stop_periodic_measurement().ok();
            sensor.reinit().ok();

            let serial = sensor.serial_number().unwrap();
            info!("scd41_sensor: serial: {:#04x}", serial);

            sensor.start_periodic_measurement().unwrap();

            sensor
        }),
    )
    .await??;

    loop {
        info!("scd41_sensor: sleeping 5 secs...");
        tokio::time::sleep(std::time::Duration::from_secs(5)).await;

        info!("scd41_sensor: taking measurement...");
        if let Ok(measurement) = sensor.measurement() {
            info!("sht45_sensor: Broadcasting measurements...");
            tx.broadcast((
                measurement.co2,
                measurement.humidity,
                measurement.temperature,
            ))
            .await
            .ok();
        };
    }
}

async fn sht45_setup(
    driver: I2cProxy<'static, Mutex<I2cDriver<'_>>>,
) -> Result<
    Result<Sht4x<I2cProxy<'static, Mutex<I2cDriver<'static>>>, CustomDelay>, JoinError>,
    Elapsed,
> {
    tokio::time::timeout(
        std::time::Duration::from_secs(10),
        tokio::task::spawn_blocking(|| {
            info!("sht45_sensor: setup...");
            let mut delay = CustomDelay {};
            let mut sensor = Sht4x::new(driver);
            info!("sht45_sensor: verifying connection...");
            sensor.serial_number(&mut delay).ok();
            sensor
        }),
    )
    .await
}

async fn sht45_sensor(
    tx: Sender<Sht4xReading>,
    driver: I2cProxy<'static, std::sync::Mutex<I2cDriver<'_>>>,
) -> Result<()> {
    let mut delay = CustomDelay {};
    let (mut sensor, mut delay) = tokio::time::timeout(
        std::time::Duration::from_secs(10),
        tokio::task::spawn_blocking(|| {
            info!("sht45_sensor: setup...");
            let mut sensor = Sht4x::new(driver);
            info!("sht45_sensor: verifying connection...");
            sensor.serial_number(&mut delay).ok();
            (sensor, delay)
        }),
    )
    .await??;

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
    pub async fn configure(&mut self) -> Result<(), EspError> {
        info!("Setting Wi-Fi credentials...");
        self.wifi
            .set_configuration(&Configuration::Client(ClientConfiguration {
                ssid: WIFI_SSID.into(),
                password: WIFI_PASS.into(),
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

async fn echo_server() -> anyhow::Result<()> {
    let addr = format!("0.0.0.0:{TCP_LISTENING_PORT}");

    info!("Binding to {addr}...");
    let listener = TcpListener::bind(&addr).await?;

    loop {
        info!("Waiting for new connection on socket: {listener:?}");
        let (socket, _) = listener.accept().await?;

        info!("Spawning handle for: {socket:?}...");
        tokio::spawn(async move {
            info!("Spawned handler!");
            let peer = socket.peer_addr();
            if let Err(e) = serve_client(socket).await {
                info!("Got error handling {peer:?}: {e:?}");
            }
        });
    }
}

async fn serve_client(mut stream: TcpStream) -> anyhow::Result<()> {
    info!("Handling {stream:?}...");

    let mut buf = [0u8; 512];
    loop {
        info!("About to read...");
        let n = stream.read(&mut buf).await?;
        info!("Read {n} bytes...");

        if n == 0 {
            break;
        }

        stream.write_all(&buf[0..n]).await?;
        info!("Wrote {n} bytes back...");
    }

    Ok(())
}

async fn sensor(tx: Sender<String>) -> Result<()> {
    for i in 0..100 {
        info!("Broadcasting from {:?}: {}", core(), i);
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
        info!("Received in {:?}: {}", core(), data);
        tokio::time::sleep(std::time::Duration::from_secs(5)).await;
    }

    Ok(())
}
