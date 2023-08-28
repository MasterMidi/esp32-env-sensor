use async_broadcast::{broadcast, Receiver, Sender};

use esp_idf_sys::{self as _, esp, esp_app_desc}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;
use tokio::{self};

use anyhow::{Ok, Result};

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


    info!("Starting async run loop");
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?
        .block_on(async_main())?;

    info!("Shutting down");
    Ok(())
}

async fn async_main() -> Result<()> {
    let (mut s1, r1) = broadcast(1);

    s1.set_overflow(true);

    let t1 = tokio::spawn(sensor(s1));

    let t2 = tokio::spawn(sensor2(r1));

    let _ = t1.await?;
    let _ = t2.await?;

    Ok(())
}

async fn sensor(tx: Sender<String>) -> Result<()> {
    for i in 0..10 {
        info!("Broadcasting: {}", i);
        tx.broadcast(format!("reading of {i}")).await?;
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
                    info!("Channel closed");
                    break;
                }
                async_broadcast::RecvError::Overflowed(num) => {
                    info!("Channel overflow, missed {num} message(s)");
                    continue;
                }
            },
        };
        info!("Received: {}", data);
        tokio::time::sleep(std::time::Duration::from_secs(3)).await;
    }

    Ok(())
}
