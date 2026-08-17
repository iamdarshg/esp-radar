//! RADAR-TX application entry point (spec §2, §10, §13, §17).
//!
//! Boot sequence:
//!   1. Link patches + logger; commit any OTA-updated image (`mark_app_valid`).
//!   2. Load the radar config from NVS (fresh flash → defaults).
//!   3. Bring up the "ESP32-RADAR" AP on the configured channel (§10).
//!   4. Apply the commissioned TX power, or a conservative default until the
//!      boot auto-CAL2 runs (§5).
//!   5. Start the dashboard (HTTP + WebSocket) and register `/cal` + `/ota`.
//!   6. Spawn the traffic, fusion/controller, and coprocessor tasks.
//!
//! Nothing here ever returns: `main` sleeps forever, keeping the leaked
//! `EspWifi` and the `Dashboard` alive so the AP and HTTP server outlive the
//! bootstrap.

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::Duration;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::log::EspLogger;

use radar_storage::nvs::Nvs;
use radar_storage::RadarConfig;
use radar_web::server::Dashboard;
use radar_web::telemetry::StatusSnapshot;

use crate::calibrate::CalCommand;

mod calibrate;
// The RP2350 coprocessor link (UART2/GPIO16-17) is suspended: those pins are
// now the wired data-plane link to RADAR-RX2/CAM. The module stays compiled
// for future use once the coprocessor gets its own pins.
#[allow(dead_code)]
mod cp;
mod fusion;
mod httpd;
mod traffic;
mod wifi;
mod wired;

fn main() -> anyhow::Result<()> {
    // esp-idf-sys's `binstart` feature needs link_patches() before anything
    // else, or some runtime symbols never link.
    esp_idf_sys::link_patches();
    EspLogger::initialize_default();

    // An OTA-updated image must be committed early: if we crash later the
    // bootloader would roll back to the previous slot on the next boot.
    if let Err(rc) = radar_ota::ota::mark_app_valid() {
        log::warn!("mark_app_valid failed (0x{rc:08x}); continuing");
    }

    // -- NVS config ---------------------------------------------------------
    let nvs = Nvs::take(true).map_err(|e| anyhow::anyhow!("nvs init: {e}"))?;
    let config = match nvs.load_config() {
        Ok(c) => c,
        Err(_) => {
            log::info!("no config in NVS; writing defaults (tx_power_db=0 → auto-commission)");
            let c = RadarConfig::default();
            let _ = nvs.store_config(&c);
            c
        }
    };
    log::info!(
        "config: channel={} rate={}Hz report_every={} pair_tol={} tx_power_db={}",
        config.channel,
        config.tx_rate_hz,
        config.report_every,
        config.pair_tolerance,
        config.tx_power_db
    );

    // -- radio -------------------------------------------------------------
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let wifi = wifi::bring_up_ap(peripherals.modem, sys_loop, &config)?;
    // `EspWifi`'s Drop stops the radio; leak it so the AP stays up forever.
    let _wifi = Box::leak(Box::new(wifi));

    // -- shared state --------------------------------------------------------
    let tx_power: Arc<AtomicU8> = Arc::new(AtomicU8::new(0));
    let cal_active: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));
    let status: Arc<Mutex<StatusSnapshot>> = Arc::new(Mutex::new(StatusSnapshot {
        channel: config.channel,
        tx_rate_hz: config.tx_rate_hz,
        ..Default::default()
    }));

    // Apply the commissioned TX power, or a conservative default until the boot
    // auto-CAL2 picks the right value (§5). `esp_wifi_set_max_tx_power` needs
    // the WiFi started, which it now is.
    let boot_power = if config.tx_power_db != 0 {
        config.tx_power_db
    } else {
        calibrate::DEFAULT_TX_POWER_DBM
    };
    tx_power.store(boot_power, Ordering::Relaxed);
    calibrate::set_tx_power(boot_power as i16);
    log::info!(
        "boot TX power: {boot_power} dBm{}",
        if config.tx_power_db == 0 { " (auto-commission pending)" } else { "" }
    );

    // -- dashboard + endpoints ------------------------------------------------
    let mut dashboard = Dashboard::start(status.clone())?;
    let broadcaster = dashboard.broadcaster();
    let (cal_tx, cal_rx) = mpsc::channel::<CalCommand>();
    httpd::register(&mut dashboard, cal_tx)?;

    // -- wired data plane ------------------------------------------------------
    // The two RX boards report over crossed UART links instead of WiFi, so
    // they never transmit on the 2.4 GHz sensing band:
    //   UART1 GPIO18 TX / GPIO19 RX  ←→  RADAR-RX1 (right DevKit) GPIO19/18
    //   UART2 GPIO17 TX / GPIO16 RX  ←→  RADAR-RX2 (CAM)         IO13 / IO14
    // (UART2/GPIO16-17 were the RP2350 coprocessor pins; that task is parked
    // in `cp.rs` for future use.)
    let link1 = wired::WiredLink::open(peripherals.uart1, peripherals.pins.gpio18, peripherals.pins.gpio19)?;
    let link2 = wired::WiredLink::open(peripherals.uart2, peripherals.pins.gpio17, peripherals.pins.gpio16)?;

    // -- tasks ----------------------------------------------------------------
    // The traffic closure is `move`, so it would swallow the `tx_power` /
    // `cal_active` bindings even though it only clones them; pre-clone so the
    // fusion task below still gets the originals.
    let traffic_tx_power = tx_power.clone();
    let traffic_cal_active = cal_active.clone();
    std::thread::Builder::new()
        .stack_size(4096)
        .name("traffic".into())
        .spawn(move || traffic::run(config.tx_rate_hz, traffic_tx_power, traffic_cal_active))?;

    std::thread::Builder::new()
        .stack_size(16384)
        .name("fusion".into())
        .spawn(move || fusion::run(fusion::RunParams {
            config,
            status,
            broadcaster,
            tx_power,
            cal_active,
            cal_rx,
            nvs,
            links: [link1, link2],
        }))?;

    log::info!(
        "RADAR-TX ready — AP '{}' ch{} → http://192.168.4.1",
        wifi::SSID,
        config.channel
    );

    // The dashboard + leaked wifi live as long as this task does; sleep forever.
    loop {
        std::thread::sleep(Duration::from_secs(60));
    }
}
