//! RADAR-RX application entry point — one binary, two boards (spec §4, §10).
//!
//! The same firmware runs on RADAR-RX1 (ESP32 DevKitC, no PSRAM) and
//! RADAR-RX2 (ESP32-CAM, PSRAM). The only difference is the node role, which
//! is resolved (and persisted) at boot:
//!
//!   1. `RadarConfig::node_role` from NVS, if provisioned;
//!   2. otherwise hardware inference — only the ESP32-CAM has PSRAM, so
//!      `esp_psram_is_initialized()` tells the two apart (§4, §17);
//!   3. last resort RADAR-RX1.
//!
//! Boot sequence:
//!   1. Link patches + logger.
//!   2. NVS: config (defaults on fresh flash) + role resolution.
//!   3. Associate as a STA with the "ESP32-RADAR" AP on the radar channel,
//!      learning the AP BSSID for CSI MAC filtering.
//!   4. Start CSI capture into a leaked lock-free ring.
//!   5. Spawn the measurement/DSP/calibration loop (`radar::run`) and sleep
//!      forever — the leaked `EspWifi` and `CsiRing` must outlive `main`.

use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::log::EspLogger;

use radar_csi::wifi::{start_csi, CsiConfig};
use radar_csi::CsiRing;
use radar_protocol::node;
use radar_storage::nvs::Nvs;
use radar_storage::RadarConfig;

mod link;
mod radar;
mod wifi;
mod wired;

fn main() -> anyhow::Result<()> {
    // esp-idf-sys's `binstart` feature needs link_patches() before anything
    // else, or some runtime symbols never link.
    esp_idf_sys::link_patches();
    EspLogger::initialize_default();

    // -- NVS config + role --------------------------------------------------
    let nvs = Nvs::take(true).map_err(|e| anyhow::anyhow!("nvs init: {e}"))?;
    let mut config = match nvs.load_config() {
        Ok(c) => c,
        Err(_) => {
            log::info!("no config in NVS; writing defaults");
            let c = RadarConfig::default();
            let _ = nvs.store_config(&c);
            c
        }
    };
    let node_id = link::resolve_role(&nvs, &mut config);
    let link = link::rx_link_for(node_id)
        .ok_or_else(|| anyhow::anyhow!("node role {node_id} is not a receiver"))?;
    log::info!(
        "config: channel={} rate={}Hz report_every={}",
        config.channel,
        config.tx_rate_hz,
        config.report_every
    );

    // -- radio --------------------------------------------------------------
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let (wifi, ap_bssid) = wifi::connect_sta(peripherals.modem, sys_loop, config.channel)?;
    // `EspWifi`'s Drop disconnects the STA; leak it so the association stays up.
    let _wifi = Box::leak(Box::new(wifi));

    // -- CSI ----------------------------------------------------------------
    // The ring must be `'static`: the WiFi CSI callback (running in the WiFi
    // task context) writes into it, and the radar task reads from it. 128 slots
    // × 256 B ≈ 32 KB of heap — plenty at the 200 Hz rate.
    let ring: &'static CsiRing = Box::leak(Box::new(CsiRing::new(128)));
    start_csi(ring, &CsiConfig::default()).map_err(|e| anyhow::anyhow!("csi start: {e}"))?;

    // -- wired data plane -----------------------------------------------------
    // Reports/CAL/snapshots go over UART, not WiFi, so this board stops
    // transmitting on the 2.4 GHz sensing band. The GPIO matrix routes each
    // board's UART1 to the pins on the edge facing the middle, so the links are
    // short, parallel jumpers across the gaps. The role match gives each board
    // its crossed-pair pins (the other arm's pins are never moved):
    //   RX1 (LEFT DevKit, 180°-rotated): UART1 GPIO17 TX / GPIO16 RX  ←→  middle GPIO16 / GPIO17
    //   RX2 (CAM, 180°-rotated):          UART1 IO15 TX / IO13 RX     ←→  middle GPIO22 / GPIO23
    // (IO15 is the TDO strap — its level only selects ROM console output, so
    // driving it as UART TX is boot-safe.)
    let wired = match node_id {
        n if n == node::RX1 => {
            wired::WiredLink::open(peripherals.uart1, peripherals.pins.gpio17, peripherals.pins.gpio16)?
        }
        n if n == node::RX2 => {
            // GPIO5 (D5) is the CAM's power-return path: it is tied to the
            // shared GND rail and sinks the board's supply return. Drive it
            // output-low so the firmware actively holds that line instead of
            // leaving the SDIO strap pull-up to float it high.
            let mut d5 = PinDriver::output(peripherals.pins.gpio5)?;
            d5.set_low()?;
            wired::WiredLink::open(peripherals.uart1, peripherals.pins.gpio15, peripherals.pins.gpio13)?
        }
        _ => unreachable!("rx_link_for guaranteed a receiver role"),
    };

    // -- measurement / DSP / calibration loop --------------------------------
    std::thread::Builder::new()
        .stack_size(16384)
        .name("radar".into())
        .spawn(move || {
            radar::run(radar::RunParams {
                config,
                node_id,
                link,
                wired,
                ring,
                nvs,
                ap_bssid,
            });
        })?;

    log::info!(
        "RADAR-RX{} ready — CSI active on ch{} (AP bssid {ap_bssid:02x?})",
        node_id,
        config.channel
    );

    // The leaked wifi + ring live as long as this task does; sleep forever.
    loop {
        std::thread::sleep(std::time::Duration::from_secs(60));
    }
}
