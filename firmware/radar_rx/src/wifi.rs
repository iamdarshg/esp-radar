//! RADAR-RX station bring-up (spec §10).
//!
//! The RX nodes are STA clients of the RADAR-TX AP ("ESP32-RADAR"). They
//! associate on the radar's configured channel so CSI is captured from the
//! broadcast measurement traffic; the channel hint also makes association
//! quick and guarantees both receivers observe the same 20 MHz channel.
//!
//! On success this returns the running `EspWifi` and the **AP BSSID**, which
//! the DSP loop uses to filter CSI: the Wi-Fi CSI callback fires for *every*
//! received 802.11 frame (beacons, foreign stations, ...), so we keep only the
//! frames whose source MAC is the TX AP — the broadcast data frames.

use esp_idf_hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
// esp-idf-hal 0.46.2 does not re-export `EspError`; `esp_idf_svc::sys` re-exports
// the esp-idf-sys bindings where it lives.
use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, EspWifi};
use heapless::String as HeaplessString;

use radar_transport::Ipv4Addr;

/// SSID of the radar AP (matches `radar_tx::wifi::SSID`).
pub const SSID: &str = "ESP32-RADAR";
/// Per-association wait before retrying (ms).
const POLL_STEP_MS: u64 = 100;
/// Associations attempted before logging and starting over.
const POLL_STEPS: u32 = 50; // 5 s per attempt

/// Associate with the RADAR-TX AP and return the running STA plus the AP's
/// BSSID (the MAC to filter CSI by). Retries forever — RADAR-TX boots first,
/// but a cold-start RX must not fall over if it associates before the AP is up.
pub fn connect_sta(
    modem: Modem<'static>,
    sys_loop: EspSystemEventLoop,
    channel: u8,
) -> Result<(EspWifi<'static>, [u8; 6]), EspError> {
    let mut wifi = EspWifi::new(modem, sys_loop, None)?;

    let mut client = ClientConfiguration::default();
    client.ssid = HeaplessString::try_from(SSID).unwrap();
    client.auth_method = AuthMethod::None;
    client.channel = Some(channel); // no scan: join the radar channel directly
    wifi.set_configuration(&Configuration::Client(client))?;
    wifi.start()?;

    loop {
        // `connect` is non-blocking; poll the association state.
        if wifi.is_connected()? {
            break;
        }
        wifi.connect()?;
        let mut connected = false;
        for _ in 0..POLL_STEPS {
            std::thread::sleep(std::time::Duration::from_millis(POLL_STEP_MS));
            if wifi.is_connected()? {
                connected = true;
                break;
            }
        }
        if connected {
            break;
        }
        log::warn!("STA: no association with '{SSID}' yet; retrying");
    }

    // `get_ap_info` needs an active association, which we have now.
    let bssid = wifi.get_ap_info()?.bssid;
    log::info!(
        "STA associated: AP '{}' at {} (BSSID {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})",
        SSID,
        Ipv4Addr::AP,
        bssid[0],
        bssid[1],
        bssid[2],
        bssid[3],
        bssid[4],
        bssid[5]
    );
    Ok((wifi, bssid))
}
