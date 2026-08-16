//! AP bring-up for RADAR-TX (spec §10).
//!
//! RADAR-TX is the head's only AP; both RX stations and any dashboard
//! (phone/tablet) connect directly to it — no router, no internet required.
//! The radar keeps sensing whether or not anything is connected, so the AP is
//! brought up unconditionally at boot.

use esp_idf_hal::modem::Modem;
// esp-idf-hal 0.46.2 does not re-export `EspError`; `esp_idf_svc::sys` re-exports
// the esp-idf-sys bindings where it lives (`esp_idf_sys::EspError`).
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::sys::EspError;
use esp_idf_svc::wifi::{AccessPointConfiguration, AuthMethod, Configuration, EspWifi, Protocol};
use heapless::String as HeaplessString;

use radar_storage::RadarConfig;

/// SSID of the radar AP. Fixed — the phone/tablet dashboard app connects to
/// this name.
pub const SSID: &str = "ESP32-RADAR";

/// Bring up the AP on the configured channel and return the running `EspWifi`.
///
/// `None` for the NVS partition disables the WiFi driver's *own* NVS use — the
/// radar config lives in `radar_storage`'s namespace, and the two must not
/// both call `nvs_flash_init` in incompatible ways.
pub fn bring_up_ap(
    modem: Modem<'static>,
    sys_loop: EspSystemEventLoop,
    config: &RadarConfig,
) -> Result<EspWifi<'static>, EspError> {
    let mut wifi = EspWifi::new(modem, sys_loop, None)?;

    let ap = AccessPointConfiguration {
        ssid: HeaplessString::<32>::try_from(SSID).unwrap(),
        ssid_hidden: false,
        channel: config.channel,
        secondary_channel: None,
        // BGN on HT20 keeps all 56 subcarriers of a single 20 MHz channel.
        protocols: Protocol::P802D11BGN.into(),
        auth_method: AuthMethod::None,
        password: HeaplessString::<64>::new(),
        max_connections: 4,
    };
    wifi.set_configuration(&Configuration::AccessPoint(ap))?;
    wifi.start()?;
    log::info!("AP '{}' up on channel {}", SSID, config.channel);
    Ok(wifi)
}
