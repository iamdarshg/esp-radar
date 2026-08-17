//! Node-role resolution for the shared radar_rx binary (spec §4).
//!
//! Both receivers run the same firmware; the only difference is which physical
//! board it is. The role is resolved in this order:
//!
//!   1. `RadarConfig::node_role` already set in NVS (provisioned) — use it.
//!   2. Otherwise infer from hardware: only RADAR-RX2 (the ESP32-CAM) has
//!      PSRAM, so `esp_psram_is_initialized()` distinguishes it from RADAR-RX1
//!      (the DevKit, no PSRAM). Persist the inference so later boots are
//!      deterministic and the calibration host can read it back.
//!   3. Last resort: RADAR-RX1.
//!
//! This inference assumes the fixed physical layout of the head, one rigid
//! three-board assembly: the only PSRAM-equipped board among the three is the
//! ESP32-CAM (RADAR-RX2). Board side is not load-bearing — RX1 is the DevKit
//! (rotated 180° on the left of the middle) and RX2 the CAM (rotated 180° on
//! the right), but each board resolves its own role at boot. The boards are
//! permanently mounted, so the role is effectively immutable after the first
//! boot.
//!
//! Wired data plane (`main.rs`): the GPIO matrix routes each board's UART1 to
//! the pins on the edge facing the middle, so the crossed links are short,
//! parallel jumpers across the board gaps:
//!
//! ```text
//!   RX1 (LEFT DevKit, 180°): UART1 GPIO17 TX / GPIO16 RX  ←→  middle GPIO16 / GPIO17
//!   RX2 (CAM, 180°):         UART1 IO15  TX / IO13  RX    ←→  middle GPIO22 / GPIO23
//! ```
//!
//! Power: the middle's 3V3 feeds both neighbours; all three boards share a
//! common GND; the CAM's GPIO5 (D5) ties to GND as the board's power-return
//! sink (driven output-low in `main.rs`).

use radar_protocol::node;
use radar_storage::nvs::Nvs;
use radar_storage::{RadarConfig, RxLink};

/// Turn a protocol node id (`node::RX1` / `node::RX2`) into the NVS namespace
/// key, or `None` if it is not a receiver role.
pub fn rx_link_for(node_id: u8) -> Option<RxLink> {
    match node_id {
        n if n == node::RX1 => Some(RxLink::Rx1),
        n if n == node::RX2 => Some(RxLink::Rx2),
        _ => None,
    }
}

/// Resolve the receiver role, persisting any inference to NVS. Returns the
/// protocol node id (`node::RX1` or `node::RX2`).
pub fn resolve_role(nvs: &Nvs, config: &mut RadarConfig) -> u8 {
    if config.node_role == node::RX1 || config.node_role == node::RX2 {
        log::info!("node role from NVS: {}", role_name(config.node_role));
        return config.node_role;
    }

    // Hardware inference. `esp_psram_is_initialized` is `bool` in C; bindgen
    // maps it to a Rust bool return value.
    let inferred = if unsafe { esp_idf_sys::esp_psram_is_initialized() } {
        node::RX2
    } else {
        node::RX1
    };
    log::info!(
        "node role inferred from hardware: {}",
        role_name(inferred)
    );
    config.node_role = inferred;
    if let Err(e) = nvs.store_config(config) {
        log::warn!("could not persist inferred node role: {e}");
    }
    inferred
}

/// Human-readable receiver role for logs.
pub fn role_name(node_id: u8) -> &'static str {
    match node_id {
        n if n == node::RX1 => "RADAR-RX1",
        n if n == node::RX2 => "RADAR-RX2",
        _ => "unknown",
    }
}
