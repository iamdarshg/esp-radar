//! Persistent configuration for the compact radar head.
//!
//! RADAR-TX / RADAR-RX keep their one-time, long-lived settings in NVS (ESP32
//! non-volatile storage) rather than in code. The boards are permanently
//! mounted and never repositioned, so the antenna offsets, channel, TX power
//! and the calibration artifacts are *device facts*, not compile-time defaults.
//!
//! Layout:
//!   * [`RadarConfig`] — the system config blob (spec §3 offsets, §6 radio).
//!   * [`RxLink`]      — which physical receiver a per-link artifact belongs to.
//!   * `#[cfg(feature = "device")] pub mod nvs` — the raw ESP32 NVS binding.
//!
//! SD-card rolling logs and triggered CSI bursts are deferred by design (the
//! boards boot with a fresh flash and the web UI covers diagnostics); this
//! crate is deliberately NVS-only for now.
//!
//! This crate is pure Rust and std-enabled like the other pure crates — device
//! builds target `xtensa-esp32-espidf`, host builds run the unit tests.

use radar_protocol::node;

/// Schema version of the on-disk [`RadarConfig`] blob. Bump when fields change;
/// `from_bytes` falls back to defaults for unknown or missing bytes.
pub const CONFIG_VERSION: u32 = 1;

/// One of the two fixed receiver links. Calibration artifacts (baselines,
/// thresholds) are per-link, as are the spec §3 antenna offsets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum RxLink {
    Rx1 = 0,
    Rx2 = 1,
}

impl RxLink {
    /// Node id in the `radar_protocol` sense (`node::RX1` / `node::RX2`).
    pub fn node_id(self) -> u8 {
        match self {
            RxLink::Rx1 => node::RX1,
            RxLink::Rx2 => node::RX2,
        }
    }
}

/// System-wide configuration, serialized to a single NVS blob.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadarConfig {
    /// [`CONFIG_VERSION`] — lets readers detect a stale layout.
    pub version: u32,
    /// 2.4 GHz Wi-Fi channel the AP operates on (spec §6).
    pub channel: u8,
    /// Measurement frames per second emitted by RADAR-TX (spec §6).
    pub tx_rate_hz: u16,
    /// RX feature reports are sent every N frames.
    pub report_every: u16,
    /// Max frame-seq gap tolerated when pairing RX1/RX2 observations.
    pub pair_tolerance: u16,
    /// TX power in dBm. `0` means "not commissioned → auto" (spec §5).
    pub tx_power_db: u8,
    /// Which role this device plays (`radar_protocol::node::*`). Shared by the
    /// same radar_rx binary on both receivers, so it must come from NVS.
    pub node_role: u8,
    /// Distance TX antenna → RX1 antenna, millimetres (spec §3).
    pub antenna_offset_txrx1_mm: u16,
    /// Distance TX antenna → RX2 antenna, millimetres (spec §3).
    pub antenna_offset_txrx2_mm: u16,
    /// Extra subcarrier shift applied at CSI capture (per-link trim).
    pub csi_shift: u8,
    /// 1 = QEMU RF-sim mode: skip WiFi bring-up and feed the CSI ring from the
    /// `simdata` flash partition (quantification runs). 0 = normal operation.
    pub sim_mode: u8,
    /// Spare bytes — keep for ABI stability when extending.
    pub reserved: [u8; 2],
}

impl Default for RadarConfig {
    fn default() -> Self {
        Self {
            version: CONFIG_VERSION,
            channel: 6,
            tx_rate_hz: 200,
            report_every: 20,
            pair_tolerance: 10,
            tx_power_db: 0, // not commissioned → auto
            node_role: 0,   // unset
            antenna_offset_txrx1_mm: 0,
            antenna_offset_txrx2_mm: 0,
            csi_shift: 0,
            sim_mode: 0,
            reserved: [0; 2],
        }
    }
}

impl RadarConfig {
    /// 4 (version) + 1 + 2 + 2 + 2 + 1 + 1 + 2 + 2 + 1 + 1 (sim_mode)
    /// + 2 (reserved) = 21 bytes — byte-compatible with the pre-sim layout.
    pub const SERIALIZED_LEN: usize = 4 + 1 + 2 + 2 + 2 + 1 + 1 + 2 + 2 + 1 + 3;

    /// Little-endian encoding matching the field layout above.
    pub fn to_bytes(&self) -> [u8; Self::SERIALIZED_LEN] {
        let mut out = [0u8; Self::SERIALIZED_LEN];
        let mut o = 0;
        out[o..o + 4].copy_from_slice(&self.version.to_le_bytes());
        o += 4;
        out[o] = self.channel;
        o += 1;
        out[o..o + 2].copy_from_slice(&self.tx_rate_hz.to_le_bytes());
        o += 2;
        out[o..o + 2].copy_from_slice(&self.report_every.to_le_bytes());
        o += 2;
        out[o..o + 2].copy_from_slice(&self.pair_tolerance.to_le_bytes());
        o += 2;
        out[o] = self.tx_power_db;
        o += 1;
        out[o] = self.node_role;
        o += 1;
        out[o..o + 2].copy_from_slice(&self.antenna_offset_txrx1_mm.to_le_bytes());
        o += 2;
        out[o..o + 2].copy_from_slice(&self.antenna_offset_txrx2_mm.to_le_bytes());
        o += 2;
        out[o] = self.csi_shift;
        o += 1;
        out[o] = self.sim_mode;
        o += 1;
        out[o..o + 2].copy_from_slice(&self.reserved);
        out
    }

    /// Decode a blob. Returns defaults when the input is too short (e.g. an
    /// older, shorter layout) — callers decide whether that's acceptable.
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut c = Self::default();
        if b.len() < Self::SERIALIZED_LEN {
            return c;
        }
        let mut o = 0;
        let read_u32 = |o: &mut usize| -> u32 {
            let v = u32::from_le_bytes([b[*o], b[*o + 1], b[*o + 2], b[*o + 3]]);
            *o += 4;
            v
        };
        let read_u16 = |o: &mut usize| -> u16 {
            let v = u16::from_le_bytes([b[*o], b[*o + 1]]);
            *o += 2;
            v
        };
        c.version = read_u32(&mut o);
        c.channel = b[o];
        o += 1;
        c.tx_rate_hz = read_u16(&mut o);
        c.report_every = read_u16(&mut o);
        c.pair_tolerance = read_u16(&mut o);
        c.tx_power_db = b[o];
        o += 1;
        c.node_role = b[o];
        o += 1;
        c.antenna_offset_txrx1_mm = read_u16(&mut o);
        c.antenna_offset_txrx2_mm = read_u16(&mut o);
        c.csi_shift = b[o];
        o += 1;
        c.sim_mode = b[o];
        o += 1;
        c.reserved.copy_from_slice(&b[o..o + 2]);
        c
    }
}

#[cfg(feature = "device")]
pub mod nvs;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_defaults_are_sane() {
        let c = RadarConfig::default();
        assert_eq!(c.version, CONFIG_VERSION);
        assert_eq!(c.channel, 6);
        assert_eq!(c.tx_rate_hz, 200);
        assert_eq!(c.tx_power_db, 0); // not commissioned
        assert_eq!(c.node_role, 0); // unset
        assert_eq!(c.antenna_offset_txrx1_mm, 0);
    }

    #[test]
    fn config_roundtrip_default() {
        let c = RadarConfig::default();
        let bytes = c.to_bytes();
        assert_eq!(bytes.len(), RadarConfig::SERIALIZED_LEN);
        assert_eq!(RadarConfig::from_bytes(&bytes), c);
    }

    #[test]
    fn config_roundtrip_nondefault() {
        let c = RadarConfig {
            version: 1,
            channel: 11,
            tx_rate_hz: 500,
            report_every: 50,
            pair_tolerance: 32,
            tx_power_db: 20,
            node_role: node::TX,
            antenna_offset_txrx1_mm: 55,
            antenna_offset_txrx2_mm: 42,
            csi_shift: 1,
            sim_mode: 1,
            reserved: [7, 8],
        };
        let bytes = c.to_bytes();
        assert_eq!(RadarConfig::from_bytes(&bytes), c);
    }

    #[test]
    fn config_byte_layout_is_backward_compatible() {
        // The pre-sim blob (21 bytes, sim_mode byte = the old reserved[0] = 0)
        // must decode to sim_mode 0 — i.e. normal operation — with the same size.
        let legacy = [
            0x01, 0x00, 0x00, 0x00, // version
            0x06, // channel
            0xc8, 0x00, // rate 200
            0x14, 0x00, // report_every 20
            0x0a, 0x00, // pair_tol 10
            0x00, // tx_power
            0x03, // node_role RX2
            0x00, 0x00, 0x00, 0x00, // ant offsets
            0x00, // csi_shift
            0x00, 0x00, 0x00, // reserved[0..3] (old layout)
        ];
        assert_eq!(legacy.len(), 21);
        let c = RadarConfig::from_bytes(&legacy);
        assert_eq!(c.sim_mode, 0);
        assert_eq!(c.node_role, node::RX2);
        assert_eq!(c.channel, 6);
        // And a sim blob: set the byte that used to be reserved[0] (offset 18)
        // to 1 — decodes as sim_mode=1 with the same size.
        let mut sim = legacy;
        sim[18] = 1;
        let c2 = RadarConfig::from_bytes(&sim);
        assert_eq!(c2.sim_mode, 1);
        assert_eq!(c2.reserved, [0, 0]);
    }

    #[test]
    fn config_short_blob_falls_back_to_defaults() {
        // A truncated blob (old/short layout) must not panic — defaults out.
        let short = [0x01, 0x00, 0x00, 0x00]; // just a version field
        let c = RadarConfig::from_bytes(&short);
        assert_eq!(c.channel, 6); // default, not garbage
        assert_eq!(c.version, CONFIG_VERSION); // from default
    }

    #[test]
    fn rx_link_maps_to_protocol_nodes() {
        assert_eq!(RxLink::Rx1.node_id(), node::RX1);
        assert_eq!(RxLink::Rx2.node_id(), node::RX2);
    }
}
