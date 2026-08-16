//! Shared constants, configuration and the child-summary wire format for the
//! integration test (Task 6).

use std::collections::HashMap;

/// Measurement port, TX -> RX (both receivers listen). Reuses the crate
/// constant so the tool and firmware agree by construction.
pub const MEASURE_PORT: u16 = radar_transport::MEASURE_PORT; // 4444
/// Report port, RX -> TX (TX listens). Reuses the crate constant.
pub const REPORT_PORT: u16 = radar_transport::REPORT_PORT; // 4445

/// ---------------------------------------------------------------------------
/// Transport choice: **two-unicast** (the brief's option 2).
///
/// The preferred multicast route (`239.0.0.1:4444`, both RX bound to
/// `0.0.0.0:4444` with `SO_REUSEADDR` and joined to the loopback group) was
/// probed empirically on this Windows 11 host and proved unreliable at the
/// real 200 Hz rate: with two receivers joined, loopback multicast delivery
/// was inconsistent between runs — sometimes both received every datagram,
/// sometimes one receiver received all and the other received none (a known
/// Windows behaviour: multicast packets bound to a `SO_REUSEADDR` port are
/// delivered to an arbitrary single socket, not to all joiners). The
/// two-unicast fallback sends the same frame bytes (same header, same CRC,
/// same seq) to each receiver's own loopback address, which exercises
/// identical framing/CRC/seq semantics and is deterministic on loopback.
/// ---------------------------------------------------------------------------
/// RX1's measurement bind (TX unicasts a copy of every DataFrame here).
pub const RX1_MEASURE_ADDR: &str = "127.0.0.2";
/// RX2's measurement bind.
pub const RX2_MEASURE_ADDR: &str = "127.0.0.3";
/// TX listens for RATE-2/3 (feature reports + snapshots) on the loopback
/// wildcard. Both RX unicast to this address.
pub const TX_REPORT_ADDR: &str = "127.0.0.1";

/// Default measurement rate (Hz) — matches the firmware default.
pub const DEFAULT_RATE_HZ: u64 = 200;
/// Default run window (seconds).
pub const DEFAULT_DURATION_SECS: u64 = 8;
/// Default PRNG seed for the synthetic CSI.
pub const DEFAULT_SEED: u64 = 42;
/// Report cadence: one FeatureReport per this many TX frames (matches the
/// firmware default `report_every = 20`).
pub const REPORT_EVERY: u32 = 20;
/// Pairer tolerance in TX seq units (≈ half the report window).
pub const PAIR_TOLERANCE: u32 = 10;
/// Snapshot cadence: one CsiSnapshot per this many TX frames → ~2 Hz at 200 Hz.
pub const SNAPSHOT_EVERY: u32 = 100;

/// A radar node role.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Role {
    Tx,
    Rx1,
    Rx2,
}

impl Role {
    pub fn as_str(&self) -> &'static str {
        match self {
            Role::Tx => "tx",
            Role::Rx1 => "rx1",
            Role::Rx2 => "rx2",
        }
    }

    pub fn node_id(&self) -> u8 {
        match self {
            Role::Tx => radar_protocol::node::TX,
            Role::Rx1 => radar_protocol::node::RX1,
            Role::Rx2 => radar_protocol::node::RX2,
        }
    }

    pub fn parse(s: &str) -> Option<Role> {
        match s {
            "tx" => Some(Role::Tx),
            "rx1" => Some(Role::Rx1),
            "rx2" => Some(Role::Rx2),
            _ => None,
        }
    }
}

/// Runtime configuration shared by all roles.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub duration_secs: u64,
    pub rate_hz: u64,
    pub seed: u64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            duration_secs: DEFAULT_DURATION_SECS,
            rate_hz: DEFAULT_RATE_HZ,
            seed: DEFAULT_SEED,
        }
    }
}

/// Microseconds elapsed since `start`, offset so timestamps stay positive.
pub fn t_us_since(start: std::time::Instant) -> u64 {
    1_000_000 + start.elapsed().as_micros() as u64
}

/// Parse a child's `SUMMARY|key=value|...` line into a map. Returns `None`
/// for lines that are not summary lines.
pub fn parse_summary(line: &str) -> Option<HashMap<String, String>> {
    let line = line.trim();
    if !line.starts_with("SUMMARY|") {
        return None;
    }
    let mut map = HashMap::new();
    for field in line["SUMMARY|".len()..].split('|') {
        if field.is_empty() {
            continue;
        }
        if let Some((k, v)) = field.split_once('=') {
            map.insert(k.trim().to_string(), v.trim().to_string());
        }
    }
    Some(map)
}

/// Get a numeric field from a summary map, defaulting to 0.
pub fn get_u64(map: &HashMap<String, String>, key: &str) -> u64 {
    map.get(key).and_then(|s| s.parse().ok()).unwrap_or(0)
}

/// Get a string field from a summary map.
pub fn get_str<'a>(map: &'a HashMap<String, String>, key: &str) -> &'a str {
    map.get(key).map(|s| s.as_str()).unwrap_or("")
}
