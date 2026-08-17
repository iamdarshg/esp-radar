//! Wire format for the compact ESP32 radar head.
//!
//! All multi-byte fields are little-endian (ESP32 native). The header is
//! deliberately small and versioned so a future wired coprocessor (RP2350) or
//! a host tool can participate in the same network without a rewrite.
//!
//! Frame types:
//!   TX -> RX    `DataFrame`       one measurement packet; `seq` is the global
//!                                 packet counter that pairs CSI across links
//!   RX -> TX    `FeatureReport`   compact DSP/feature payload, high rate
//!   TX -> RX    `CalCmd`          calibration orchestration
//!   RX -> TX    `CalResp`         per-stage calibration result
//!   any         `Status`          diagnostics ping
//!   TX <-> RP   `cp`              versioned coprocessor message
//!
//! This crate is pure Rust (no ESP dependencies) so it is host-testable.

pub mod cp;
pub mod crc;

use crc::crc16;

/// Magic bytes "RDR1" — first four bytes of every radar frame.
pub const MAGIC: u32 = 0x5244_5231;
/// Wire version. Bumped to 2 for the phase-coherent radar path: `FeatureReport`
/// gained `phase_motion`/`doppler_hz` and the new `CSI_PHASE` frame appeared.
/// All boards are flashed from the same tree, so the version check is
/// consistency-only.
pub const VERSION: u8 = 2;

/// Node roles.
pub mod node {
    pub const TX: u8 = 0x01;
    pub const RX1: u8 = 0x02;
    pub const RX2: u8 = 0x03;
    pub const RP2350: u8 = 0x04;
}

/// Frame types.
pub mod frame_type {
    pub const DATA_FRAME: u8 = 0x10;
    pub const FEATURE_REPORT: u8 = 0x11;
    pub const CAL_CMD: u8 = 0x12;
    pub const CAL_RESP: u8 = 0x13;
    pub const STATUS: u8 = 0x14;
    /// Low-rate per-subcarrier CSI snapshot (waterfall + spectrogram source).
    pub const CSI_SNAPSHOT: u8 = 0x15;
    /// Full-rate RAW per-subcarrier phase telemetry (sim mode; the radar
    /// observable for the RF-sim error analysis).
    pub const CSI_PHASE: u8 = 0x16;
    pub const CP_MESSAGE: u8 = 0x20;
}

pub const MAX_PAYLOAD: usize = 512;

/// Common header. `payload_len` is the length of the payload that follows.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Header {
    pub magic: u32,
    pub version: u8,
    pub kind: u8, // frame_type::*
    pub src_node: u8,
    pub dst_node: u8,
    pub seq: u32,
    pub t_us: u64,
    pub payload_len: u16,
    pub crc16: u16,
}

impl Header {
    pub fn new(kind: u8, src: u8, dst: u8, seq: u32, t_us: u64, payload_len: u16) -> Self {
        Self {
            magic: MAGIC,
            version: VERSION,
            kind,
            src_node: src,
            dst_node: dst,
            seq,
            t_us,
            payload_len,
            crc16: 0,
        }
    }
}

/// TX -> RX measurement packet payload.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DataPayload {
    pub tx_power_db: u8,
    pub flags: u8,
}

pub mod data_flags {
    pub const CAL: u8 = 0x01; // this frame is part of a calibration stage
    pub const SYNC: u8 = 0x02; // this frame carries a resync marker
}

pub const MAX_PCA: usize = 8;

/// RX -> TX compact feature report.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeatureReport {
    pub seq: u32,        // last TX seq this report covers
    pub n_frames: u32,   // frames processed inside the window
    pub n_missing: u32,  // expected-but-unavailable frames in the window
    pub rssi: i16,       // dBm
    pub snr: i8,         // dB
    pub csi_quality: u8, // 0..100
    pub sat_score: u8,   // 0..100, higher = more clipped/saturated
    pub dyn_range: u8,   // 0..100
    pub flags: u8,
    pub amp_mean: f32, // mean amplitude across active subcarriers
    pub amp_std: f32,
    pub motion_energy: f32,    // energy in the human-motion band, windowed
    pub spectral_entropy: f32, // 0..1, higher = more broadband
    pub dominant_freq_hz: f32,
    pub phase_dispersion: f32, // circular std of sanitized phase
    /// Mean |normalized amp - baseline| / baseline std over active subcarriers.
    /// Static-presence indicator (spec §8): a persistent deviation from the
    /// empty-room baseline with low temporal motion energy ⇒ STATIC PRESENCE.
    pub baseline_dev: f32,
    pub pca_scores: [f32; MAX_PCA],
    /// RMS per-packet coherent phase increment (radians) over the window —
    /// the phase-coherent motion observable. A target displacement Δr shifts
    /// the received phase by Δφ = 4π·Δr/λ (round-trip; λ ≈ 12.5 cm at 2.4 GHz
    /// → 1 mm ≈ 0.10 rad). Amplitude-independent, unlike `motion_energy`.
    pub phase_motion: f32,
    /// Mean Doppler shift (Hz) over the window, from the per-frame phase rate
    /// after a scalar high-pass strips the per-board CFO (a DC bias in the
    /// phase rate — without it, a 100 Hz residual CFO aliases to the whole
    /// unambiguous velocity range). f_d = Δφ̄·fs/(2π); sign positive when the
    /// target approaches.
    pub doppler_hz: f32,
}

pub mod report_flags {
    pub const OVERFLOW: u8 = 0x01; // CSI ring buffer overflowed in this window
}

/// Number of subcarriers in the fixed head's HT20 CSI (matches `radar_dsp`).
pub const N_SUBCARRIERS: usize = 56;
/// Number of STFT frequency bins in a snapshot's motion-spectrum column.
pub const N_SPEC_BINS: usize = 64;

/// RX → TX per-subcarrier CSI snapshot (frame type `CSI_SNAPSHOT`).
///
/// The high-rate [`FeatureReport`] carries compact aggregates; this low-rate
/// frame (a few Hz) carries the actual per-subcarrier observation so RADAR-TX
/// can render the dashboard's LIVE CSI WATERFALL and PER-SUBCARRIER plots
/// (spec §6) with real data rather than a synthetic stand-in.
///
/// `#[repr(C, packed)]`, all fields little-endian. Total size is fixed:
/// 15 header bytes + 224 (IQ) + 56 (amp) + 64 (spec) = 359 bytes.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsiSnapshot {
    /// Last TX seq this snapshot covers (pairs with [`FeatureReport::seq`]).
    pub seq: u32,
    pub rssi: i16,
    pub snr: i8,
    pub csi_quality: u8,
    pub noise_floor: f32,
    pub flags: u8,
    /// Active subcarrier count (≤ [`N_SUBCARRIERS`]); trailing entries unused.
    pub n_sub: u8,
    pub reserved: u8,
    /// Sanitized complex CSI per subcarrier, interleaved I, Q (16-bit each).
    /// The dashboard derives RAW I / RAW Q / AMPLITUDE / SANITIZED PHASE.
    pub iq: [i16; N_SUBCARRIERS * 2],
    /// Baseline-referenced normalized amplitude per subcarrier (0..255).
    /// This is the waterfall's y-axis (time × subcarrier × amplitude).
    pub amp_norm: [u8; N_SUBCARRIERS],
    /// Current motion-spectrum column (STFT magnitude, 0..255), one bin per
    /// frequency. RADAR-TX accumulates these into the spectrogram telemetry.
    pub spec: [u8; N_SPEC_BINS],
}

pub mod snapshot_flags {
    pub const OVERFLOW: u8 = 0x01; // CSI ring overflowed since the last snapshot
}

impl Default for CsiSnapshot {
    fn default() -> Self {
        // Manual impl: `Default` is not derived for arrays longer than 32
        // elements, so the derive would fail for the iq/amp_norm/spec fields.
        Self {
            seq: 0,
            rssi: 0,
            snr: 0,
            csi_quality: 0,
            noise_floor: 0.0,
            flags: 0,
            n_sub: 0,
            reserved: 0,
            iq: [0; N_SUBCARRIERS * 2],
            amp_norm: [0; N_SUBCARRIERS],
            spec: [0; N_SPEC_BINS],
        }
    }
}

/// RX → TX RAW per-subcarrier phase telemetry (frame type `CSI_PHASE`).
///
/// Unlike [`CsiSnapshot`], whose IQ is reconstructed from the *sanitized*
/// phase, this frame carries the raw measured phase — the linear-across-
/// subcarrier slope, which carries the dominant-path range signal
/// (`φ_k = -2π·f_k·τ`), is preserved. A downstream analyzer can therefore
/// recover displacement and Doppler of the dominant path from it.
///
/// Emitted only in sim mode (QEMU RF-sim feed) at the full measurement rate.
/// The 24-byte header carries the pairing `seq` and the producer-stamped
/// `t_us` (µs) — the payload is deliberately just the phase array so the frame
/// stays 136 B total (112 B payload + 24 B header ≈ 59 % of the 46080 B/s
/// byte budget at 200 Hz). Phase is fixed-point radians × 1000 in `i16`:
/// resolution 0.001 rad (≈ 0.04 mm displacement at 2.4 GHz), range ±32.7 rad,
/// well inside the [-π, π] span of one `atan2` sample.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsiPhase {
    /// RAW phase per active subcarrier, radians × 1000.
    pub phase: [i16; N_SUBCARRIERS],
}

impl Default for CsiPhase {
    fn default() -> Self {
        // Manual impl: `[i16; N_SUBCARRIERS]` (> 32 elements) has no derive.
        Self {
            phase: [0; N_SUBCARRIERS],
        }
    }
}

/// Calibration stages (§17).
pub mod cal_stage {
    pub const IDENTITY: u8 = 1;
    pub const RF_POWER: u8 = 2;
    pub const EMPTY_ROOM: u8 = 3;
    pub const MOVING_TEST: u8 = 4;
    pub const FINGERPRINT: u8 = 5;
}

pub mod cal_action {
    pub const START: u8 = 1;
    pub const ABORT: u8 = 2;
    pub const COLLECT: u8 = 3; // accumulate `collect_ms` then respond
    pub const DONE: u8 = 4;
}

/// TX -> RX calibration command payload.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CalCmd {
    pub stage: u8,  // cal_stage::*
    pub action: u8, // cal_action::*
    pub collect_ms: u32,
    pub tx_power_db: i16, // proposed TX power (RF power sweep)
}

pub mod cal_result {
    pub const OK: u8 = 0;
    pub const ERR: u8 = 1;
    pub const TIMEOUT: u8 = 2;
    pub const SAT: u8 = 3; // receiver saturated, retry at lower power
}

/// RX -> TX calibration response payload.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CalResp {
    pub stage: u8,
    pub result: u8, // cal_result::*
    pub rssi: i16,
    pub snr: i8,
    pub csi_quality: u8,
    pub sat_score: u8,
    pub dyn_range: u8,
    pub reserved: u8,
    pub noise_floor: f32,
    pub n_samples: u32,
}

/// Diagnostics status payload.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    pub uptime_s: u32,
    pub tx_seq: u32,
    pub rx_packets: u32,
    pub rx_drops: u32,
    pub heap_free: u16,
    pub wifi_connected: u8,
    pub csi_enabled: u8,
    pub paired_frames_per_s: u8,
    pub reserved: u8,
}

/// Header size for serialization sizing.
pub const HEADER_SIZE: usize = core::mem::size_of::<Header>();

/// Serialize `header` + `payload` into `dst`. Returns bytes written, or 0 if
/// `dst` is too small. Fills in the CRC.
pub fn build(dst: &mut [u8], hdr: &Header, payload: &[u8]) -> usize {
    let total = HEADER_SIZE + hdr.payload_len as usize;
    if dst.len() < total {
        return 0;
    }
    let mut h = *hdr;
    h.payload_len = payload.len() as u16;
    // Copy header with crc16 zeroed, then compute CRC over header + payload.
    unsafe {
        let src_bytes =
            core::slice::from_raw_parts((&h as *const Header) as *const u8, HEADER_SIZE);
        dst[..HEADER_SIZE].copy_from_slice(src_bytes);
    }
    dst[HEADER_SIZE..total].copy_from_slice(payload);
    // Recompute CRC over the header-with-zero-crc region + payload.
    let mut crc_hdr = h;
    crc_hdr.crc16 = 0;
    let crc = {
        unsafe {
            let src_bytes =
                core::slice::from_raw_parts((&crc_hdr as *const Header) as *const u8, HEADER_SIZE);
            let mut c = crc16(src_bytes);
            c = crc16_ext(c, payload);
            c
        }
    };
    dst[HEADER_SIZE - 2..HEADER_SIZE].copy_from_slice(&crc.to_le_bytes());
    total
}

/// Extend a running CRC over a payload (see [`build`]).
pub fn crc16_ext(crc: u16, data: &[u8]) -> u16 {
    crc::crc16_ext(crc, data)
}

/// Parse and validate a received frame. On success returns the header and the
/// payload slice (a view into `src`). Magic, version and CRC are checked.
pub fn parse(src: &[u8]) -> Option<(Header, &[u8])> {
    if src.len() < HEADER_SIZE {
        return None;
    }
    let hdr: Header = unsafe { (src.as_ptr() as *const Header).read_unaligned() };
    if hdr.magic != MAGIC || hdr.version != VERSION {
        return None;
    }
    let total = HEADER_SIZE + hdr.payload_len as usize;
    if src.len() < total {
        return None;
    }
    let payload = &src[HEADER_SIZE..total];
    // Verify CRC.
    let mut crc_hdr = hdr;
    let stored_crc = hdr.crc16;
    crc_hdr.crc16 = 0;
    let mut c = {
        unsafe {
            let src_bytes =
                core::slice::from_raw_parts((&crc_hdr as *const Header) as *const u8, HEADER_SIZE);
            crc16(src_bytes)
        }
    };
    c = crc16_ext(c, payload);
    if c != stored_crc {
        return None;
    }
    Some((hdr, payload))
}

/// Deserialize a CSI snapshot payload (must be at least the fixed struct size).
pub fn parse_csi_snapshot(payload: &[u8]) -> Option<CsiSnapshot> {
    if payload.len() < core::mem::size_of::<CsiSnapshot>() {
        return None;
    }
    Some(unsafe { (payload.as_ptr() as *const CsiSnapshot).read_unaligned() })
}

/// Deserialize a CSI phase payload (must be at least the fixed struct size).
pub fn parse_csi_phase(payload: &[u8]) -> Option<CsiPhase> {
    if payload.len() < core::mem::size_of::<CsiPhase>() {
        return None;
    }
    Some(unsafe { (payload.as_ptr() as *const CsiPhase).read_unaligned() })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csi_snapshot_size_fits_max_payload() {
        assert!(
            core::mem::size_of::<CsiSnapshot>() <= MAX_PAYLOAD,
            "snapshot too large for MAX_PAYLOAD"
        );
    }

    #[test]
    fn csi_snapshot_roundtrip() {
        // Arrays are built as locals and copied in by value — mutating a
        // packed struct's array fields in place would form misaligned refs.
        let mut iq = [0i16; N_SUBCARRIERS * 2];
        iq[0] = 1234;
        iq[1] = -5678;
        iq[N_SUBCARRIERS * 2 - 1] = 42;
        let mut amp = [0u8; N_SUBCARRIERS];
        amp[3] = 200;
        amp[55] = 7;
        let mut spec = [0u8; N_SPEC_BINS];
        spec[63] = 250;
        let snap = CsiSnapshot {
            seq: 1234,
            rssi: -58,
            snr: 24,
            csi_quality: 92,
            noise_floor: -97.5,
            flags: snapshot_flags::OVERFLOW,
            n_sub: 56,
            reserved: 0,
            iq,
            amp_norm: amp,
            spec,
        };

        let payload = unsafe {
            core::slice::from_raw_parts(
                (&snap as *const CsiSnapshot) as *const u8,
                core::mem::size_of::<CsiSnapshot>(),
            )
        };
        let hdr = Header::new(
            frame_type::CSI_SNAPSHOT,
            node::RX2,
            node::TX,
            1234,
            99,
            payload.len() as u16,
        );
        let mut buf = [0u8; 1024];
        let n = build(&mut buf, &hdr, payload);
        assert!(n > 0);

        let (parsed, pl) = parse(&buf[..n]).expect("snapshot frame parses");
        // u8 fields are 1-byte aligned, so direct comparison is safe; wider
        // packed fields must be copied by value before use (see test below).
        assert_eq!(parsed.kind, frame_type::CSI_SNAPSHOT);
        assert_eq!(parsed.src_node, node::RX2);
        let back = parse_csi_snapshot(pl).unwrap();
        // Copy packed fields out by value (indexing a packed array field would
        // form a misaligned reference — E0793).
        let seq = back.seq;
        let iq = back.iq;
        let amp = back.amp_norm;
        let spec = back.spec;
        assert_eq!(seq, 1234);
        assert_eq!(iq[0], 1234);
        assert_eq!(iq[1], -5678);
        assert_eq!(iq[N_SUBCARRIERS * 2 - 1], 42);
        assert_eq!(amp[3], 200);
        assert_eq!(spec[63], 250);
    }

    #[test]
    fn csi_snapshot_rejects_short_payload() {
        assert!(parse_csi_snapshot(&[0u8; 8]).is_none());
    }

    #[test]
    fn csi_phase_size_and_roundtrip() {
        assert!(core::mem::size_of::<CsiPhase>() <= MAX_PAYLOAD);
        // 112 B payload + 24 B header = 136 B/frame (the size budget the sim
        // link-budget relies on).
        assert_eq!(core::mem::size_of::<CsiPhase>(), N_SUBCARRIERS * 2);
        let mut phase = [0i16; N_SUBCARRIERS];
        phase[0] = 3141; // ~π rad
        phase[27] = -1571; // ~-π/2
        phase[55] = 5;
        let cph = CsiPhase { phase };

        let payload = unsafe {
            core::slice::from_raw_parts(
                (&cph as *const CsiPhase) as *const u8,
                core::mem::size_of::<CsiPhase>(),
            )
        };
        let hdr = Header::new(
            frame_type::CSI_PHASE,
            node::RX2,
            node::TX,
            1234,
            99,
            payload.len() as u16,
        );
        let mut buf = [0u8; 1024];
        let n = build(&mut buf, &hdr, payload);
        assert!(n > 0);
        assert_eq!(n, HEADER_SIZE + core::mem::size_of::<CsiPhase>());

        let (parsed, pl) = parse(&buf[..n]).expect("phase frame parses");
        let kind = parsed.kind;
        let seq = parsed.seq;
        let t_us = parsed.t_us;
        assert_eq!(kind, frame_type::CSI_PHASE);
        assert_eq!(seq, 1234);
        assert_eq!(t_us, 99);
        let back = parse_csi_phase(pl).unwrap();
        // Packed fields must be copied by value before use (E0793).
        let phase = back.phase;
        assert_eq!(phase[0], 3141);
        assert_eq!(phase[27], -1571);
        assert_eq!(phase[55], 5);
    }

    #[test]
    fn csi_phase_rejects_short_payload() {
        assert!(parse_csi_phase(&[0u8; 8]).is_none());
    }

    #[test]
    fn crc_known_vector() {
        // CRC-16/XMODEM of "123456789" is 0x31C3.
        assert_eq!(crc16(b"123456789"), 0x31C3);
    }

    #[test]
    fn build_parse_roundtrip() {
        let mut buf = [0u8; 512];
        let mut report = FeatureReport {
            seq: 42,
            n_frames: 200,
            motion_energy: 1.5,
            rssi: -55,
            ..Default::default()
        };
        report.pca_scores[0] = 0.25;
        let payload = unsafe {
            core::slice::from_raw_parts(
                (&report as *const FeatureReport) as *const u8,
                core::mem::size_of::<FeatureReport>(),
            )
        };
        let hdr = Header::new(
            frame_type::FEATURE_REPORT,
            node::RX1,
            node::TX,
            42,
            123456789,
            payload.len() as u16,
        );
        let n = build(&mut buf, &hdr, payload);
        assert!(n > 0);

        let (parsed, pl) = parse(&buf[..n]).expect("frame parses");
        // Packed-struct fields must be read by value, not by reference.
        let kind = parsed.kind;
        let p_seq = parsed.seq;
        let p_src = parsed.src_node;
        assert_eq!(kind, frame_type::FEATURE_REPORT);
        assert_eq!(p_seq, 42);
        assert_eq!(p_src, node::RX1);
        let back: FeatureReport = unsafe { (pl.as_ptr() as *const FeatureReport).read_unaligned() };
        let b_seq = back.seq;
        let b_energy = back.motion_energy;
        let b_pca0 = back.pca_scores[0];
        assert_eq!(b_seq, 42);
        assert!((b_energy - 1.5).abs() < 1e-6, "energy {}", b_energy);
        assert_eq!(b_pca0, 0.25);
    }

    #[test]
    fn parse_rejects_bad_crc() {
        let mut buf = [0u8; 64];
        let hdr = Header::new(frame_type::STATUS, node::TX, node::RX1, 1, 0, 0);
        let n = build(&mut buf, &hdr, &[]);
        buf[0] ^= 0xFF; // corrupt magic
        assert!(parse(&buf[..n]).is_none());
    }
}
