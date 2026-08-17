//! Compact binary telemetry for the live dashboard (spec §6).
//!
//! RADAR-TX pushes three frame kinds over the WebSocket to connected
//! dashboards, plus a JSON snapshot over plain HTTP for non-WS clients:
//!
//! | kind | frame | payload |
//! |------|-------|---------|
//! | `0x01` | [`StatusFrame`] | live status + occupancy + differential stats |
//! | `0x02` | [`WaterfallFrame`] | per-link normalized CSI amplitude matrix (time × subcarrier) |
//! | `0x03` | [`SpectrogramFrame`] | STFT/PCA motion spectrum matrix (time × frequency) |
//!
//! All integers are little-endian, all matrices are 8-bit normalized values
//! packed row-major (time bin-major). Every frame starts with the same header:
//! `magic u32 = TELEMETRY_MAGIC, version u8, kind u8`.
//!
//! This is the *wire format* — the JavaScript dashboard decodes these exact
//! byte layouts (see `crates/radar_web/static/app.js`).

use radar_features::OccupancyState;

/// Magic bytes identifying a radar telemetry frame: `"RTM1"`.
pub const TELEMETRY_MAGIC: u32 = 0x52544D31;
/// Frame format version.
pub const TELEMETRY_VERSION: u8 = 1;

/// Frame kinds.
pub mod kind {
    pub const STATUS: u8 = 0x01;
    pub const WATERFALL: u8 = 0x02;
    pub const SPECTROGRAM: u8 = 0x03;
}

/// Link identifiers used by matrix frames.
pub mod link {
    pub const RX1: u8 = 1;
    pub const RX2: u8 = 2;
    pub const FUSED: u8 = 3;
}

/// Errors from the frame encoders.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// `buf` is too small for the frame.
    TooSmall { need: usize, have: usize },
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EncodeError::TooSmall { need, have } => {
                write!(f, "telemetry frame needs {need} bytes, buffer has {have}")
            }
        }
    }
}

/// Occupancy state → wire byte (`1..=6`, `0` = UNKNOWN).
pub fn occupancy_to_u8(s: OccupancyState) -> u8 {
    match s {
        OccupancyState::Unknown => 0,
        OccupancyState::Empty => 1,
        OccupancyState::PossiblePresence => 2,
        OccupancyState::StaticPresence => 3,
        OccupancyState::Movement => 4,
        OccupancyState::StrongMovement => 5,
        OccupancyState::ComplexMovement => 6,
    }
}

/// Wire byte → occupancy state (unknown bytes decode to `Unknown`).
pub fn occupancy_from_u8(v: u8) -> OccupancyState {
    match v {
        1 => OccupancyState::Empty,
        2 => OccupancyState::PossiblePresence,
        3 => OccupancyState::StaticPresence,
        4 => OccupancyState::Movement,
        5 => OccupancyState::StrongMovement,
        6 => OccupancyState::ComplexMovement,
        _ => OccupancyState::Unknown,
    }
}

/// Human-readable occupancy name for the JSON status endpoint.
pub fn occupancy_name(s: OccupancyState) -> &'static str {
    use OccupancyState::*;
    match s {
        Unknown => "UNKNOWN",
        Empty => "EMPTY",
        PossiblePresence => "POSSIBLE PRESENCE",
        StaticPresence => "STATIC PRESENCE",
        Movement => "MOVEMENT",
        StrongMovement => "STRONG MOVEMENT",
        ComplexMovement => "COMPLEX/MULTIPLE MOVEMENT",
    }
}

/// Single-shot live status: everything the LIVE STATUS and DIFFERENTIAL
/// CHANNEL blocks need, packed into 66 bytes.
///
/// Layout (all LE):
/// `magic u32 | version u8 | kind u8=0x01 | occupancy u8 | confidence u8 |
/// tx_power_db i8 | rssi_rx1 i8 | rssi_rx2 i8 | csi_quality_rx1 u8 |
/// csi_quality_rx2 u8 | sat_rx1 u8 | sat_rx2 u8 | dyn_rx1 u8 | dyn_rx2 u8 |
/// packet_delivery_pct u8 | paired_frames_s u16 | seq u32 | t_us u64 |
/// motion_energy_rx1 f32 | motion_energy_rx2 f32 | motion_energy_fused f32 |
/// spectral_entropy f32 | dominant_freq_hz u16 | pca1 f32 | pca2 f32 |
/// correlation f32 | differential f32`
#[derive(Clone, Copy, Debug)]
pub struct StatusFrame {
    pub seq: u32,
    pub t_us: u64,
    pub occupancy: OccupancyState,
    /// 0..=100
    pub confidence: u8,
    /// dBm; `0` means "not commissioned".
    pub tx_power_db: i8,
    pub rssi_rx1: i8,
    pub rssi_rx2: i8,
    pub csi_quality_rx1: u8,
    pub csi_quality_rx2: u8,
    pub sat_score_rx1: u8,
    pub sat_score_rx2: u8,
    pub dyn_range_rx1: u8,
    pub dyn_range_rx2: u8,
    /// 0..=100
    pub packet_delivery_pct: u8,
    pub paired_frames_s: u16,
    pub motion_energy_rx1: f32,
    pub motion_energy_rx2: f32,
    pub motion_energy_fused: f32,
    pub spectral_entropy: f32,
    pub dominant_freq_hz: u16,
    pub pca1: f32,
    pub pca2: f32,
    pub correlation: f32,
    pub differential: f32,
}

impl Default for StatusFrame {
    fn default() -> Self {
        Self {
            seq: 0,
            t_us: 0,
            occupancy: OccupancyState::Unknown,
            confidence: 0,
            tx_power_db: 0,
            rssi_rx1: 0,
            rssi_rx2: 0,
            csi_quality_rx1: 0,
            csi_quality_rx2: 0,
            sat_score_rx1: 0,
            sat_score_rx2: 0,
            dyn_range_rx1: 0,
            dyn_range_rx2: 0,
            packet_delivery_pct: 0,
            paired_frames_s: 0,
            motion_energy_rx1: 0.0,
            motion_energy_rx2: 0.0,
            motion_energy_fused: 0.0,
            spectral_entropy: 0.0,
            dominant_freq_hz: 0,
            pca1: 0.0,
            pca2: 0.0,
            correlation: 0.0,
            differential: 0.0,
        }
    }
}

impl StatusFrame {
    /// Encoded size (see layout above): 6-byte header + 60 bytes of payload.
    pub const LEN: usize = 66;

    /// Encode into `buf`, returning the number of bytes written (always
    /// [`StatusFrame::LEN`]).
    pub fn encode(&self, buf: &mut [u8]) -> Result<usize, EncodeError> {
        let mut w = Writer::new(buf);
        w.put(&TELEMETRY_MAGIC.to_le_bytes())?;
        w.u8(TELEMETRY_VERSION)?;
        w.u8(kind::STATUS)?;
        w.u8(occupancy_to_u8(self.occupancy))?;
        w.u8(self.confidence)?;
        w.i8(self.tx_power_db)?;
        w.i8(self.rssi_rx1)?;
        w.i8(self.rssi_rx2)?;
        w.u8(self.csi_quality_rx1)?;
        w.u8(self.csi_quality_rx2)?;
        w.u8(self.sat_score_rx1)?;
        w.u8(self.sat_score_rx2)?;
        w.u8(self.dyn_range_rx1)?;
        w.u8(self.dyn_range_rx2)?;
        w.u8(self.packet_delivery_pct)?;
        w.u16(self.paired_frames_s)?;
        w.u32(self.seq)?;
        w.u64(self.t_us)?;
        w.f32(self.motion_energy_rx1)?;
        w.f32(self.motion_energy_rx2)?;
        w.f32(self.motion_energy_fused)?;
        w.f32(self.spectral_entropy)?;
        w.u16(self.dominant_freq_hz)?;
        w.f32(self.pca1)?;
        w.f32(self.pca2)?;
        w.f32(self.correlation)?;
        w.f32(self.differential)?;
        debug_assert_eq!(w.pos, Self::LEN);
        Ok(w.pos)
    }
}

/// Per-link CSI waterfall (kind `0x02`): a time × subcarrier matrix of
/// normalized amplitudes, 8-bit values.
///
/// Layout (all LE): `magic u32 | version u8 | kind u8=0x02 | link u8 |
/// n_sub u8 | bins u16 | scale u8 | data[n_sub*bins]`
pub struct WaterfallFrame<'a> {
    /// [`link::RX1`] or [`link::RX2`].
    pub link: u8,
    /// Number of subcarriers per time bin (fixed head: 56).
    pub n_sub: u8,
    /// Number of time bins.
    pub bins: u16,
    /// Right-shift applied to 16-bit amplitudes before packing to u8, so the
    /// dashboard can reconstruct approximate values: `amp ≈ raw << scale`.
    pub scale: u8,
    /// `n_sub * bins` 8-bit values, time-major.
    pub data: &'a [u8],
}

impl WaterfallFrame<'_> {
    /// 11-byte header + payload.
    pub fn len(&self) -> usize {
        11 + self.n_sub as usize * self.bins as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn encode(&self, buf: &mut [u8]) -> Result<usize, EncodeError> {
        let mut w = Writer::new(buf);
        w.put(&TELEMETRY_MAGIC.to_le_bytes())?;
        w.u8(TELEMETRY_VERSION)?;
        w.u8(kind::WATERFALL)?;
        w.u8(self.link)?;
        w.u8(self.n_sub)?;
        w.u16(self.bins)?;
        w.u8(self.scale)?;
        w.put(self.data)?;
        Ok(w.pos)
    }
}

/// Motion spectrogram (kind `0x03`): a time × frequency matrix of STFT/PCA
/// magnitude, 8-bit values. Used for RX1, RX2 and fused spectra.
///
/// Layout (all LE): `magic u32 | version u8 | kind u8=0x03 | link u8 |
/// n_freq u8 | bins u16 | scale u8 | data[n_freq*bins]`
pub struct SpectrogramFrame<'a> {
    /// [`link::RX1`], [`link::RX2`] or [`link::FUSED`].
    pub link: u8,
    /// Number of frequency bins.
    pub n_freq: u8,
    /// Number of time bins.
    pub bins: u16,
    /// Right-shift used before packing to u8.
    pub scale: u8,
    /// `n_freq * bins` 8-bit values, time-major.
    pub data: &'a [u8],
}

impl SpectrogramFrame<'_> {
    pub fn len(&self) -> usize {
        11 + self.n_freq as usize * self.bins as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn encode(&self, buf: &mut [u8]) -> Result<usize, EncodeError> {
        let mut w = Writer::new(buf);
        w.put(&TELEMETRY_MAGIC.to_le_bytes())?;
        w.u8(TELEMETRY_VERSION)?;
        w.u8(kind::SPECTROGRAM)?;
        w.u8(self.link)?;
        w.u8(self.n_freq)?;
        w.u16(self.bins)?;
        w.u8(self.scale)?;
        w.put(self.data)?;
        Ok(w.pos)
    }
}

/// The live status snapshot RADAR-TX keeps for the `/status` JSON endpoint and
/// to hand-build the WebSocket [`StatusFrame`] from.
#[derive(Clone, Debug)]
pub struct StatusSnapshot {
    pub frame: StatusFrame,
    /// 2.4 GHz channel the AP runs on.
    pub channel: u8,
    /// Measurement frames/second.
    pub tx_rate_hz: u16,
    /// 0 = inactive, 1..=5 = calibration stage (spec §17).
    pub cal_stage: u8,
    pub cal_active: bool,
    /// True once the RX links have reported at least once.
    pub radar_active: bool,
}

impl Default for StatusSnapshot {
    fn default() -> Self {
        Self {
            frame: StatusFrame::default(),
            channel: 6,
            tx_rate_hz: 200,
            cal_stage: 0,
            cal_active: false,
            radar_active: false,
        }
    }
}

impl StatusSnapshot {
    /// Compact JSON for the `/status` endpoint.
    pub fn to_json(&self) -> String {
        let f = &self.frame;
        format!(
            concat!(
                "{{\"radar_active\":{},\"channel\":{},\"tx_rate_hz\":{},\"cal_stage\":{},\"cal_active\":{},",
                "\"occupancy\":\"{}\",\"occupancy_code\":{},\"confidence\":{},\"tx_power_db\":{},",
                "\"rssi_rx1\":{},\"rssi_rx2\":{},\"csi_quality_rx1\":{},\"csi_quality_rx2\":{},",
                "\"sat_score_rx1\":{},\"sat_score_rx2\":{},\"dyn_range_rx1\":{},\"dyn_range_rx2\":{},",
                "\"packet_delivery_pct\":{},\"paired_frames_s\":{},\"seq\":{},\"t_us\":{},",
                "\"motion_energy_rx1\":{:.2},\"motion_energy_rx2\":{:.2},\"motion_energy_fused\":{:.2},",
                "\"spectral_entropy\":{:.3},\"dominant_freq_hz\":{},\"pca1\":{:.2},\"pca2\":{:.2},",
                "\"correlation\":{:.3},\"differential\":{:.3}}}",
            ),
            self.radar_active as u8,
            self.channel,
            self.tx_rate_hz,
            self.cal_stage,
            self.cal_active as u8,
            occupancy_name(f.occupancy),
            occupancy_to_u8(f.occupancy),
            f.confidence,
            f.tx_power_db,
            f.rssi_rx1,
            f.rssi_rx2,
            f.csi_quality_rx1,
            f.csi_quality_rx2,
            f.sat_score_rx1,
            f.sat_score_rx2,
            f.dyn_range_rx1,
            f.dyn_range_rx2,
            f.packet_delivery_pct,
            f.paired_frames_s,
            f.seq,
            f.t_us,
            f.motion_energy_rx1,
            f.motion_energy_rx2,
            f.motion_energy_fused,
            f.spectral_entropy,
            f.dominant_freq_hz,
            f.pca1,
            f.pca2,
            f.correlation,
            f.differential,
        )
    }
}

/// Little-endian byte writer with bounds checks.
struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn put(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        let end = self.pos + bytes.len();
        if end > self.buf.len() {
            return Err(EncodeError::TooSmall {
                need: end,
                have: self.buf.len(),
            });
        }
        self.buf[self.pos..end].copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }

    fn u8(&mut self, v: u8) -> Result<(), EncodeError> {
        self.put(&[v])
    }
    fn i8(&mut self, v: i8) -> Result<(), EncodeError> {
        self.put(&[v as u8])
    }
    fn u16(&mut self, v: u16) -> Result<(), EncodeError> {
        self.put(&v.to_le_bytes())
    }
    fn u32(&mut self, v: u32) -> Result<(), EncodeError> {
        self.put(&v.to_le_bytes())
    }
    fn u64(&mut self, v: u64) -> Result<(), EncodeError> {
        self.put(&v.to_le_bytes())
    }
    fn f32(&mut self, v: f32) -> Result<(), EncodeError> {
        self.put(&v.to_le_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_frame_len_and_magic() {
        let f = StatusFrame::default();
        let mut buf = [0u8; 256];
        let n = f.encode(&mut buf).unwrap();
        assert_eq!(n, StatusFrame::LEN);
        assert_eq!(&buf[0..4], &TELEMETRY_MAGIC.to_le_bytes());
        assert_eq!(buf[4], TELEMETRY_VERSION);
        assert_eq!(buf[5], kind::STATUS);
    }

    #[test]
    fn status_frame_too_small() {
        let f = StatusFrame::default();
        let mut buf = [0u8; 10];
        let err = f.encode(&mut buf).unwrap_err();
        // The writer reports where the buffer ran out, not the total frame size.
        assert!(matches!(err, EncodeError::TooSmall { need, have: 10 } if need > 10));
    }

    #[test]
    fn status_frame_occupancy_wire_value() {
        let f = StatusFrame {
            occupancy: OccupancyState::StrongMovement,
            ..Default::default()
        };
        let mut buf = [0u8; StatusFrame::LEN];
        f.encode(&mut buf).unwrap();
        // header 6 bytes, then occupancy at offset 6
        assert_eq!(buf[6], 5);
        assert_eq!(occupancy_from_u8(buf[6]), OccupancyState::StrongMovement);
    }

    #[test]
    fn waterfall_roundtrip_len() {
        let data = [0u8; 56 * 32];
        let wf = WaterfallFrame {
            link: link::RX1,
            n_sub: 56,
            bins: 32,
            scale: 4,
            data: &data,
        };
        assert_eq!(wf.len(), 11 + 56 * 32);
        let mut buf = [0u8; 4096];
        let n = wf.encode(&mut buf).unwrap();
        assert_eq!(n, wf.len());
        assert_eq!(buf[5], kind::WATERFALL);
        assert_eq!(buf[6], link::RX1);
        assert_eq!(buf[7], 56);
        assert_eq!(&buf[11..n], &data[..]);
    }

    #[test]
    fn spectrogram_roundtrip_len() {
        let data = [7u8; 64 * 20];
        let sp = SpectrogramFrame {
            link: link::FUSED,
            n_freq: 64,
            bins: 20,
            scale: 2,
            data: &data,
        };
        let mut buf = [0u8; 4096];
        let n = sp.encode(&mut buf).unwrap();
        assert_eq!(n, 11 + 64 * 20);
        assert_eq!(buf[5], kind::SPECTROGRAM);
        assert_eq!(buf[6], link::FUSED);
    }

    #[test]
    fn occupancy_mapping_roundtrip() {
        use OccupancyState::*;
        for s in [
            Unknown,
            Empty,
            PossiblePresence,
            StaticPresence,
            Movement,
            StrongMovement,
            ComplexMovement,
        ] {
            assert_eq!(occupancy_from_u8(occupancy_to_u8(s)), s);
        }
        assert_eq!(occupancy_from_u8(0), Unknown);
        assert_eq!(occupancy_from_u8(99), Unknown);
    }

    #[test]
    fn status_snapshot_json() {
        let snap = StatusSnapshot {
            frame: StatusFrame {
                occupancy: OccupancyState::Movement,
                confidence: 87,
                paired_frames_s: 196,
                motion_energy_fused: 1.234,
                ..Default::default()
            },
            channel: 6,
            tx_rate_hz: 200,
            cal_stage: 0,
            cal_active: false,
            radar_active: true,
        };
        let json = snap.to_json();
        assert!(json.contains("\"radar_active\":1"));
        assert!(json.contains("\"occupancy\":\"MOVEMENT\""));
        assert!(json.contains("\"confidence\":87"));
        assert!(json.contains("\"paired_frames_s\":196"));
        assert!(json.contains("\"motion_energy_fused\":1.23"));
    }
}
