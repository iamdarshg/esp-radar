//! Fixed-head calibration for the compact radar (spec §11, §17, §6).
//!
//! The boards are permanently mounted and never repositioned, so calibration
//! is about characterising the *fixed* propagation geometry, not about array
//! alignment:
//!
//!  * **CAL 1 — empty-room baseline.** A per-subcarrier mean/std of the quiet
//!    room. This becomes the `Normalizer` that turns raw amplitude into
//!    z-scores, and is the reference for static-presence detection.
//!  * **CAL 2 — TX power sweep.** Walks the TX power from low to high while
//!    the RX links report RSSI/SNR/saturation. Fits a linear model
//!    `rssi = a·p + b` so the TX can auto-commission to the highest power that
//!    does not saturate the receivers (§6 TX power auto-commissioning).
//!  * **CAL 4 — classifier thresholds.** Distilled from a calibration run into
//!    the `ClassThresholds` the occupancy estimator uses.
//!
//! All structures are fixed-size and byte-serializable so they can live in NVS
//! (see `radar_storage`).
//!
//! This crate is pure Rust and std-enabled, like the rest of the pure crates —
//! the device builds against the `xtensa-esp32-espidf` target which provides
//! std, and the host builds run the unit tests.

use radar_dsp::transform::Normalizer;
use radar_dsp::Channel;

/// Where the active classifier thresholds came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ThresholdSource {
    /// Factory defaults — no calibration has been run yet.
    Default = 0,
    /// Overridden by a completed CAL 4 run.
    Calibrated = 1,
}

/// Per-subcarrier statistics of the empty room (CAL 1 output).
#[derive(Clone, Debug)]
pub struct BaselineStats {
    pub mean: [f32; radar_dsp::N_SUBCARRIERS],
    pub std: [f32; radar_dsp::N_SUBCARRIERS],
    pub n_samples: u32,
    /// Mean RSSI observed during collection (dBm).
    pub rssi_mean: i16,
    /// Mean noise floor observed during collection (dBm).
    pub noise_floor: f32,
    pub valid: bool,
}

impl Default for BaselineStats {
    fn default() -> Self {
        Self {
            mean: [0.0; radar_dsp::N_SUBCARRIERS],
            std: [1.0; radar_dsp::N_SUBCARRIERS],
            n_samples: 0,
            rssi_mean: 0,
            noise_floor: 0.0,
            valid: false,
        }
    }
}

impl BaselineStats {
    pub fn normalizer(&self) -> Normalizer {
        Normalizer {
            base: self.mean,
            scale: self.std,
        }
    }

    /// Fixed-size encoding for NVS (CAL 1 persists the baseline).
    /// Layout: mean[56] f32 LE, std[56] f32 LE, n_samples u32 LE,
    /// rssi_mean i16 LE, noise_floor f32 LE, valid u8.
    pub const SERIALIZED_LEN: usize = 56 * 4 + 56 * 4 + 4 + 2 + 4 + 1;

    pub fn to_bytes(&self) -> [u8; Self::SERIALIZED_LEN] {
        let mut out = [0u8; Self::SERIALIZED_LEN];
        let mut o = 0;
        for &m in self.mean.iter() {
            out[o..o + 4].copy_from_slice(&m.to_le_bytes());
            o += 4;
        }
        for &s in self.std.iter() {
            out[o..o + 4].copy_from_slice(&s.to_le_bytes());
            o += 4;
        }
        out[o..o + 4].copy_from_slice(&self.n_samples.to_le_bytes());
        o += 4;
        out[o..o + 2].copy_from_slice(&self.rssi_mean.to_le_bytes());
        o += 2;
        out[o..o + 4].copy_from_slice(&self.noise_floor.to_le_bytes());
        o += 4;
        out[o] = self.valid as u8;
        o += 1;
        debug_assert_eq!(o, Self::SERIALIZED_LEN);
        out
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        let mut s = Self::default();
        if b.len() < Self::SERIALIZED_LEN {
            s.valid = false;
            return s;
        }
        let mut o = 0;
        let read_f32 = |o: &mut usize| -> f32 {
            let v = f32::from_le_bytes([b[*o], b[*o + 1], b[*o + 2], b[*o + 3]]);
            *o += 4;
            v
        };
        for m in s.mean.iter_mut() {
            *m = read_f32(&mut o);
        }
        for st in s.std.iter_mut() {
            *st = read_f32(&mut o);
        }
        s.n_samples = u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        o += 4;
        s.rssi_mean = i16::from_le_bytes([b[o], b[o + 1]]);
        o += 2;
        s.noise_floor = read_f32(&mut o);
        s.valid = b[o] != 0;
        s
    }
}

/// Streaming accumulator for CAL 1. Feed it decoded channels from the quiet
/// room and call `finish` once enough samples are collected.
pub struct BaselineCollector {
    sum: [f64; radar_dsp::N_SUBCARRIERS],
    sumsq: [f64; radar_dsp::N_SUBCARRIERS],
    count: u32,
    rssi_sum: i64,
    noise_sum: f64,
}

impl Default for BaselineCollector {
    fn default() -> Self {
        Self::new()
    }
}

impl BaselineCollector {
    pub fn new() -> Self {
        Self {
            sum: [0.0; radar_dsp::N_SUBCARRIERS],
            sumsq: [0.0; radar_dsp::N_SUBCARRIERS],
            count: 0,
            rssi_sum: 0,
            noise_sum: 0.0,
        }
    }

    pub fn update(&mut self, ch: &Channel, rssi: i16) {
        if !ch.valid {
            return;
        }
        for (s, &a) in self.sum.iter_mut().zip(ch.amps.iter()) {
            let a = a as f64;
            *s += a;
        }
        for (sq, &a) in self.sumsq.iter_mut().zip(ch.amps.iter()) {
            let a = a as f64;
            *sq += a * a;
        }
        self.count += 1;
        self.rssi_sum += rssi as i64;
        self.noise_sum += ch.noise_floor as f64;
    }

    pub fn count(&self) -> u32 {
        self.count
    }

    pub fn finish(&self) -> BaselineStats {
        let n = self.count.max(1) as f64;
        let mut mean = [0.0f32; radar_dsp::N_SUBCARRIERS];
        let mut std = [0.0f32; radar_dsp::N_SUBCARRIERS];
        for i in 0..radar_dsp::N_SUBCARRIERS {
            let m = self.sum[i] / n;
            let v = (self.sumsq[i] / n - m * m).max(0.0);
            mean[i] = m as f32;
            // Floor the scale so a dead subcarrier doesn't blow up z-scores.
            std[i] = (v.sqrt() as f32).max(1e-3);
        }
        BaselineStats {
            mean,
            std,
            n_samples: self.count,
            rssi_mean: (self.rssi_sum as f64 / n).round() as i16,
            noise_floor: (self.noise_sum / n) as f32,
            valid: self.count >= MIN_BASELINE_SAMPLES,
        }
    }

    pub fn reset(&mut self) {
        self.sum = [0.0; radar_dsp::N_SUBCARRIERS];
        self.sumsq = [0.0; radar_dsp::N_SUBCARRIERS];
        self.count = 0;
        self.rssi_sum = 0;
        self.noise_sum = 0.0;
    }
}

/// Minimum quiet-room samples for a CAL 1 baseline to be considered valid.
/// At 20 Hz this is ~5 s of collection.
pub const MIN_BASELINE_SAMPLES: u32 = 100;

/// One TX power sweep point (CAL 2).
#[derive(Clone, Copy, Debug, Default)]
pub struct SweepPoint {
    pub tx_power_db: i16,
    pub rssi: i16,
    pub snr: i8,
    pub csi_quality: u8,
    pub sat_score: u8,
    pub dyn_range: u8,
}

/// Linear TX power ↔ RSSI model fitted from the CAL 2 sweep.
#[derive(Clone, Copy, Debug, Default)]
pub struct TxPowerModel {
    /// dRSSI/dPower (dB per dB), expected ≈ 1.0 for a well-behaved radio.
    pub slope: f32,
    /// RSSI at 0 dBm TX power.
    pub intercept: f32,
    /// Goodness of fit (0..1).
    pub r2: f32,
    pub n_points: u8,
}

impl TxPowerModel {
    /// Least-squares linear fit `rssi = slope·p + intercept`.
    pub fn fit(points: &[SweepPoint]) -> Option<Self> {
        let n = points.len();
        if n < 2 {
            return None;
        }
        let mut sx = 0.0f64;
        let mut sy = 0.0f64;
        let mut sxx = 0.0f64;
        let mut sxy = 0.0f64;
        for p in points {
            let x = p.tx_power_db as f64;
            let y = p.rssi as f64;
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        let denom = n as f64 * sxx - sx * sx;
        if denom.abs() < 1e-9 {
            return None;
        }
        let slope = (n as f64 * sxy - sx * sy) / denom;
        let intercept = (sy - slope * sx) / n as f64;

        // R²
        let mean_y = sy / n as f64;
        let (mut ss_res, mut ss_tot) = (0.0f64, 0.0f64);
        for p in points {
            let y = p.rssi as f64;
            let pred = slope * p.tx_power_db as f64 + intercept;
            ss_res += (y - pred) * (y - pred);
            ss_tot += (y - mean_y) * (y - mean_y);
        }
        let r2 = if ss_tot > 1e-12 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        Some(Self {
            slope: slope as f32,
            intercept: intercept as f32,
            r2: r2 as f32,
            n_points: n as u8,
        })
    }

    pub fn predict_rssi(&self, power_db: i16) -> f32 {
        self.slope * power_db as f32 + self.intercept
    }

    /// Highest TX power whose predicted RSSI stays below `target_rssi`
    /// (i.e. does not saturate the receivers). Clamped to [4, 78] dBm
    /// (ESP32's valid TX power range).
    pub fn power_for_rssi(&self, target_rssi: f32) -> i16 {
        let p = (target_rssi - self.intercept) / self.slope.max(1e-3);
        p.round().clamp(4.0, 78.0) as i16
    }

    /// Fixed-size encoding for NVS (CAL 2 persists the model so the TX can
    /// auto-commission on boot, spec §5).
    /// Layout: slope f32 LE, intercept f32 LE, r2 f32 LE, n_points u8.
    pub const SERIALIZED_LEN: usize = 4 + 4 + 4 + 1;

    pub fn to_bytes(&self) -> [u8; Self::SERIALIZED_LEN] {
        let mut out = [0u8; Self::SERIALIZED_LEN];
        let mut o = 0;
        for v in [self.slope, self.intercept, self.r2] {
            out[o..o + 4].copy_from_slice(&v.to_le_bytes());
            o += 4;
        }
        out[o] = self.n_points;
        out
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        let mut m = Self::default();
        if b.len() < Self::SERIALIZED_LEN {
            return m;
        }
        let mut o = 0;
        let read_f32 = |o: &mut usize| -> f32 {
            let v = f32::from_le_bytes([b[*o], b[*o + 1], b[*o + 2], b[*o + 3]]);
            *o += 4;
            v
        };
        m.slope = read_f32(&mut o);
        m.intercept = read_f32(&mut o);
        m.r2 = read_f32(&mut o);
        m.n_points = b[o];
        m
    }
}

/// Classifier thresholds (CAL 4 output or factory defaults).
#[derive(Clone, Copy, Debug)]
pub struct ClassThresholds {
    pub empty_thresh: f32,
    pub move_thresh: f32,
    pub strong_thresh: f32,
    pub static_thresh: f32,
    pub hold_frames: u32,
    pub source: ThresholdSource,
}

impl Default for ClassThresholds {
    fn default() -> Self {
        Self {
            empty_thresh: 0.05,
            move_thresh: 0.6,
            strong_thresh: 2.5,
            static_thresh: 1.4,
            hold_frames: 6,
            source: ThresholdSource::Default,
        }
    }
}

impl ClassThresholds {
    /// Convert into `radar_features::ClassifierParams`.
    pub fn to_params(&self) -> radar_features::ClassifierParams {
        radar_features::ClassifierParams {
            move_thresh: self.move_thresh,
            strong_thresh: self.strong_thresh,
            static_thresh: self.static_thresh,
            empty_thresh: self.empty_thresh,
            corr_weight: 0.5,
            hold_frames: self.hold_frames,
        }
    }

    pub const SERIALIZED_LEN: usize = 4 + 4 + 4 + 4 + 4 + 1;

    pub fn to_bytes(&self) -> [u8; Self::SERIALIZED_LEN] {
        let mut out = [0u8; Self::SERIALIZED_LEN];
        let mut o = 0;
        for v in [
            self.empty_thresh,
            self.move_thresh,
            self.strong_thresh,
            self.static_thresh,
        ] {
            out[o..o + 4].copy_from_slice(&v.to_le_bytes());
            o += 4;
        }
        out[o..o + 4].copy_from_slice(&self.hold_frames.to_le_bytes());
        o += 4;
        out[o] = self.source as u8;
        out
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        let mut t = Self::default();
        if b.len() < Self::SERIALIZED_LEN {
            return t;
        }
        let mut o = 0;
        let mut f32s = [0.0f32; 4];
        for v in f32s.iter_mut() {
            *v = f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
            o += 4;
        }
        t.empty_thresh = f32s[0];
        t.move_thresh = f32s[1];
        t.strong_thresh = f32s[2];
        t.static_thresh = f32s[3];
        t.hold_frames = u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        o += 4;
        t.source = match b[o] {
            1 => ThresholdSource::Calibrated,
            _ => ThresholdSource::Default,
        };
        t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn baseline_collector_matches_mean() {
        let mut c = BaselineCollector::new();
        for _ in 0..200 {
            let ch = Channel {
                amps: [10.0; radar_dsp::N_SUBCARRIERS],
                rssi: -55,
                noise_floor: -95,
                valid: true,
                ..Default::default()
            };
            c.update(&ch, -55);
        }
        let b = c.finish();
        assert!(b.valid);
        assert!((b.mean[0] - 10.0).abs() < 1e-3);
        assert!(b.std.iter().all(|&s| s < 1e-2));
        assert_eq!(b.rssi_mean, -55);
        assert_eq!(b.n_samples, 200);
    }

    #[test]
    fn baseline_roundtrip_bytes() {
        let b = BaselineStats {
            mean: [3.5; radar_dsp::N_SUBCARRIERS],
            std: [0.25; radar_dsp::N_SUBCARRIERS],
            n_samples: 150,
            rssi_mean: -60,
            noise_floor: -98.5,
            valid: true,
        };
        let bytes = b.to_bytes();
        let c = BaselineStats::from_bytes(&bytes);
        assert!(c.valid);
        assert_eq!(c.n_samples, 150);
        assert_eq!(c.rssi_mean, -60);
        assert!((c.mean[0] - 3.5).abs() < 1e-6);
        assert!((c.std[55] - 0.25).abs() < 1e-6);
        assert!((c.noise_floor - -98.5).abs() < 1e-6);
    }

    #[test]
    fn baseline_requires_min_samples() {
        let mut c = BaselineCollector::new();
        let ch = Channel {
            amps: [1.0; 56],
            valid: true,
            ..Default::default()
        };
        for _ in 0..50 {
            c.update(&ch, -50);
        }
        let b = c.finish();
        assert!(!b.valid, "50 samples < MIN_BASELINE_SAMPLES=100");
    }

    #[test]
    fn tx_power_model_fit() {
        // Perfectly linear: rssi = power - 50.
        let points: Vec<SweepPoint> = [4i16, 20, 40, 60, 78]
            .iter()
            .map(|&p| SweepPoint {
                tx_power_db: p,
                rssi: p - 50,
                ..Default::default()
            })
            .collect();
        let m = TxPowerModel::fit(&points).expect("fit");
        assert!((m.slope - 1.0).abs() < 1e-3, "slope {}", m.slope);
        assert!(
            (m.intercept - -50.0).abs() < 0.1,
            "intercept {}",
            m.intercept
        );
        assert!(m.r2 > 0.99);
        // To keep RSSI at -45 we should pick TX power ≈ +5 (rssi = p - 50).
        let p = m.power_for_rssi(-45.0);
        assert_eq!(p, 5);
    }

    #[test]
    fn tx_power_fit_needs_two_points() {
        let one = [SweepPoint {
            tx_power_db: 20,
            rssi: -30,
            ..Default::default()
        }];
        assert!(TxPowerModel::fit(&one).is_none());
    }

    #[test]
    fn tx_power_model_roundtrip() {
        let m = TxPowerModel {
            slope: 0.98,
            intercept: -51.5,
            r2: 0.992,
            n_points: 5,
        };
        let bytes = m.to_bytes();
        let c = TxPowerModel::from_bytes(&bytes);
        assert!((c.slope - 0.98).abs() < 1e-6);
        assert!((c.intercept - -51.5).abs() < 1e-6);
        assert!((c.r2 - 0.992).abs() < 1e-6);
        assert_eq!(c.n_points, 5);
    }

    #[test]
    fn class_thresholds_roundtrip() {
        let t = ClassThresholds {
            empty_thresh: 0.01,
            move_thresh: 0.5,
            strong_thresh: 2.0,
            static_thresh: 1.0,
            hold_frames: 10,
            source: ThresholdSource::Calibrated,
        };
        let bytes = t.to_bytes();
        let c = ClassThresholds::from_bytes(&bytes);
        assert_eq!(c.move_thresh, 0.5);
        assert_eq!(c.hold_frames, 10);
        assert_eq!(c.source, ThresholdSource::Calibrated);
    }
}
