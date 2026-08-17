//! CSI byte decoding, subcarrier selection, phase sanitization and
//! baseline normalization.
//!
//! ESP32 CSI buffer layout (HT20, `I8` data type): the raw buffer holds
//! interleaved I/Q `i8` pairs, one pair per FFT bin, in the order
//! subcarrier -32 .. +31 (bin index 0 == subcarrier -32, bin 32 == DC).
//! We keep the 56 occupied HT20 subcarriers (-28..-1 and +1..+28) and drop
//! DC and the guard bands.

use crate::{Channel, Complex, N_SUBCARRIERS};

/// Subcarrier indices (FFT bin positions) kept by the radar.
/// `bin = subcarrier + 32`.
pub fn valid_bins() -> [usize; N_SUBCARRIERS] {
    let mut out = [0usize; N_SUBCARRIERS];
    let mut k = 0;
    // subcarriers -28..=-1  -> bins 4..=31
    for sub in -28i32..0 {
        out[k] = (sub + 32) as usize;
        k += 1;
    }
    // subcarriers +1..=+28 -> bins 33..=60
    for sub in 1i32..=28 {
        out[k] = (sub + 32) as usize;
        k += 1;
    }
    out
}

/// Relative frequency (kHz from centre) of each valid subcarrier. Useful for
/// correct x-axis labels on the waterfall. Subcarrier spacing is 312.5 kHz.
pub fn subcarrier_khz() -> [f32; N_SUBCARRIERS] {
    const SPACING_KHZ: f32 = 312.5;
    let mut out = [0.0f32; N_SUBCARRIERS];
    let mut k = 0;
    for sub in -28i32..0 {
        out[k] = sub as f32 * SPACING_KHZ;
        k += 1;
    }
    for sub in 1i32..=28 {
        out[k] = sub as f32 * SPACING_KHZ;
        k += 1;
    }
    out
}

/// Decode a raw int8 interleaved I/Q CSI buffer into a [`Channel`].
///
/// `buf` must contain at least `2 * max_bin` bytes. `first_word_invalid` skips
/// the first two complex samples (hardware limitation, ESP32). `rssi` and
/// `noise_floor` are copied from the packet radio metadata.
pub fn decode_channel(
    buf: &[i8],
    first_word_invalid: bool,
    rssi: i16,
    noise_floor: i16,
) -> Channel {
    let bins = valid_bins();
    let max_bin = *bins.iter().max().unwrap_or(&0);
    let needed = (max_bin + 1) * 2; // I/Q i8 pair per bin
    let offset = if first_word_invalid { 4 } else { 0 }; // skip 2 complex samples

    let mut ch = Channel {
        rssi,
        noise_floor,
        valid: buf.len() >= needed + offset,
        ..Default::default()
    };
    if !ch.valid {
        return ch;
    }

    let mut raw = [Complex::default(); N_SUBCARRIERS];
    for (i, &bin) in bins.iter().enumerate() {
        let base = offset + bin * 2;
        let re = buf[base] as f32;
        let im = buf[base + 1] as f32;
        raw[i] = Complex::new(re, im);
    }

    let mut phase = [0.0f32; N_SUBCARRIERS];
    for (i, c) in raw.iter().enumerate() {
        ch.amps[i] = c.mag();
        phase[i] = c.phase();
    }
    ch.raw_phase = phase;
    sanitize_phase(&mut phase);
    ch.phase = phase;
    ch
}

/// Coherent per-packet phase increment across subcarriers, radians.
///
/// For each subcarrier the conjugate product `cur · conj(prev)` rotates by the
/// displacement phase `Δφ_k = -2π·f_k·Δτ` (identical across subcarriers up to
/// the ±0.36 % carrier spread). Summing the products magnitude-weights the
/// combine (~√56 SNR gain) and `atan2` of the sum unwraps 2π automatically —
/// robust against the CFO ramp that would otherwise alias a raw phase
/// difference.
///
/// `Δφ̄` maps to motion as:
///   Doppler  f_d = Δφ̄·fs/(2π)          (positive when approaching)
///   velocity v   = -Δφ̄·fs·c/(4π·f_c)
pub fn phase_increment(prev: &Channel, cur: &Channel) -> f32 {
    let mut re = 0.0f32;
    let mut im = 0.0f32;
    for k in 0..N_SUBCARRIERS {
        let d = cur.raw_phase[k] - prev.raw_phase[k];
        let (s, c) = d.sin_cos();
        let w = cur.amps[k] * prev.amps[k];
        re += w * c;
        im += w * s;
    }
    im.atan2(re)
}

/// Remove the linear phase slope across subcarriers (CFO/SFO component),
/// leaving the frequency-dependent multipath phase structure we care about.
pub fn sanitize_phase(phase: &mut [f32]) {
    let n = phase.len() as f32;
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sxx = 0.0;
    let mut sxy = 0.0;
    for (i, &p) in phase.iter().enumerate() {
        let x = i as f32;
        sx += x;
        sy += p;
        sxx += x * x;
        sxy += x * p;
    }
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-9 {
        return;
    }
    let a = (n * sxy - sx * sy) / denom;
    let b = (sy - a * sx) / n;
    for (i, p) in phase.iter_mut().enumerate() {
        *p -= a * (i as f32) + b;
    }
}

/// Per-subcarrier baseline normalization (z-score against an empty-room
/// baseline). `base[k]`/`scale[k]` come from `radar_calibration`.
#[derive(Clone, Debug)]
pub struct Normalizer {
    pub base: [f32; N_SUBCARRIERS],
    pub scale: [f32; N_SUBCARRIERS], // std, floored to avoid div-by-zero
}

impl Default for Normalizer {
    fn default() -> Self {
        Self {
            base: [0.0; N_SUBCARRIERS],
            scale: [1.0; N_SUBCARRIERS],
        }
    }
}

impl Normalizer {
    /// Apply baseline subtraction + z-score to amplitude.
    pub fn normalize(&self, amps: &[f32; N_SUBCARRIERS]) -> [f32; N_SUBCARRIERS] {
        let mut out = [0.0f32; N_SUBCARRIERS];
        for i in 0..N_SUBCARRIERS {
            out[i] = (amps[i] - self.base[i]) / self.scale[i].max(1e-6);
        }
        out
    }
}

/// Spatial (across-subcarrier) moving-average smoothing for the waterfall
/// display — adjacent subcarriers are correlated, so this is a cheap
/// denoise step for visualization.
pub fn spatial_smooth(amps: &[f32; N_SUBCARRIERS], radius: usize) -> [f32; N_SUBCARRIERS] {
    let mut out = [0.0f32; N_SUBCARRIERS];
    for (i, slot) in out.iter_mut().enumerate() {
        let lo = i.saturating_sub(radius);
        let hi = (i + radius).min(N_SUBCARRIERS - 1);
        let sum: f32 = amps[lo..=hi].iter().sum();
        *slot = sum / (hi - lo + 1) as f32;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_bins_exclude_dc_and_guards() {
        let bins = valid_bins();
        assert_eq!(bins.len(), 56);
        assert!(!bins.contains(&32), "DC bin must be excluded");
        assert!(!bins.contains(&0) && !bins.contains(&1) && !bins.contains(&63));
        // Monotonic, first = 4 (subcarrier -28), last = 60 (subcarrier +28).
        assert_eq!(bins[0], 4);
        assert_eq!(bins[55], 60);
    }

    #[test]
    fn decode_constant_channel() {
        // Constant I=100, Q=0 across bins → amp 100, phase 0.
        let mut buf = [0i8; 128];
        for i in (0..128).step_by(2) {
            buf[i] = 100;
        }
        let ch = decode_channel(&buf, false, -50, -100);
        assert!(ch.valid);
        assert_eq!(ch.rssi, -50);
        assert!((ch.amps[0] - 100.0).abs() < 0.01);
        assert!((ch.mean_amp() - 100.0).abs() < 0.01);
        // Sanitized phase of a constant channel is ~0.
        assert!(
            ch.phase.iter().all(|p| p.abs() < 1e-3),
            "phase {:?}",
            &ch.phase[..4]
        );
    }

    #[test]
    fn decode_linear_phase_is_removed() {
        // A linear phase ramp across subcarriers (simulated CFO) must be
        // removed by sanitization, leaving a residual near zero.
        let mut phase: Vec<f32> = (0..56).map(|i| 0.05 * i as f32).collect();
        sanitize_phase(&mut phase);
        assert!(phase.iter().all(|p| p.abs() < 1e-3));
    }

    #[test]
    fn raw_phase_kept_and_sanitized_slope_removed() {
        // A constant-amplitude channel with a small linear phase ramp across
        // subcarriers (the dominant-path delay signature). raw_phase must
        // preserve the ramp; the sanitized phase (slope removed) must be ~0.
        let mut buf = [0i8; 128];
        for (k, &bin) in valid_bins().iter().enumerate() {
            let phi = 0.002 * k as f32; // small enough that atan2 never wraps
            let (s, c) = phi.sin_cos();
            buf[bin * 2] = (80.0 * c).round() as i8;
            buf[bin * 2 + 1] = (80.0 * s).round() as i8;
        }
        let ch = decode_channel(&buf, false, -50, -100);
        assert!(ch.valid);
        for (k, &rp) in ch.raw_phase.iter().enumerate() {
            let want = 0.002 * k as f32;
            assert!(
                (rp - want).abs() < 0.02,
                "k={k} raw {rp} vs {want} (i8 I/Q quantization ~0.5/80 rad)"
            );
        }
        assert!(
            ch.phase.iter().all(|p| p.abs() < 0.02),
            "sanitized phase {:?}",
            &ch.phase[..4]
        );
    }

    #[test]
    fn phase_increment_recovers_displacement() {
        // Consecutive channels whose raw phase advances by a common Δφ on
        // every subcarrier (a moving target: Δφ = -2π·f_k·Δτ ≈ constant across
        // the ±0.36% carrier spread). phase_increment must return Δφ.
        let mut prev = Channel::default();
        let mut cur = Channel::default();
        let dphi = 0.1f32;
        for k in 0..N_SUBCARRIERS {
            prev.amps[k] = 100.0;
            cur.amps[k] = 100.0;
            prev.raw_phase[k] = 0.7 + 0.001 * k as f32;
            cur.raw_phase[k] = 0.7 + 0.001 * k as f32 + dphi;
        }
        let got = phase_increment(&prev, &cur);
        assert!((got - dphi).abs() < 1e-4, "got {got}, want {dphi}");
    }

    #[test]
    fn phase_increment_handles_2pi_wrap() {
        // The same +0.1 rad increment, but the raw phases straddle ±π so a
        // naive per-subcarrier difference would alias to -6.2 rad. The coherent
        // conjugate-product sum must still recover +0.1.
        let mut prev = Channel::default();
        let mut cur = Channel::default();
        for k in 0..N_SUBCARRIERS {
            prev.amps[k] = 100.0;
            cur.amps[k] = 100.0;
            prev.raw_phase[k] = core::f32::consts::PI - 0.04; // ~3.10
            // 3.10 + 0.1 wraps past π → -3.083
            cur.raw_phase[k] = prev.raw_phase[k] + 0.1 - 2.0 * core::f32::consts::PI;
        }
        let got = phase_increment(&prev, &cur);
        assert!((got - 0.1).abs() < 1e-4, "got {got}, want 0.1");
    }

    #[test]
    fn normalizer_matches_baseline() {
        let n = Normalizer {
            base: [10.0; N_SUBCARRIERS],
            scale: [2.0; N_SUBCARRIERS],
        };
        let amps = [10.0; N_SUBCARRIERS];
        let out = n.normalize(&amps);
        assert!(out.iter().all(|&v| v.abs() < 1e-6));
    }
}
