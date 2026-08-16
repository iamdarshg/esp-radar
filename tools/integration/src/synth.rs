//! Deterministic synthetic CSI + the RX-side DSP pipeline.
//!
//! The real DataFrame payload is only `{ tx_power_db, flags }` — there is no
//! CSI on the wire. So each RX synthesises a per-frame channel *deterministically
//! from the shared TX sequence number*, feeds it through the same `radar_dsp`
//! pipeline the firmware uses, and emits `FeatureReport`s keyed by that seq.
//!
//! The two links share the same motion envelope (a function of `seq` only) so
//! their `motion_energy` is correlated, which is what lets the TX fusion
//! classify MOVEMENT rather than EMPTY. Per-link noise is deterministic from
//! `(seq, link_id)` so runs are reproducible.

use radar_dsp::filter::Biquad;
use radar_dsp::metrics::{circular_variance, dominant_freq_hz, energy, rms};
use radar_dsp::pca::Pca;
use radar_dsp::transform::{decode_channel, Normalizer};
use radar_dsp::Channel;

use crate::common::{REPORT_EVERY};

const REPORT_WINDOW: usize = REPORT_EVERY as usize;
const N_SUB: usize = radar_dsp::N_SUBCARRIERS;

/// Raw i8 CSI buffer handed to `decode_channel`. Covers the 64 FFT bins
/// (-32..+31); `decode_channel` keeps the 56 occupied HT20 bins.
const RAW_BUF: usize = 128;

/// Deterministic noise in -1..1, keyed by (seed, seq, link, salt).
fn hash01(seed: u64, seq: u32, link: u8, salt: u64) -> f32 {
    let mut x = seed
        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
        .wrapping_add(seq as u64 * 0x1000_0001)
        .wrapping_add(link as u64 * 0x3141_5927)
        .wrapping_add(salt * 0x2654_4353);
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    (x as u32 as f32 / u32::MAX as f32) * 2.0 - 1.0
}

/// The RX-side DSP pipeline. Genuinely exercises `radar_dsp` and
/// `radar_calibration` on the host.
pub struct RxDsp {
    sample_rate_hz: f32,
    normalizer: Normalizer,
    bp: Biquad,
    pca: Pca,
    seed: u64,
    link: u8,
    /// Band-passed mean amplitude, accumulated over a report window.
    filtered_window: Vec<f32>,
    /// Raw mean amplitude, accumulated over a report window.
    amp_window: Vec<f32>,
}

impl RxDsp {
    /// `rate_hz` is the measurement frame rate (used for the band-pass centre
    /// and the dominant-frequency conversion); `link` is `node::RX1`/`RX2`.
    pub fn new(rate_hz: f32, link: u8, seed: u64) -> Self {
        // A default empty-room baseline (mean 0, std 1) turns raw amps into
        // ~z-scores; scale is floored so nothing divides by zero.
        let baseline = radar_calibration::BaselineStats::default();
        let normalizer = baseline.normalizer();
        // Human-motion band-pass centered on ~1 Hz. fc = 1 Hz / frame rate.
        let bp = Biquad::bandpass(0.005, 1.0); // 0.005 = 1 Hz at 200 Hz
        // PCA over 8 subband-energy dims, 16-frame history, 2 components.
        let pca = Pca::new(8, 16, 2);
        Self {
            sample_rate_hz: rate_hz.max(1.0),
            normalizer,
            bp,
            pca,
            seed,
            link,
            filtered_window: Vec::with_capacity(REPORT_WINDOW),
            amp_window: Vec::with_capacity(REPORT_WINDOW),
        }
    }

    /// Synthesise the channel that would have been observed for TX `seq`, using
    /// the real `tx_power_db` from the DataFrame payload (higher power → higher
    /// received amplitude) and the link id for per-link noise.
    pub fn synth_channel(&self, seq: u32, tx_power_db: u8) -> Channel {
        // Shared motion envelope: ~1 Hz (period 200 frames at 200 Hz). Both
        // links see exactly the same envelope because it depends only on seq.
        let motion = (seq as f32 * core::f32::consts::TAU / 200.0).sin();
        // A secondary slower drift (period ~3.3 s) so the energy envelope has
        // structure over the 8 s run.
        let drift = (seq as f32 * core::f32::consts::TAU / 660.0) * 0.4;
        let base_amp = 70.0 + tx_power_db as f32 * 1.1 + (motion + drift) * 25.0;

        let mut raw = [0i8; RAW_BUF];
        for bin in 0..64usize {
            let sub = bin as f32 - 32.0; // -32..+31
            let sub_pattern = 1.0 + 0.25 * (sub * 0.25).sin();
            // Per-link noise: ~±4% amplitude, ±0.15 rad phase.
            let amp_noise = hash01(self.seed, seq, self.link, 1) * 0.04;
            let phase_noise = hash01(self.seed, seq, self.link, 2) * 0.15;
            let amp = base_amp * sub_pattern * (1.0 + amp_noise);
            let phase = sub * 0.12 + motion * 0.4 + phase_noise;
            raw[bin * 2] = (amp * phase.cos()) as i8;
            raw[bin * 2 + 1] = (amp * phase.sin()) as i8;
        }

        let rssi = -55i16 - (tx_power_db as i16 / 3);
        decode_channel(&raw, false, rssi, -100)
    }

    /// Feed one frame's channel through the pipeline. Returns the per-frame
    /// band-passed mean amplitude (aggregated for the report).
    pub fn process_frame(&mut self, ch: &Channel) -> f32 {
        // Baseline-normalised amplitudes (z-scores).
        let z = self.normalizer.normalize(&ch.amps);
        // 8 subband energies as the (cheap) PCA input.
        let mut subbands = [0f32; 8];
        for (s, v) in subbands.iter_mut().enumerate() {
            let lo = s * 7;
            let hi = (lo + 7).min(N_SUB);
            let sum: f32 = z[lo..hi].iter().map(|x| x * x).sum();
            *v = sum / (hi - lo) as f32;
        }
        self.pca.update(&subbands);

        // Band-pass the mean amplitude; the filtered series feeds motion energy.
        let mean_amp = ch.mean_amp();
        let filtered = self.bp.process(mean_amp);
        self.filtered_window.push(filtered);
        self.amp_window.push(mean_amp);
        filtered
    }

    /// Consume the accumulated window and emit the metrics for a FeatureReport.
    pub fn finish_window(&mut self, seq: u32) -> radar_protocol::FeatureReport {
        let f = core::mem::take(&mut self.filtered_window);
        let a = core::mem::take(&mut self.amp_window);

        let motion_energy = energy(&f);
        let amp_mean = if a.is_empty() { 0.0 } else { a.iter().sum::<f32>() / a.len() as f32 };
        let centered: Vec<f32> = a.iter().map(|x| x - amp_mean).collect();
        let amp_std = rms(&centered);

        // Power spectrum of the band-passed window for spectral metrics.
        let spec = radar_dsp::fft::power_spectrum(&f);
        let spectral_entropy = radar_dsp::metrics::spectral_entropy(&spec);
        let dominant = dominant_freq_hz(&spec, self.sample_rate_hz);

        // Phase dispersion from a freshly synthesised channel's sanitised phase.
        let ch = self.synth_channel(seq, 20);
        let phase_dispersion = circular_variance(&ch.phase);

        // Static-presence indicator: how far the current mean amp sits from the
        // baseline mean (0 in z-space), in relative units.
        let baseline_dev = amp_mean.abs() / 100.0;

        let subbands = subband_energy(&ch, &self.normalizer);
        let scores = self.pca.project(&subbands);
        let mut pca_scores = [0f32; 8];
        for (o, &s) in pca_scores.iter_mut().zip(scores.iter().take(8)) {
            *o = s;
        }

        radar_protocol::FeatureReport {
            seq,
            n_frames: REPORT_WINDOW as u32,
            n_missing: 0,
            rssi: ch.rssi,
            snr: 22,
            csi_quality: 80,
            sat_score: 0,
            dyn_range: 70,
            flags: 0,
            amp_mean,
            amp_std,
            motion_energy,
            spectral_entropy,
            dominant_freq_hz: dominant,
            phase_dispersion,
            baseline_dev,
            pca_scores,
        }
    }

    /// Build a low-rate CsiSnapshot for the last TX `seq`.
    pub fn synth_snapshot(&self, seq: u32) -> radar_protocol::CsiSnapshot {
        let ch = self.synth_channel(seq, 20);
        let mut iq = [0i16; radar_protocol::N_SUBCARRIERS * 2];
        let mut amp_norm = [0u8; radar_protocol::N_SUBCARRIERS];
        for (k, a) in ch.amps.iter().enumerate() {
            iq[2 * k] = (a * 0.9) as i16;
            iq[2 * k + 1] = (a * 0.15) as i16;
            let v = (128.0 + (a - 70.0) * 2.5) as i32;
            amp_norm[k] = v.clamp(0, 255) as u8;
        }
        let mut spec = [0u8; radar_protocol::N_SPEC_BINS];
        // A slowly moving Gaussian peak in the motion spectrum.
        let dom_bin = (16.0 + 10.0 * (seq as f32 * core::f32::consts::TAU / 660.0).sin()).clamp(1.0, 62.0);
        for (b, v) in spec.iter_mut().enumerate() {
            let d = b as f32 - dom_bin;
            let g = (d * d / 5.0).exp();
            *v = (30.0 + 200.0 * g).clamp(0.0, 255.0) as u8;
        }
        radar_protocol::CsiSnapshot {
            seq,
            rssi: ch.rssi,
            snr: 22,
            csi_quality: 80,
            noise_floor: -100.0,
            flags: 0,
            n_sub: radar_protocol::N_SUBCARRIERS as u8,
            reserved: 0,
            iq,
            amp_norm,
            spec,
        }
    }
}

fn subband_energy(ch: &Channel, normalizer: &Normalizer) -> Vec<f32> {
    let z = normalizer.normalize(&ch.amps);
    (0..8)
        .map(|s| {
            let lo = s * 7;
            let hi = (lo + 7).min(N_SUB);
            let sum: f32 = z[lo..hi].iter().map(|x| x * x).sum();
            sum / (hi - lo) as f32
        })
        .collect()
}
