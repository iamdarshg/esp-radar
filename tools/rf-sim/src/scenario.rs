//! RF scenario model + simdata blob generator.
//!
//! The generator turns a [`Scenario`] (target trajectory, multipath, CFO,
//! noise, amplitude) into a burnable `simdata` blob in EXACTLY the format
//! `firmware/radar_rx/src/sim.rs` reads back:
//!
//! ```text
//! 32-byte header (magic "SIMD", version, channel, rate_hz, n_frames, rssi,
//!                noise_floor, bssid, cwb, sig_mode, frame_len=128) +
//! n_frames × 128-byte ESP32 HT20 raw CSI records
//!                (interleaved i8 I/Q, bin = subcarrier + 32, valid
//!                subcarriers at bins 4..31 and 33..60, guards/DC zeroed)
//! ```
//!
//! The channel model is a sum of propagation paths — a dominant moving
//! target plus static reflectors — rotated by a common CFO:
//!
//! ```text
//! H_k(t) = e^{j2π·f_cfo·t} · Σ_i a_i · e^{-j4π·f_k·r_i(t)/c}
//! ```
//!
//! with `r_i(t) = delay_m_i + v_i·t` (one-way range; `v` positive = receding).
//! For a clean single-path scenario this reduces to a phase-coherent CW radar:
//! `φ_k(t) = 2π·f_cfo·t − 4π·f_k·r(t)/c`, so a displacement Δr shifts the
//! phase by Δφ = −4π·f·Δr/c (round-trip). The per-subcarrier index order
//! matches `radar_dsp::transform::valid_bins`: k 0..27 → subcarrier −28..−1,
//! k 28..55 → subcarrier +1..+28.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::rng::Rng;

/// Valid HT20 subcarriers kept by the radar (`radar_dsp::transform::valid_bins`).
pub const N_SUBC: usize = 56;
/// Simdata blob magic ("SIMD"), matching `sim.rs`.
pub const SIM_MAGIC: u32 = 0x444D_4953;
/// Simdata blob schema version, matching `sim.rs`.
pub const SIM_VERSION: u8 = 1;
/// Fixed header bytes before the frame records.
pub const SIM_HEADER_LEN: usize = 32;
/// Raw CSI buffer length per frame (64 FFT bins × interleaved i8 I/Q).
pub const SIM_FRAME_LEN: usize = 128;

pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Target displacement as a function of time (seconds). The STATIC range is
/// held by the target path's `delay_m`; the trajectory supplies the DYNAMIC
/// part, so the differential quantities the analyzer measures are
/// delay-independent.
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Trajectory {
    /// Constant radial velocity. `v_m_s` positive = receding (range growing).
    ConstVel { v_m_s: f64 },
    /// Sinusoidal displacement (breathing / sway):
    /// `r(t) = delay + amp·sin(2π·f·t + φ0)`.
    Sin { amp_m: f64, f_hz: f64, phase0_rad: f64 },
    /// Static target (e.g. CFO-only scenarios).
    Static,
}

/// One propagation path. `delay_m` is the one-way path length (sets the static
/// per-subcarrier phase slope); `v_m_s` its radial velocity (only the target
/// path moves). `amp` is relative to the other paths.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PropPath {
    pub amp: f64,
    pub delay_m: f64,
    pub v_m_s: f64,
}

fn default_seed() -> u64 {
    0xC0FFEE
}

/// A fully-specified RF scenario. The JSON form is the analyzer's ground
/// truth: every derived metric is recomputed from these parameters.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Scenario {
    pub version: u32,
    pub name: String,
    /// Intended frame rate (Hz) — the firmware's `fs`.
    pub rate_hz: u32,
    /// Number of raw CSI frames in the blob.
    pub n_frames: u32,
    /// Carrier frequency (Hz). Channel 6 centre = 2.437 GHz.
    pub fc_hz: f64,
    /// Subcarrier spacing (Hz). HT20 = 312.5 kHz.
    pub sub_spacing_hz: f64,
    /// Nominal per-subcarrier IQ amplitude after per-frame peak normalization.
    /// Keep ≤ ~120 for i8 headroom (the phase floor is 0.2887/A rad).
    pub amp: f64,
    /// Per-subcarrier SNR (dB) of the additive complex-Gaussian noise.
    pub snr_db: f64,
    /// Carrier-frequency offset (Hz): a common phase-rate bias on all
    /// subcarriers. It is a DC term in the per-frame phase rate, so a static
    /// CFO aliases to a whole-velocity bias; |f_cfo + f_d| > rate/2 wraps.
    pub cfo_hz: f64,
    pub rssi_dbm: i16,
    pub noise_floor_dbm: i16,
    pub channel: u8,
    /// AP BSSID the scenario emulates (the firmware's ring filter).
    pub bssid: [u8; 6],
    pub trajectory: Trajectory,
    /// Multipath: the dominant (moving) path at `target_path`, plus static
    /// reflectors. Clean LoS = a single path.
    pub paths: Vec<PropPath>,
    pub target_path: usize,
    /// PRNG seed for the additive noise (deterministic blob output).
    #[serde(default = "default_seed")]
    pub seed: u64,
}

impl Scenario {
    /// Static + dynamic target range at scenario time `t` (seconds).
    pub fn target_range(&self, t: f64) -> f64 {
        let dc = self
            .paths
            .get(self.target_path)
            .map(|p| p.delay_m)
            .unwrap_or(0.0);
        dc + match &self.trajectory {
            Trajectory::ConstVel { v_m_s } => v_m_s * t,
            Trajectory::Sin {
                amp_m,
                f_hz,
                phase0_rad,
            } => amp_m * (std::f64::consts::TAU * f_hz * t + phase0_rad).sin(),
            Trajectory::Static => 0.0,
        }
    }

    /// Target radial velocity (d range/dt) at scenario time `t`.
    pub fn target_velocity(&self, t: f64) -> f64 {
        match &self.trajectory {
            Trajectory::ConstVel { v_m_s } => *v_m_s,
            Trajectory::Sin {
                amp_m,
                f_hz,
                phase0_rad,
            } => std::f64::consts::TAU
                * f_hz
                * amp_m
                * (std::f64::consts::TAU * f_hz * t + phase0_rad).cos(),
            Trajectory::Static => 0.0,
        }
    }

    /// True Doppler shift of the target: f_d = −2·f_c·v/c (positive when the
    /// phase rate rises, i.e. approaching). Constant-velocity Doppler is a DC
    /// phase-rate term, indistinguishable from CFO on a single RX without a
    /// separate CFO estimate.
    pub fn target_doppler_hz(&self, t: f64) -> f64 {
        -2.0 * self.fc_hz * self.target_velocity(t) / SPEED_OF_LIGHT
    }

    /// Per-subcarrier analytic phase-error floor: σ_n + quantization.
    /// σ_n = A/√(2·SNR) (a complex sample with amplitude A and per-component
    /// noise σ has SNR = A²/2σ²), σ_q = 1/√12 LSB; both divide by A to give
    /// radians. This is the CRB for one subcarrier's phase measurement.
    pub fn sigma_phi_floor_rad(&self) -> f64 {
        let snr_lin = 10f64.powf(self.snr_db / 10.0);
        let sigma_n = self.amp / (2.0 * snr_lin).sqrt();
        let sigma_q = 1.0 / 12f64.sqrt();
        (sigma_n * sigma_n + sigma_q * sigma_q).sqrt() / self.amp
    }

    /// Combined Δφ floor: N independent per-subcarrier phase measurements
    /// coherently combined by the conjugate-product / atan2 sum give a
    /// √N improvement (uniform amplitude → equal weights).
    pub fn sigma_dphi_floor_rad(&self) -> f64 {
        self.sigma_phi_floor_rad() / (N_SUBC as f64).sqrt()
    }

    /// Subcarrier number (relative to DC) of analyzer index `k`, matching
    /// `radar_dsp::transform::valid_bins`.
    pub fn subcarrier_of(k: usize) -> i32 {
        if k < 28 {
            k as i32 - 28
        } else {
            k as i32 - 27
        }
    }

    /// Frequency (Hz) of analyzer index `k`.
    pub fn subcarrier_hz(&self, k: usize) -> f64 {
        self.fc_hz + Self::subcarrier_of(k) as f64 * self.sub_spacing_hz
    }

    /// Raw (pre-normalization, pre-noise) complex channel for subcarrier `k`
    /// at scenario time `t`: the multipath sum rotated by the common CFO.
    /// Returns `(re, im)`.
    pub fn channel(&self, t: f64, k: usize) -> (f64, f64) {
        let f_k = self.subcarrier_hz(k);
        let mut re = 0.0;
        let mut im = 0.0;
        // NOTE: `f64::sin_cos` returns (sin, cos) — bind as (s, c)!
        for p in &self.paths {
            let r = p.delay_m + p.v_m_s * t;
            let ph = -4.0 * std::f64::consts::PI * f_k * r / SPEED_OF_LIGHT;
            let (s, c) = ph.sin_cos();
            re += p.amp * c;
            im += p.amp * s;
        }
        let ph_cfo = std::f64::consts::TAU * self.cfo_hz * t;
        let (s, c) = ph_cfo.sin_cos();
        (re * c - im * s, re * s + im * c)
    }

    /// Clean (noise-free) per-subcarrier phase at scenario frame `n`, radians,
    /// in `(-π, π]`. This is the analyzer's ground truth for frame `n`.
    pub fn phase_clean(&self, n: u64, k: usize) -> f64 {
        let t = n as f64 / self.rate_hz as f64;
        let (re, im) = self.channel(t, k);
        im.atan2(re)
    }
}

/// FFT bin for a subcarrier index (`bin = subcarrier + 32`), matching the
/// ESP32 HT20 CSI layout.
fn bin_of(sub: i32) -> usize {
    (sub + 32) as usize
}

fn clamp_i8(x: f64) -> u8 {
    x.round().clamp(-128.0, 127.0) as i8 as u8
}

/// Write the 32-byte simdata header.
fn write_header(out: &mut [u8], sc: &Scenario) {
    out[0..4].copy_from_slice(&SIM_MAGIC.to_le_bytes());
    out[4] = SIM_VERSION;
    out[5] = sc.channel;
    out[6] = 0;
    out[7] = 0;
    out[8..12].copy_from_slice(&sc.rate_hz.to_le_bytes());
    out[12..16].copy_from_slice(&sc.n_frames.to_le_bytes());
    out[16..18].copy_from_slice(&sc.rssi_dbm.to_le_bytes());
    out[18..20].copy_from_slice(&sc.noise_floor_dbm.to_le_bytes());
    out[20..26].copy_from_slice(&sc.bssid);
    out[26] = 0; // cwb = 20 MHz (metadata; decode does not use it)
    out[27] = 1; // sig_mode = HT (metadata)
    out[28..30].copy_from_slice(&(SIM_FRAME_LEN as u16).to_le_bytes());
    out[30] = 0;
    out[31] = 0;
}

/// Render one raw CSI frame (128 B) for scenario frame `n` into `buf`.
fn write_frame(sc: &Scenario, n: u32, rng: &mut Rng, buf: &mut [u8]) {
    buf.fill(0);
    let t = n as f64 / sc.rate_hz as f64;

    // Per-frame peak normalization keeps the strongest subcarrier at `amp`
    // (phase-preserving; the DSP's amplitude weighting then sees a uniform
    // envelope, so its conjugate-product combine ≈ equal-weight).
    let mut mag_max = 0.0f64;
    let mut chans = [(0.0f64, 0.0f64); N_SUBC];
    for k in 0..N_SUBC {
        let (re, im) = sc.channel(t, k);
        let m = (re * re + im * im).sqrt();
        if m > mag_max {
            mag_max = m;
        }
        chans[k] = (re, im);
    }
    let scale = sc.amp / mag_max.max(1e-9);

    let sigma_n = sc.amp / (2.0 * 10f64.powf(sc.snr_db / 10.0)).sqrt();

    for k in 0..N_SUBC {
        let (re, im) = chans[k];
        let b = bin_of(Scenario::subcarrier_of(k)) * 2;
        buf[b] = clamp_i8(scale * re + rng.gauss() * sigma_n);
        buf[b + 1] = clamp_i8(scale * im + rng.gauss() * sigma_n);
    }
}

/// Generate the simdata blob for `scenario` and write it to `path`.
pub fn generate(sc: &Scenario, path: &Path) -> std::io::Result<()> {
    let mut rng = Rng::new(sc.seed);
    let mut header = [0u8; SIM_HEADER_LEN];
    write_header(&mut header, sc);

    let mut out = BufWriter::new(File::create(path)?);
    out.write_all(&header)?;
    let mut frame = [0u8; SIM_FRAME_LEN];
    for n in 0..sc.n_frames {
        write_frame(sc, n, &mut rng, &mut frame);
        out.write_all(&frame)?;
    }
    out.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lo_scenario() -> Scenario {
        Scenario {
            version: 1,
            name: "test-los".into(),
            rate_hz: 200,
            n_frames: 100,
            fc_hz: 2.437e9,
            sub_spacing_hz: 312_500.0,
            amp: 100.0,
            snr_db: 30.0,
            cfo_hz: 50.0,
            rssi_dbm: -52,
            noise_floor_dbm: -96,
            channel: 6,
            bssid: [36, 15, 40, 1, 2, 3],
            trajectory: Trajectory::ConstVel { v_m_s: 0.35 },
            paths: vec![PropPath {
                amp: 1.0,
                delay_m: 2.0,
                v_m_s: 0.35,
            }],
            target_path: 0,
            seed: 7,
        }
    }

    #[test]
    fn single_path_channel_phase_matches_formula() {
        let sc = lo_scenario();
        let t = 1.5;
        for k in [0usize, 13, 27, 28, 55] {
            let f_k = sc.subcarrier_hz(k);
            let r = 2.0 + 0.35 * t;
            // The analytic (unwrapped) phase; `got` is wrapped, so compare the
            // wrapped residual.
            let expected = std::f64::consts::TAU * sc.cfo_hz * t
                - 4.0 * std::f64::consts::PI * f_k * r / SPEED_OF_LIGHT;
            let (re, im) = sc.channel(t, k);
            let got = im.atan2(re);
            let d = wrap(got - expected);
            assert!(d.abs() < 1e-9, "k={k}: {got} vs {expected}");
        }
    }

    #[test]
    fn clean_phase_moves_with_displacement() {
        let sc = lo_scenario();
        let k = 28; // subcarrier +1
        let p0 = sc.phase_clean(100, k);
        let p1 = sc.phase_clean(101, k);
        // Phase advance over one frame = CFO term + round-trip target term.
        let f_k = sc.subcarrier_hz(k);
        let dr = 0.35 / 200.0; // Δr = v/rate
        let target_term = -4.0 * std::f64::consts::PI * f_k * dr / SPEED_OF_LIGHT;
        let cfo_term = std::f64::consts::TAU * sc.cfo_hz / sc.rate_hz as f64;
        let expected = cfo_term + target_term;
        assert!(expected.abs() < std::f64::consts::PI, "expected wraps: {expected}");
        let d = wrap(p1 - p0) - expected;
        assert!(d.abs() < 1e-6, "dphi {} vs {expected}", wrap(p1 - p0));
    }

    fn wrap(x: f64) -> f64 {
        x - std::f64::consts::TAU * (x / std::f64::consts::TAU).round()
    }
}

