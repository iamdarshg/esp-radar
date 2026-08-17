//! CSI signal processing for the compact radar head.
//!
//! Implements the progressive pipeline from spec §16:
//!
//! ```text
//! raw complex CSI
//!   → valid-subcarrier selection
//!   → amplitude / phase
//!   → normalization (per-subcarrier baseline)
//!   → baseline subtraction
//!   → outlier removal
//!   → temporal filtering (band-pass, human-motion band)
//!   → PCA
//!   → STFT
//!   → per-link features
//! ```
//!
//! This crate is pure Rust — no ESP dependencies — so the entire pipeline can
//! be unit-tested on the host and re-used verbatim in `tools/replay`.

pub mod fft;
pub mod filter;
pub mod metrics;
pub mod pca;
pub mod transform;

/// Number of valid HT20 subcarriers used by the radar: the 56 occupied bins
/// (-28..-1 and +1..+28), excluding DC and the guard bands.
pub const N_SUBCARRIERS: usize = 56;

/// A complex channel coefficient.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    pub fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }
    pub fn from_polar(r: f32, theta: f32) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }
    pub fn mag(self) -> f32 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
    pub fn phase(self) -> f32 {
        self.im.atan2(self.re)
    }
}

impl core::ops::Add for Complex {
    type Output = Complex;
    fn add(self, o: Complex) -> Complex {
        Complex::new(self.re + o.re, self.im + o.im)
    }
}
impl core::ops::Sub for Complex {
    type Output = Complex;
    fn sub(self, o: Complex) -> Complex {
        Complex::new(self.re - o.re, self.im - o.im)
    }
}
impl core::ops::Mul for Complex {
    type Output = Complex;
    fn mul(self, o: Complex) -> Complex {
        Complex::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

/// One decoded CSI channel observation over the active subcarriers.
#[derive(Clone, Debug)]
pub struct Channel {
    /// Amplitude per active subcarrier.
    pub amps: [f32; N_SUBCARRIERS],
    /// Sanitized phase per active subcarrier (linear slope removed).
    pub phase: [f32; N_SUBCARRIERS],
    pub rssi: i16,
    pub noise_floor: i16,
    pub valid: bool,
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            amps: [0.0; N_SUBCARRIERS],
            phase: [0.0; N_SUBCARRIERS],
            rssi: 0,
            noise_floor: 0,
            valid: false,
        }
    }
}

impl Channel {
    pub fn mean_amp(&self) -> f32 {
        self.amps.iter().sum::<f32>() / N_SUBCARRIERS as f32
    }
    pub fn std_amp(&self) -> f32 {
        let m = self.mean_amp();
        let v = self.amps.iter().map(|a| (a - m) * (a - m)).sum::<f32>() / N_SUBCARRIERS as f32;
        v.sqrt()
    }
    /// Per-subcarrier normalized amplitude (0..1-ish) for waterfall display.
    pub fn normalized_amps(&self, lo: f32, hi: f32) -> [f32; N_SUBCARRIERS] {
        let span = (hi - lo).max(1e-6);
        let mut out = [0.0f32; N_SUBCARRIERS];
        for (o, &a) in out.iter_mut().zip(self.amps.iter()) {
            *o = ((a - lo) / span).clamp(0.0, 1.0);
        }
        out
    }
}
