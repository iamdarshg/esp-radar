//! Biquad (RBJ cookbook) filters for temporal filtering of CSI-derived time
//! series. The human-motion band (~0.2 Hz – 5 Hz) is what survives the
//! band-pass, while slow baseline drift and fast Wi-Fi noise are attenuated.

/// Second-order IIR section (Direct Form I).
#[derive(Clone, Debug)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    pub fn new(b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) -> Self {
        Self { b0, b1, b2, a1, a2, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }

    /// Low-pass with normalized cutoff `fc` (0..0.5 of sample rate) and
    /// Butterworth Q (0.7071).
    pub fn lowpass(fc: f32) -> Self {
        let q = 0.7071_f32;
        Self::design(1, fc, q)
    }

    /// High-pass with normalized cutoff `fc`.
    pub fn highpass(fc: f32) -> Self {
        let q = 0.7071_f32;
        Self::design(2, fc, q)
    }

    /// Band-pass with normalized centre `fc` and bandwidth Q.
    pub fn bandpass(fc: f32, q: f32) -> Self {
        Self::design(1, fc, q) // RBJ bandpass with Q
    }

    /// Design a biquad. `kind`: 1=lowpass, 2=highpass, 3=bandpass.
    fn design(kind: u8, fc: f32, q: f32) -> Self {
        let w0 = core::f32::consts::PI * 2.0 * fc.clamp(0.001, 0.49);
        let cos_w0 = w0.cos();
        let sin_w0 = w0.sin();
        let alpha = sin_w0 / (2.0 * q);

        let (b0, b1, b2, a0, a1, a2) = match kind {
            1 => {
                // Low-pass (RBJ cookbook; note the 1/2 on b0/b2 so the DC gain
                // is exactly 1.0 after the a0 normalization below).
                (
                    0.5 * (1.0 - cos_w0),
                    1.0 - cos_w0,
                    0.5 * (1.0 - cos_w0),
                    1.0 + alpha,
                    -2.0 * cos_w0,
                    1.0 - alpha,
                )
            }
            2 => {
                // High-pass.
                (
                    0.5 * (1.0 + cos_w0),
                    -(1.0 + cos_w0),
                    0.5 * (1.0 + cos_w0),
                    1.0 + alpha,
                    -2.0 * cos_w0,
                    1.0 - alpha,
                )
            }
            _ => {
                // Band-pass (constant 0 dB peak gain)
                (
                    alpha,
                    0.0,
                    -alpha,
                    1.0 + alpha,
                    -2.0 * cos_w0,
                    1.0 - alpha,
                )
            }
        };

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Process one sample.
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

/// Simple median-of-5 outlier/spike remover for a per-subcarrier time series.
/// Replaces a sample that deviates from its neighbours by more than `threshold`
/// (relative) with the window median.
pub fn median5(x: [f32; 5]) -> f32 {
    let mut v = x;
    // Partial insertion sort for median (order 3 of 5).
    for i in 1..5 {
        let mut j = i;
        while j > 0 && v[j] < v[j - 1] {
            v.swap(j, j - 1);
            j -= 1;
        }
    }
    v[2]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lowpass_removes_high_frequency() {
        let mut lp = Biquad::lowpass(0.05); // fc = 0.05 * fs (normalized to Nyquist)
        // Feed DC + a high-frequency tone (0.4 fs). The tone should be
        // strongly attenuated while the DC path settles to 1.0.
        let mut out: Vec<f32> = Vec::new();
        for i in 0..2000 {
            let t = i as f32;
            let x = 1.0 + (t * core::f32::consts::PI * 2.0 * 0.4).sin();
            out.push(lp.process(x));
        }
        // Skip the transient; measure the settled tail.
        let tail = &out[1800..];
        let mean = tail.iter().sum::<f32>() / tail.len() as f32;
        // Mean should be near the DC level (1.0), not shifted by the tone.
        assert!((mean - 1.0).abs() < 0.15, "tail mean {mean}");
        // And the residual AC ripple (the attenuated 0.4 fs tone) must be
        // small relative to the DC level.
        let ripple: f32 = tail.iter().map(|x| (x - mean) * (x - mean)).sum::<f32>() / tail.len() as f32;
        assert!(ripple.sqrt() < 0.1, "ripple rms {}", ripple.sqrt());
    }

    #[test]
    fn highpass_removes_dc() {
        let mut hp = Biquad::highpass(0.01);
        let mut out: Vec<f32> = Vec::new();
        for i in 0..2000 {
            let x = 3.0 + (i as f32 * 0.1).sin(); // DC + slow oscillation
            out.push(hp.process(x));
        }
        let tail = &out[1800..];
        let mean = tail.iter().sum::<f32>() / tail.len() as f32;
        // DC must be blocked → output mean ≈ 0.
        assert!(mean.abs() < 0.05, "highpass DC leakage {mean}");
    }

    #[test]
    fn median_removes_spike() {
        let x = [1.0, 1.0, 100.0, 1.0, 1.0];
        let m = median5(x);
        assert!((m - 1.0).abs() < 1e-6);
    }
}
