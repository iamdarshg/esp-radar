//! Iterative radix-2 complex FFT (self-contained, no external DSP dependency)
//! plus spectrum helpers.

use crate::Complex;

/// In-place forward FFT. `buf.len()` must be a power of two.
pub fn fft_inplace(buf: &mut [Complex]) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two(), "FFT length must be a power of two");
    if n < 2 {
        return;
    }

    // Bit-reversal permutation.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Cooley-Tukey butterflies.
    let mut len = 2usize;
    while len <= n {
        let ang = -2.0 * core::f32::consts::PI / len as f32;
        let wlen = Complex::from_polar(1.0, ang);
        let half = len >> 1;
        let mut i = 0usize;
        while i < n {
            let mut w = Complex::new(1.0, 0.0);
            for k in 0..half {
                let u = buf[i + k];
                let v = buf[i + k + half] * w;
                buf[i + k] = u + v;
                buf[i + k + half] = u - v;
                w = w * wlen;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// In-place inverse FFT (scaled).
pub fn ifft_inplace(buf: &mut [Complex]) {
    let n = buf.len();
    for c in buf.iter_mut() {
        c.im = -c.im;
    }
    fft_inplace(buf);
    let scale = 1.0 / n as f32;
    for c in buf.iter_mut() {
        c.re *= scale;
        c.im = -c.im * scale;
    }
}

/// Magnitude spectrum (first `n/2` bins).
pub fn magnitude_spectrum(signal: &[f32]) -> Vec<f32> {
    let n = next_pow2(signal.len());
    let mut buf: Vec<Complex> = signal
        .iter()
        .map(|&x| Complex::new(x, 0.0))
        .collect();
    buf.resize(n, Complex::default());
    fft_inplace(&mut buf);
    buf[..n / 2].iter().map(|c| c.mag()).collect()
}

/// Power spectrum (|X|^2), first `n/2` bins.
pub fn power_spectrum(signal: &[f32]) -> Vec<f32> {
    let n = next_pow2(signal.len());
    let mut buf: Vec<Complex> = signal
        .iter()
        .map(|&x| Complex::new(x, 0.0))
        .collect();
    buf.resize(n, Complex::default());
    fft_inplace(&mut buf);
    buf[..n / 2]
        .iter()
        .map(|c| c.re * c.re + c.im * c.im)
        .collect()
}

/// Short-time Fourier transform producing one spectrum frame.
///
/// `frame` is zero-padded/truncated to `fft_len`. Returns the magnitude
/// spectrum of length `fft_len / 2`.
pub fn stft_frame(frame: &[f32], fft_len: usize) -> Vec<f32> {
    debug_assert!(fft_len.is_power_of_two());
    let mut buf: Vec<Complex> = frame
        .iter()
        .take(fft_len)
        .map(|&x| Complex::new(x, 0.0))
        .collect();
    buf.resize(fft_len, Complex::default());
    fft_inplace(&mut buf);
    buf[..fft_len / 2].iter().map(|c| c.mag()).collect()
}

/// Next power of two >= n.
pub fn next_pow2(n: usize) -> usize {
    let mut p = 1usize;
    while p < n {
        p <<= 1;
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn fft_of_dc_signal() {
        // Constant signal → energy concentrated in bin 0.
        let mut buf: Vec<Complex> = vec![Complex::new(1.0, 0.0); 8];
        fft_inplace(&mut buf);
        assert!(approx(buf[0].re, 8.0, 1e-4), "bin0 real = {}", buf[0].re);
        assert!(buf[0].im.abs() < 1e-4);
        for c in &buf[1..] {
            assert!(c.mag() < 1e-4, "leakage into other bins");
        }
    }

    #[test]
    fn fft_roundtrip() {
        let mut buf: Vec<Complex> = (0..16)
            .map(|i| Complex::new((i as f32).sin(), (i as f32) * 0.01))
            .collect();
        let orig = buf.clone();
        fft_inplace(&mut buf);
        ifft_inplace(&mut buf);
        for (a, b) in buf.iter().zip(orig.iter()) {
            assert!(approx(a.re, b.re, 1e-3));
            assert!(approx(a.im, b.im, 1e-3));
        }
    }

    #[test]
    fn spectrum_dominant_bin() {
        // A pure tone at 2 cycles across 64 samples → peak at bin 2.
        let signal: Vec<f32> = (0..64).map(|i| ((i as f32) * 2.0 * core::f32::consts::PI * 2.0 / 64.0).sin()).collect();
        let spec = power_spectrum(&signal);
        let peak = (1..spec.len()).max_by(|&a, &b| spec[a].partial_cmp(&spec[b]).unwrap()).unwrap();
        assert_eq!(peak, 2);
    }
}
