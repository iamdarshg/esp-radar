//! Spectral and temporal metrics computed from filtered CSI time series.

/// Normalized spectral entropy of a power spectrum (0..1). 0 = single-tone,
/// 1 = uniform/white. Used to separate rhythmic motion from broadband noise.
pub fn spectral_entropy(power: &[f32]) -> f32 {
    let total: f32 = power.iter().sum();
    if total <= 1e-9 {
        return 1.0; // no energy → maximally uncertain
    }
    let n = power.len() as f32;
    let mut h = 0.0f32;
    for &p in power {
        let prob = (p / total).max(1e-12);
        h -= prob * prob.log2();
    }
    (h / n.log2()).clamp(0.0, 1.0)
}

/// Frequency of the dominant spectral peak, in Hz.
pub fn dominant_freq_hz(power: &[f32], sample_rate_hz: f32) -> f32 {
    let n = power.len();
    if n == 0 {
        return 0.0;
    }
    // Skip DC (bin 0) — a static baseline has huge DC energy we don't want to
    // report as "motion".
    let mut best = 1usize;
    let mut best_p = power[1.min(n - 1)];
    for (i, &p) in power.iter().enumerate().skip(1) {
        if p > best_p {
            best_p = p;
            best = i;
        }
    }
    // Bin width: n bins cover 0 .. sample_rate/2, so bin i => i*(fs/2)/n.
    let bin_width = sample_rate_hz * 0.5 / n as f32;
    best as f32 * bin_width
}

/// Signal energy in the motion band. Feed it the band-pass-filtered series and
/// the sum of squares is the "motion energy" (spec §6).
pub fn energy(filtered: &[f32]) -> f32 {
    filtered.iter().map(|x| x * x).sum()
}

/// RMS of a series.
pub fn rms(x: &[f32]) -> f32 {
    if x.is_empty() {
        return 0.0;
    }
    (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
}

/// Circular variance of phases (0..1). Used to report how spread the
/// subcarrier phase is: low = coherent, high = diffuse multipath.
pub fn circular_variance(phases: &[f32]) -> f32 {
    let n = phases.len() as f32;
    if n == 0.0 {
        return 1.0;
    }
    let (mut sx, mut sy) = (0.0f32, 0.0f32);
    for &p in phases {
        sx += p.cos();
        sy += p.sin();
    }
    let r = (sx * sx + sy * sy).sqrt() / n;
    (1.0 - r).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entropy_zero_for_single_tone() {
        let mut power = vec![0.0f32; 32];
        power[8] = 100.0;
        let e = spectral_entropy(&power);
        assert!(e < 0.05, "single tone entropy = {e}");
    }

    #[test]
    fn entropy_high_for_uniform() {
        let power = vec![1.0f32; 32];
        let e = spectral_entropy(&power);
        assert!(e > 0.95, "uniform entropy = {e}");
    }

    #[test]
    fn dominant_freq_peak() {
        // 32-bin spectrum, fs = 64 Hz -> bin width = 1 Hz. Peak at bin 3 => 3 Hz.
        let mut power = vec![0.0f32; 32];
        power[3] = 50.0;
        let f = dominant_freq_hz(&power, 64.0);
        assert!((f - 3.0).abs() < 0.01, "dominant freq = {f}");
    }

    #[test]
    fn circular_variance_bounds() {
        let coherent = [0.1, 0.12, 0.09, 0.11];
        let diffuse = [0.0, 1.5, 3.0, 4.6];
        let vc = circular_variance(&coherent);
        let vd = circular_variance(&diffuse);
        assert!(vc < 0.05, "coherent variance = {vc}");
        assert!(vd > 0.5, "diffuse variance = {vd}");
    }
}
