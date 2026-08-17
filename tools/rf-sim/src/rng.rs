//! Deterministic seeded PRNG for the scenario generator.
//!
//! A xorshift64* stream (avalanched with SplitMix64 at seed time) plus a
//! Box–Muller transform for Gaussian noise. The generator is fully
//! reproducible from the scenario JSON alone (`seed` field): the exact IQ
//! samples written into the simdata blob are identical across runs, so two
//! QEMU runs of the same scenario produce bit-identical noise and the only
//! variance left is the firmware/DSP path under test.

/// Deterministic pseudo-random stream.
pub struct Rng(u64);

impl Rng {
    /// Seed from a `u64`. SplitMix64 avalanche decorrelates low-entropy seeds.
    pub fn new(mut s: u64) -> Self {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        s = (s ^ (s >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        s = (s ^ (s >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        Rng(s ^ (s >> 31))
    }

    /// Next `u64` from the xorshift64* core.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal via Box–Muller. Guards avoid the degenerate `ln(0)`
    /// and `ln(1)` endpoints.
    pub fn gauss(&mut self) -> f64 {
        let u = (self.next_f64() + 1e-300).clamp(1e-300, 1.0 - 1e-12);
        let v = self.next_f64().clamp(1e-12, 1.0 - 1e-12);
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_from_seed() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn gauss_has_unit_variance() {
        let mut r = Rng::new(7);
        let n = 100_000;
        let mut sum = 0.0f64;
        let mut sumsq = 0.0f64;
        for _ in 0..n {
            let x = r.gauss();
            sum += x;
            sumsq += x * x;
        }
        let mean = sum / n as f64;
        let var = sumsq / n as f64 - mean * mean;
        assert!((mean).abs() < 0.02, "mean {mean}");
        assert!((var - 1.0).abs() < 0.02, "var {var}");
    }
}
