//! Streaming PCA over a ring buffer of recent observations.
//!
//! The covariance is recomputed from the ring buffer on every `update`, which
//! is O(W·K²) per update. For K = 56 subcarriers and W = 32 frames that is
//! ~100k multiply-adds per update — trivial at the ~10–20 Hz feature rate the
//! radar actually runs. Top components are found with power iteration +
//! deflation, so no external linear-algebra crate is needed.

/// Online PCA. `k` is the number of input dimensions, `window` the rolling
/// history length, `n_components` the number of components to track.
pub struct Pca {
    k: usize,
    window: usize,
    n_components: usize,
    buffer: Vec<Vec<f32>>, // ring of observations
    head: usize,
    count: usize,
    components: Vec<Vec<f32>>, // eigenvectors (unit norm), k x n_components
}

impl Pca {
    pub fn new(k: usize, window: usize, n_components: usize) -> Self {
        Self {
            k,
            window: window.max(2),
            n_components: n_components.clamp(1, k),
            buffer: vec![vec![0.0; k]; window],
            head: 0,
            count: 0,
            components: vec![vec![0.0; k]; n_components.clamp(1, k)],
        }
    }

    pub fn n_components(&self) -> usize {
        self.n_components
    }

    /// Push an observation. Returns `true` once the buffer is warm enough to
    /// produce components.
    pub fn update(&mut self, x: &[f32]) -> bool {
        debug_assert_eq!(x.len(), self.k);
        let slot = &mut self.buffer[self.head];
        slot.copy_from_slice(x);
        self.head = (self.head + 1) % self.window;
        self.count = (self.count + 1).min(self.window);
        if self.count >= 4 {
            self.fit();
            true
        } else {
            false
        }
    }

    /// Mean vector over the current buffer.
    pub fn mean(&self) -> Vec<f32> {
        let n = self.count.max(1) as f32;
        let mut m = vec![0.0; self.k];
        for i in 0..self.count {
            let row = &self.buffer[(self.head + self.window - self.count + i) % self.window];
            for (j, v) in m.iter_mut().enumerate() {
                *v += row[j];
            }
        }
        for v in m.iter_mut() {
            *v /= n;
        }
        m
    }

    fn covariance(&self, mean: &[f32]) -> Vec<f32> {
        // Symmetric covariance stored row-major (k x k), only upper half used.
        let mut cov = vec![0.0; self.k * self.k];
        let n = self.count.max(1) as f32;
        for i in 0..self.count {
            let row = &self.buffer[(self.head + self.window - self.count + i) % self.window];
            for r in 0..self.k {
                let dr = row[r] - mean[r];
                for c in r..self.k {
                    let dc = row[c] - mean[c];
                    cov[r * self.k + c] += dr * dc;
                }
            }
        }
        for r in 0..self.k {
            for c in r..self.k {
                cov[r * self.k + c] /= n;
                cov[c * self.k + r] = cov[r * self.k + c];
            }
        }
        cov
    }

    fn fit(&mut self) {
        let mean = self.mean();
        let cov = self.covariance(&mean);
        let mut residual = cov.clone();
        // Power iteration with deflation for each component.
        for comp in 0..self.n_components {
            let mut v = vec![0.0; self.k];
            // Deterministic init: unit vector along the first dimension + small
            // perturbation so different components don't collapse.
            let r = (comp + 1) as f32 * 0.03125;
            for (j, x) in v.iter_mut().enumerate() {
                *x = ((j as f32 + 1.0) * r).sin() * 0.5;
            }
            v[comp.min(self.k - 1)] += 1.0;
            normalize(&mut v);

            let mut prev_ev = 0.0f32;
            for _ in 0..25 {
                let mut w = mat_vec(&residual, &v, self.k);
                let lambda = dot(&v, &w);
                normalize(&mut w);
                let ev = (lambda - prev_ev).abs();
                v = w;
                if ev < 1e-5 {
                    break;
                }
                prev_ev = lambda;
            }
            self.components[comp] = v.clone();
            // Deflate: remove the projection of this component from residual.
            let l = mat_vec(&residual, &v, self.k);
            let eig = dot(&v, &l).max(0.0);
            for r in 0..self.k {
                for c in 0..self.k {
                    residual[r * self.k + c] -= eig * v[r] * v[c];
                }
            }
        }
    }

    /// Project an observation onto the current components. Scores are in
    /// descending eigenvalue order.
    pub fn project(&self, x: &[f32]) -> Vec<f32> {
        let mean = self.mean();
        let centered: Vec<f32> = x.iter().zip(mean.iter()).map(|(a, b)| a - b).collect();
        self.components.iter().map(|c| dot(&centered, c)).collect()
    }

    /// Normalized component vectors (unit length).
    pub fn components(&self) -> &[Vec<f32>] {
        &self.components
    }

    /// Reset all state (e.g. after a baseline change).
    pub fn reset(&mut self) {
        self.head = 0;
        self.count = 0;
        for c in self.components.iter_mut() {
            c.fill(0.0);
        }
    }
}

fn dot(a: &[f32], b: &[f32]) -> f32 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

fn normalize(v: &mut [f32]) {
    let n = (v.iter().map(|x| x * x).sum::<f32>()).sqrt();
    if n > 1e-12 {
        for x in v.iter_mut() {
            *x /= n;
        }
    }
}

fn mat_vec(m: &[f32], v: &[f32], k: usize) -> Vec<f32> {
    let mut out = vec![0.0; k];
    for r in 0..k {
        let mut acc = 0.0;
        for c in 0..k {
            acc += m[r * k + c] * v[c];
        }
        out[r] = acc;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_dominant_direction() {
        // Observations are mostly along the +x axis in 3D with small y/z noise.
        let mut pca = Pca::new(3, 16, 2);
        for i in 0..64 {
            let t = i as f32 * 0.01;
            let x = [t + (i as f32).sin() * 0.01, (i as f32).cos() * 0.01, 0.0];
            pca.update(&x);
        }
        let c0 = &pca.components[0];
        // Dominant component should be ~(1,0,0).
        assert!(c0[0] > 0.9, "c0 = {c0:?}");
    }

    #[test]
    fn projection_has_zero_mean_direction() {
        let mut pca = Pca::new(2, 8, 1);
        for i in 0..16 {
            pca.update(&[(i as f32) * 0.5, (i as f32) * -0.5]);
        }
        let scores = pca.project(&[4.0, -4.0]);
        // A vector along the anti-diagonal should project strongly (nonzero).
        assert!(scores[0].abs() > 0.5);
    }
}
