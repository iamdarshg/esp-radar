//! Per-link motion features, RX1/RX2 fusion, and the occupancy state machine
//! (spec §6 "OCCUPANCY STATE" and §6 "DIFFERENTIAL CHANNEL DISPLAY").
//!
//! Key design points for the compact head:
//!  * RX1 and RX2 are independent observations of the SAME transmitted
//!    packets, so their *time* correlation is meaningful.
//!  * Because the head is fixed and the empty-room baseline is stored, static
//!    presence can be detected as a persistent deviation from baseline while
//!    temporal motion energy stays low — this is what separates
//!    STATIC_PRESENCE from EMPTY.
//!  * "Fused" energy is not simply a sum: correlated activity across both
//!    links is weighted more strongly than activity seen on one link only
//!    (the two receivers sit centimetres apart, so real environmental motion
//!    tends to move both; receiver-local noise moves one).

use core::fmt;

/// Occupancy / motion state (spec §6).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OccupancyState {
    Unknown,
    Empty,
    PossiblePresence,
    StaticPresence,
    Movement,
    StrongMovement,
    ComplexMovement,
}

impl fmt::Display for OccupancyState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            OccupancyState::Unknown => "UNKNOWN",
            OccupancyState::Empty => "EMPTY",
            OccupancyState::PossiblePresence => "POSSIBLE PRESENCE",
            OccupancyState::StaticPresence => "STATIC PRESENCE",
            OccupancyState::Movement => "MOVEMENT",
            OccupancyState::StrongMovement => "STRONG MOVEMENT",
            OccupancyState::ComplexMovement => "COMPLEX/MULTIPLE MOVEMENT",
        };
        f.write_str(s)
    }
}

/// Per-link feature set consumed by the classifier.
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkFeatures {
    pub motion_energy: f32,
    /// Deviation of the current CSI from the empty-room baseline (static
    /// presence indicator), averaged over subcarriers.
    pub baseline_dev: f32,
    pub spectral_entropy: f32,
    pub dominant_freq_hz: f32,
    pub rssi: i16,
    pub sat_score: u8,
    pub pca0: f32,
    pub pca1: f32,
    pub amp_std: f32,
}

/// Differential/fused metrics for the dashboard (§6).
#[derive(Clone, Copy, Debug, Default)]
pub struct FusionMetrics {
    pub fused_energy: f32,
    pub cross_link_corr: f32,  // -1..1
    pub activity_ratio: f32,   // r1.activity / r2.activity
    pub differential_rms: f32, // RMS of (r1 - r2) band-passed series
    pub pca_spectral_entropy: f32,
}

/// Final occupancy estimate.
#[derive(Clone, Copy, Debug)]
pub struct OccupancyEstimate {
    pub state: OccupancyState,
    pub confidence: f32, // 0..1
    pub metrics: FusionMetrics,
}

/// Tunable classifier thresholds (defaults chosen for a ~5 m sensing radius
/// at centimetre-scale board separation; overridden by CAL 4 if available).
#[derive(Clone, Copy, Debug)]
pub struct ClassifierParams {
    /// Fused motion energy above which we call MOVEMENT.
    pub move_thresh: f32,
    /// Above which we call STRONG MOVEMENT.
    pub strong_thresh: f32,
    /// Baseline deviation above which a static person is present.
    pub static_thresh: f32,
    /// Below which we call EMPTY (deadband for noise).
    pub empty_thresh: f32,
    /// How strongly correlated activity must be to count as "real".
    pub corr_weight: f32,
    /// Frames a state must persist before it is committed (hysteresis).
    pub hold_frames: u32,
}

impl Default for ClassifierParams {
    fn default() -> Self {
        Self {
            move_thresh: 0.6,
            strong_thresh: 2.5,
            static_thresh: 1.4,
            empty_thresh: 0.05,
            corr_weight: 0.5,
            hold_frames: 6,
        }
    }
}

/// Streaming occupancy estimator with hysteresis.
pub struct OccupancyEstimator {
    params: ClassifierParams,
    state: OccupancyState,
    /// Candidate currently being accumulated; committed to `state` once
    /// `hold` reaches `hold_frames`.
    pending: OccupancyState,
    hold: u32,
    last_energy: f32,
    history: Vec<OccupancyState>,
}

impl Default for OccupancyEstimator {
    fn default() -> Self {
        Self::new(ClassifierParams::default())
    }
}

impl OccupancyEstimator {
    pub fn new(params: ClassifierParams) -> Self {
        Self {
            params,
            state: OccupancyState::Unknown,
            pending: OccupancyState::Unknown,
            hold: 0,
            last_energy: 0.0,
            history: Vec::with_capacity(16),
        }
    }

    pub fn state(&self) -> OccupancyState {
        self.state
    }

    /// Update with both links' instantaneous features. `metrics` should be the
    /// fused metrics for this frame (see [`Fuser::push`]).
    pub fn update(
        &mut self,
        r1: &LinkFeatures,
        r2: &LinkFeatures,
        metrics: &FusionMetrics,
    ) -> OccupancyEstimate {
        let e = metrics.fused_energy;
        let dev = r1.baseline_dev.max(r2.baseline_dev);

        // `metrics.cross_link_corr` already shaped `e` (see Fuser) — real
        // motion across both links is weighted up, single-link noise down.
        // Candidate state for this frame.
        let candidate = if e > self.params.strong_thresh {
            // Lots of energy and spectrally diffuse → complex/multiple motion.
            if r1.spectral_entropy > 0.7 || r2.spectral_entropy > 0.7 {
                OccupancyState::ComplexMovement
            } else {
                OccupancyState::StrongMovement
            }
        } else if e > self.params.move_thresh {
            OccupancyState::Movement
        } else if dev > self.params.static_thresh {
            // Static presence: the environment differs from the empty room but
            // nothing is moving right now.
            if e > self.params.empty_thresh {
                OccupancyState::PossiblePresence
            } else {
                OccupancyState::StaticPresence
            }
        } else if e > self.params.empty_thresh {
            OccupancyState::PossiblePresence
        } else {
            OccupancyState::Empty
        };

        // Hysteresis: accumulate a *pending* candidate and only commit it to
        // `state` once it has persisted `hold_frames`. (Tracking candidate ==
        // state would deadlock the initial Unknown → anything transition,
        // since hold could never accumulate.)
        if candidate == self.pending {
            self.hold += 1;
        } else {
            self.pending = candidate;
            self.hold = 1;
        }
        self.last_energy = e;

        if self.hold >= self.params.hold_frames && self.state != self.pending {
            self.state = self.pending;
        }

        self.history.push(self.state);
        if self.history.len() > 32 {
            self.history.remove(0);
        }

        let confidence = self.confidence(candidate, dev, e);
        OccupancyEstimate {
            state: self.state,
            confidence,
            metrics: *metrics,
        }
    }

    /// Confidence grows with margin over thresholds and with how long the
    /// state has held, and falls the closer we are to a boundary.
    fn confidence(&self, candidate: OccupancyState, dev: f32, energy: f32) -> f32 {
        let p = &self.params;
        let margin = match candidate {
            OccupancyState::Empty => {
                ((p.empty_thresh - energy) / p.empty_thresh.max(1e-6)).clamp(0.0, 1.0)
            }
            OccupancyState::PossiblePresence => (energy / p.move_thresh).min(1.0),
            OccupancyState::StaticPresence => (dev / (p.static_thresh * 2.0)).min(1.0),
            OccupancyState::Movement => {
                ((energy - p.move_thresh) / p.strong_thresh).clamp(0.0, 1.0)
            }
            OccupancyState::StrongMovement | OccupancyState::ComplexMovement => {
                ((energy - p.strong_thresh) / (p.strong_thresh + 1.0)).clamp(0.0, 1.0)
            }
            OccupancyState::Unknown => 0.0,
        };
        let stability = (self.hold as f32 / p.hold_frames.max(1) as f32).min(1.0);
        (0.4 + 0.6 * margin) * (0.5 + 0.5 * stability)
    }

    pub fn reset(&mut self) {
        self.state = OccupancyState::Unknown;
        self.pending = OccupancyState::Unknown;
        self.hold = 0;
        self.history.clear();
    }
}

/// Streaming fused-metrics calculator. Maintains the rolling series needed for
/// cross-link correlation and differential RMS.
pub struct Fuser {
    buf: Vec<(f32, f32)>, // (r1_energy, r2_energy) history
    cap: usize,
}

impl Fuser {
    pub fn new(cap: usize) -> Self {
        Self {
            buf: Vec::with_capacity(cap.max(2)),
            cap: cap.max(2),
        }
    }

    /// Push one frame of per-link band-passed energy and get fused metrics.
    pub fn push(&mut self, e1: f32, e2: f32) -> FusionMetrics {
        self.buf.push((e1, e2));
        if self.buf.len() > self.cap {
            self.buf.remove(0);
        }
        let n = self.buf.len();
        if n < 2 {
            return FusionMetrics {
                fused_energy: e1.max(e2),
                cross_link_corr: 0.0,
                activity_ratio: activity_ratio(e1, e2),
                differential_rms: 0.0,
                pca_spectral_entropy: 0.0,
            };
        }

        let corr = pearson(&self.buf);
        let diff_rms = self
            .buf
            .iter()
            .map(|&(a, b)| (a - b) * (a - b))
            .sum::<f32>()
            / n as f32;
        let differential_rms = diff_rms.sqrt();

        // Correlation-weighted fusion: real environmental motion moves both
        // links; single-link noise is de-weighted.
        let c = corr.clamp(0.0, 1.0);
        let fused = (e1 * (0.5 + c) + e2 * (0.5 + c)).min(e1.max(e2) * 2.0);

        FusionMetrics {
            fused_energy: fused,
            cross_link_corr: corr,
            activity_ratio: activity_ratio(e1, e2),
            differential_rms,
            pca_spectral_entropy: 0.0,
        }
    }
}

fn activity_ratio(a: f32, b: f32) -> f32 {
    let denom = (a + b).max(1e-9);
    a / denom
}

/// Pearson correlation over the (e1, e2) history buffer.
fn pearson(buf: &[(f32, f32)]) -> f32 {
    let n = buf.len() as f32;
    if n < 2.0 {
        return 0.0;
    }
    let mut sx = 0.0;
    let mut sy = 0.0;
    for &(x, y) in buf.iter() {
        sx += x;
        sy += y;
    }
    let mx = sx / n;
    let my = sy / n;
    let mut num = 0.0;
    let mut dx = 0.0;
    let mut dy = 0.0;
    for &(x, y) in buf.iter() {
        let a = x - mx;
        let b = y - my;
        num += a * b;
        dx += a * a;
        dy += b * b;
    }
    let denom = (dx * dy).sqrt();
    if denom < 1e-12 {
        0.0
    } else {
        num / denom
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quiet_link() -> LinkFeatures {
        LinkFeatures {
            motion_energy: 0.01,
            baseline_dev: 0.1,
            spectral_entropy: 0.3,
            ..Default::default()
        }
    }

    fn moving_link(e: f32) -> LinkFeatures {
        LinkFeatures {
            motion_energy: e,
            baseline_dev: 0.2,
            spectral_entropy: 0.4,
            ..Default::default()
        }
    }

    #[test]
    fn empty_room_is_empty() {
        let mut est = OccupancyEstimator::default();
        let mut fuser = Fuser::new(16);
        let mut final_est = None;
        for _ in 0..20 {
            let m = fuser.push(0.01, 0.01);
            final_est = Some(est.update(&quiet_link(), &quiet_link(), &m));
        }
        assert_eq!(final_est.unwrap().state, OccupancyState::Empty);
    }

    #[test]
    fn motion_escalates_to_movement() {
        // With the correlation-weighted fusion, equal fully-correlated links
        // produce fused_energy ≈ 2·e. e=0.8 → fused ≈ 1.6, squarely in the
        // MOVEMENT band (0.6 < fused < 2.5).
        let mut est = OccupancyEstimator::default();
        let mut fuser = Fuser::new(16);
        let mut final_est = None;
        for _ in 0..20 {
            let m = fuser.push(0.8, 0.8);
            final_est = Some(est.update(&moving_link(0.8), &moving_link(0.8), &m));
        }
        assert_eq!(final_est.unwrap().state, OccupancyState::Movement);
    }

    #[test]
    fn correlated_fusion_beats_uncorrelated() {
        let mut fuser = Fuser::new(16);
        // Correlated activity on both links.
        for i in 0..16 {
            let v = (i as f32).sin().abs() + 0.5;
            fuser.push(v, v);
        }
        let m = fuser.push(1.0, 1.0);
        // Then reset and push uncorrelated noise.
        let mut fuser2 = Fuser::new(16);
        for i in 0..16 {
            let v = (i as f32).sin().abs() + 0.5;
            fuser2.push(v, v + (i as f32).cos() * 3.0);
        }
        let m2 = fuser2.push(1.0, 1.0);
        assert!(m.cross_link_corr.abs() > 0.9);
        assert!(m2.cross_link_corr < m.cross_link_corr);
    }

    #[test]
    fn static_presence_detected() {
        let mut est = OccupancyEstimator::default();
        let mut fuser = Fuser::new(16);
        let mut final_est = None;
        let mut s_link = quiet_link();
        s_link.baseline_dev = 3.0; // strong deviation from baseline, no motion
        for _ in 0..20 {
            let m = fuser.push(0.01, 0.01);
            final_est = Some(est.update(&s_link, &s_link, &m));
        }
        assert_eq!(final_est.unwrap().state, OccupancyState::StaticPresence);
    }
}
