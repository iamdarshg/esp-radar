//! RX fusion + calibration controller (spec §6, §15, §17).
//!
//! This task owns the two wired UART links — the single place RX1/RX2
//! [`FeatureReport`]s and calibration responses land (the data plane moved
//! off WiFi so neither RX transmits on the sensing band). It:
//!
//! * pairs the two links' reports by TX sequence ([`Pairer`], §15),
//! * fuses them into cross-link metrics and runs the occupancy classifier (§6),
//! * drives the calibration state machine (§17),
//! * accumulates CSI snapshots into the waterfall/spectrogram telemetry (§6),
//! * pushes a status frame to connected dashboards once per second, and
//! * triggers boot-time auto-commission (CAL 2) when no power model exists.
//!
//! The controller is the only task that touches the shared `Nvs` after boot;
//! every other task is handed the pieces it needs (atomics, a broadcaster
//! clone, a command channel).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use radar_features::{
    Fuser, FusionMetrics, LinkFeatures, OccupancyEstimate, OccupancyEstimator, OccupancyState,
};
use radar_protocol::{
    CalResp, FeatureReport, N_SUBCARRIERS, N_SPEC_BINS, frame_type, node, parse_csi_snapshot,
};
use radar_storage::nvs::Nvs;
use radar_storage::RadarConfig;
use radar_transport::udp::now_us;
use radar_transport::{Pairer, SequenceTracker, parse_feature_report};
use radar_web::server::TelemetryBroadcaster;
use radar_web::telemetry::{
    StatusFrame, StatusSnapshot, SpectrogramFrame, WaterfallFrame, link as wl,
};

use crate::calibrate::{CalCommand, Calibrator};
use crate::wired::WiredLink;

/// Sleep between link polls when idle.
const TICK_SLEEP_MS: u64 = 10;
/// Rolling length of the CSI-snapshot rings (≈10 s at ~5 Hz).
const RING_CAP: usize = 48;
/// Status + matrix telemetry cadence.
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const RINGS_PERIOD: Duration = Duration::from_secs(1);
/// If the RX links haven't both been seen after this long, auto-CAL2 runs
/// anyway (so a lone receiver can't block commissioning forever).
const BOOT_AUTO_CAL_DELAY: Duration = Duration::from_secs(20);

/// Everything the controller needs. Assembled in `main`, moved into the task.
pub struct RunParams {
    pub config: RadarConfig,
    pub status: Arc<Mutex<StatusSnapshot>>,
    pub broadcaster: TelemetryBroadcaster,
    pub tx_power: Arc<AtomicU8>,
    pub cal_active: Arc<AtomicBool>,
    pub cal_rx: mpsc::Receiver<CalCommand>,
    pub nvs: Nvs,
    /// Wired links to the RX boards (index 0 = RX1, 1 = RX2).
    pub links: [WiredLink; 2],
}

/// Rolling per-link CSI columns from the low-rate `CSI_SNAPSHOT` frames.
struct SnapshotRing {
    amps: VecDeque<[u8; N_SUBCARRIERS]>,
    spec: VecDeque<[u8; N_SPEC_BINS]>,
}

impl SnapshotRing {
    fn new() -> Self {
        Self { amps: VecDeque::with_capacity(RING_CAP), spec: VecDeque::with_capacity(RING_CAP) }
    }

    fn push(&mut self, amp: &[u8], spec: &[u8]) {
        let mut a = [0u8; N_SUBCARRIERS];
        let na = amp.len().min(a.len());
        a[..na].copy_from_slice(&amp[..na]);
        let mut s = [0u8; N_SPEC_BINS];
        let ns = spec.len().min(s.len());
        s[..ns].copy_from_slice(&spec[..ns]);
        if self.amps.len() >= RING_CAP {
            self.amps.pop_front();
        }
        if self.spec.len() >= RING_CAP {
            self.spec.pop_front();
        }
        self.amps.push_back(a);
        self.spec.push_back(s);
    }
}

pub fn run(p: RunParams) {
    let RunParams { config, status, broadcaster, tx_power, cal_active, cal_rx, nvs, mut links } = p;

    // The two wired links carry every inbound report/CAL_RESP/snapshot from
    // the RX boards; the measurement plane (RATE-1 WiFi DataFrames) is the
    // traffic task's, untouched.
    log::info!("fusion controller up: two wired links to RX1/RX2");

    // Classifier. Prefer the calibrated thresholds when present (CAL 4 output).
    let thresholds = nvs.load_thresholds().unwrap_or_default();
    if thresholds.source == radar_calibration::ThresholdSource::Calibrated {
        log::info!("using calibrated thresholds (CAL 4)");
    } else {
        log::info!("using factory-default thresholds (CAL 4 not run yet)");
    }
    let mut estimator = OccupancyEstimator::new(thresholds.to_params());
    let mut fuser = Fuser::new(32);

    // Cross-link pairing by TX sequence (§15).
    let mut pairer = Pairer::new(config.pair_tolerance as u32);
    // Per-link delivery tracking.
    let mut track1 = SequenceTracker::new();
    let mut track2 = SequenceTracker::new();

    // Calibration state machine (CAL 1..5, §17).
    let mut cal = Calibrator::new();
    cal.mark_commissioned(nvs.load_power_model().is_ok());

    // CSI-snapshot rings for the waterfall / spectrogram telemetry.
    let mut ring1 = SnapshotRing::new();
    let mut ring2 = SnapshotRing::new();

    // Latest per-link features (for the status frame even when a pair is
    // momentarily missing).
    let mut last_rep1: Option<FeatureReport> = None;
    let mut last_rep2: Option<FeatureReport> = None;
    let mut last_lf1 = LinkFeatures::default();
    let mut last_lf2 = LinkFeatures::default();
    let mut last_est: Option<OccupancyEstimate> = None;

    let mut rx1_seen = false;
    let mut rx2_seen = false;
    let mut status_seq: u32 = 0;
    let boot_started = Instant::now();
    let mut auto_cal_sent = false;
    let mut next_status = Instant::now();
    let mut next_rings = Instant::now();

    loop {
        // 1. Calibration commands from the /cal HTTP endpoint.
        while let Ok(cmd) = cal_rx.try_recv() {
            cal.start(cmd, &mut links);
        }

        // 2. Advance the calibration state machine; apply finished thresholds.
        cal.tick(now_us(), &tx_power, &nvs, &mut links);
        if let Some(t) = cal.take_thresholds() {
            if let Err(e) = nvs.store_thresholds(&t) {
                log::warn!("could not store calibrated thresholds: {e}");
            }
            estimator = OccupancyEstimator::new(t.to_params());
        }
        cal_active.store(cal.is_active(), Ordering::Relaxed);

        // 3. Boot-time auto-commission: with no stored power model, sweep the
        //    TX power once both links are heard (or after a timeout).
        if !auto_cal_sent
            && config.tx_power_db == 0
            && (boot_started.elapsed() >= BOOT_AUTO_CAL_DELAY || (rx1_seen && rx2_seen))
        {
            auto_cal_sent = true;
            log::info!("boot auto-commission triggered (no stored power model)");
            cal.start(CalCommand::AutoCommission, &mut links);
        }

        // 4. Drain both wired links. The frame header carries the source node,
        //    so which physical link a frame arrived on is irrelevant.
        for link in links.iter_mut() {
            for frame in link.poll() {
                let kind = frame.kind();
                let src = frame.src();
                let seq = frame.seq();
                match kind {
                    frame_type::FEATURE_REPORT => {
                        if let Some(report) = parse_feature_report(&frame.payload) {
                            match src {
                                node::RX1 => {
                                    rx1_seen = true;
                                    track1.observe(seq);
                                    last_rep1 = Some(report);
                                    last_lf1 = features_from_report(&report);
                                }
                                node::RX2 => {
                                    rx2_seen = true;
                                    track2.observe(seq);
                                    last_rep2 = Some(report);
                                    last_lf2 = features_from_report(&report);
                                }
                                _ => {}
                            }
                            pairer.push(src, report, now_us());
                        }
                    }
                    frame_type::CAL_RESP => {
                        if let Some(resp) = parse_cal_resp(&frame.payload) {
                            cal.on_resp(src, resp);
                        }
                    }
                    frame_type::CSI_SNAPSHOT => {
                        if let Some(snap) = parse_csi_snapshot(&frame.payload) {
                            // Copy packed-array fields out by value (E0793).
                            let amp = snap.amp_norm;
                            let spec = snap.spec;
                            match src {
                                node::RX1 => ring1.push(&amp, &spec),
                                node::RX2 => ring2.push(&amp, &spec),
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        // 5. Fuse whatever pairs are ready.
        while let Some(pair) = pairer.next_pair(now_us()) {
            let r1 = features_from_report(&pair.rx1);
            let r2 = features_from_report(&pair.rx2);
            let metrics = fuser.push(r1.motion_energy, r2.motion_energy);
            let est = estimator.update(&r1, &r2, &metrics);
            cal.on_pair(metrics.fused_energy, r1.baseline_dev.max(r2.baseline_dev));
            last_lf1 = r1;
            last_lf2 = r2;
            last_est = Some(est);
        }

        // 6. Periodic status broadcast.
        if Instant::now() >= next_status {
            next_status = Instant::now() + STATUS_PERIOD;
            broadcast_status(
                &broadcaster,
                &status,
                &cal,
                &tx_power,
                &pairer,
                &track1,
                &track2,
                last_rep1,
                last_rep2,
                last_lf1,
                last_lf2,
                last_est,
                rx1_seen && rx2_seen,
                &mut status_seq,
            );
        }

        // 7. Periodic waterfall / spectrogram broadcast.
        if Instant::now() >= next_rings {
            next_rings = Instant::now() + RINGS_PERIOD;
            broadcast_rings(&broadcaster, &ring1, &ring2);
        }

        std::thread::sleep(Duration::from_millis(TICK_SLEEP_MS));
    }
}

/// `FeatureReport` → the classifier's [`LinkFeatures`] (both share the same
/// feature vocabulary; this is just a reshape).
fn features_from_report(r: &FeatureReport) -> LinkFeatures {
    let pca = r.pca_scores; // copy out of the packed struct
    LinkFeatures {
        motion_energy: r.motion_energy,
        baseline_dev: r.baseline_dev,
        spectral_entropy: r.spectral_entropy,
        dominant_freq_hz: r.dominant_freq_hz,
        rssi: r.rssi,
        sat_score: r.sat_score,
        pca0: pca[0],
        pca1: pca[1],
        amp_std: r.amp_std,
    }
}

fn parse_cal_resp(payload: &[u8]) -> Option<CalResp> {
    if payload.len() < core::mem::size_of::<CalResp>() {
        return None;
    }
    Some(unsafe { (payload.as_ptr() as *const CalResp).read_unaligned() })
}

#[allow(clippy::too_many_arguments)]
fn broadcast_status(
    broadcaster: &TelemetryBroadcaster,
    status: &Arc<Mutex<StatusSnapshot>>,
    cal: &Calibrator,
    tx_power: &Arc<AtomicU8>,
    pairer: &Pairer,
    track1: &SequenceTracker,
    track2: &SequenceTracker,
    rep1: Option<FeatureReport>,
    rep2: Option<FeatureReport>,
    lf1: LinkFeatures,
    lf2: LinkFeatures,
    est: Option<OccupancyEstimate>,
    radar_active: bool,
    status_seq: &mut u32,
) {
    *status_seq = status_seq.wrapping_add(1);
    let now = now_us();

    let est = est.unwrap_or(OccupancyEstimate {
        state: OccupancyState::Unknown,
        confidence: 0.0,
        metrics: FusionMetrics::default(),
    });

    let mut frame = StatusFrame::default();
    frame.seq = *status_seq;
    frame.t_us = now;
    frame.occupancy = est.state;
    frame.confidence = (est.confidence * 100.0).round().clamp(0.0, 100.0) as u8;
    frame.tx_power_db = tx_power.load(Ordering::Relaxed) as i8;
    if let Some(r) = rep1 {
        frame.rssi_rx1 = r.rssi as i8;
        frame.csi_quality_rx1 = r.csi_quality;
        frame.sat_score_rx1 = r.sat_score;
        frame.dyn_range_rx1 = r.dyn_range;
    }
    if let Some(r) = rep2 {
        frame.rssi_rx2 = r.rssi as i8;
        frame.csi_quality_rx2 = r.csi_quality;
        frame.sat_score_rx2 = r.sat_score;
        frame.dyn_range_rx2 = r.dyn_range;
    }
    // Delivery % = the worse of the two links.
    let delivery = (1.0 - track1.loss_ratio()).min(1.0 - track2.loss_ratio());
    frame.packet_delivery_pct = (delivery * 100.0).round().clamp(0.0, 100.0) as u8;
    frame.paired_frames_s = pairer.pair_rate.rate(now) as u16;
    frame.motion_energy_rx1 = lf1.motion_energy;
    frame.motion_energy_rx2 = lf2.motion_energy;
    frame.motion_energy_fused = est.metrics.fused_energy;
    frame.spectral_entropy = lf1.spectral_entropy.max(lf2.spectral_entropy);
    frame.dominant_freq_hz = if lf1.motion_energy >= lf2.motion_energy {
        lf1.dominant_freq_hz as u16
    } else {
        lf2.dominant_freq_hz as u16
    };
    // PCA scores from the link with more energy right now.
    let (pca0, pca1) = if lf1.motion_energy >= lf2.motion_energy {
        (lf1.pca0, lf1.pca1)
    } else {
        (lf2.pca0, lf2.pca1)
    };
    frame.pca1 = pca0;
    frame.pca2 = pca1;
    frame.correlation = est.metrics.cross_link_corr;
    frame.differential = est.metrics.differential_rms;

    // Publish for both the /status JSON endpoint and the WS status frames.
    {
        let mut snap = status.lock().unwrap();
        snap.frame = frame;
        snap.cal_stage = cal.current_stage();
        snap.cal_active = cal.is_active();
        snap.radar_active = radar_active;
    }
    let mut buf = [0u8; StatusFrame::LEN];
    if let Ok(n) = frame.encode(&mut buf) {
        broadcaster.broadcast_raw(&buf[..n]);
    }
}

fn broadcast_rings(
    broadcaster: &TelemetryBroadcaster,
    r1: &SnapshotRing,
    r2: &SnapshotRing,
) {
    let mut buf = [0u8; 4096];
    let mut data = [0u8; N_SUBCARRIERS * RING_CAP];
    if let Some(n) = encode_waterfall(&mut buf, &mut data, wl::RX1, r1) {
        broadcaster.broadcast_raw(&buf[..n]);
    }
    if let Some(n) = encode_waterfall(&mut buf, &mut data, wl::RX2, r2) {
        broadcaster.broadcast_raw(&buf[..n]);
    }

    let mut spec = [0u8; N_SPEC_BINS * RING_CAP];
    if let Some(n) = encode_spec(&mut buf, &mut spec, wl::RX1, r1) {
        broadcaster.broadcast_raw(&buf[..n]);
    }
    if let Some(n) = encode_spec(&mut buf, &mut spec, wl::RX2, r2) {
        broadcaster.broadcast_raw(&buf[..n]);
    }
    // Fused spectrogram: tail-aligned element-wise mean of both links. The two
    // links see the same broadcast, so their columns are near-time-aligned; a
    // tail alignment is a good-enough approximation for the dashboard.
    if let Some(n) = encode_fused_spec(&mut buf, &mut spec, r1, r2) {
        broadcaster.broadcast_raw(&buf[..n]);
    }
}

fn encode_waterfall(
    buf: &mut [u8],
    data: &mut [u8],
    link: u8,
    ring: &SnapshotRing,
) -> Option<usize> {
    let bins = ring.amps.len();
    if bins == 0 {
        return None;
    }
    for (t, col) in ring.amps.iter().enumerate() {
        let base = t * N_SUBCARRIERS;
        data[base..base + N_SUBCARRIERS].copy_from_slice(col);
    }
    let wf = WaterfallFrame {
        link,
        n_sub: N_SUBCARRIERS as u8,
        bins: bins as u16,
        scale: 0,
        data: &data[..N_SUBCARRIERS * bins],
    };
    wf.encode(buf).ok()
}

fn encode_spec(
    buf: &mut [u8],
    data: &mut [u8],
    link: u8,
    ring: &SnapshotRing,
) -> Option<usize> {
    let bins = ring.spec.len();
    if bins == 0 {
        return None;
    }
    for (t, col) in ring.spec.iter().enumerate() {
        let base = t * N_SPEC_BINS;
        data[base..base + N_SPEC_BINS].copy_from_slice(col);
    }
    let sp = SpectrogramFrame {
        link,
        n_freq: N_SPEC_BINS as u8,
        bins: bins as u16,
        scale: 0,
        data: &data[..N_SPEC_BINS * bins],
    };
    sp.encode(buf).ok()
}

fn encode_fused_spec(
    buf: &mut [u8],
    data: &mut [u8],
    r1: &SnapshotRing,
    r2: &SnapshotRing,
) -> Option<usize> {
    let n = r1.spec.len().min(r2.spec.len());
    if n == 0 {
        return None;
    }
    let off1 = r1.spec.len() - n;
    let off2 = r2.spec.len() - n;
    for t in 0..n {
        let c1 = &r1.spec[off1 + t];
        let c2 = &r2.spec[off2 + t];
        let base = t * N_SPEC_BINS;
        for k in 0..N_SPEC_BINS {
            data[base + k] = ((c1[k] as u16 + c2[k] as u16) / 2) as u8;
        }
    }
    let sp = SpectrogramFrame {
        link: wl::FUSED,
        n_freq: N_SPEC_BINS as u8,
        bins: n as u16,
        scale: 0,
        data: &data[..N_SPEC_BINS * n],
    };
    sp.encode(buf).ok()
}
