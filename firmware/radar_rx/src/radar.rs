//! RADAR-RX measurement, DSP and calibration loop (spec §10, §16).
//!
//! Runs on the CSI worker task. Per measurement cycle it:
//!
//!   1. Drains the lock-free [`CsiRing`], keeping only frames whose source MAC
//!      is the RADAR-TX AP (the CSI callback fires for *every* 802.11 frame —
//!      beacons, foreign stations, ...).
//!   2. Pushes each kept frame through the §16 pipeline:
//!      `decode_channel` → `Normalizer` (baseline z-score) → temporal band-pass
//!      (per-subcarrier Biquad HP+LP, human-motion band) → PCA → PC1.
//!   3. Accumulates per-window link statistics, and every `report_every` frames
//!      emits a compact [`FeatureReport`] to RADAR-TX over the wired link.
//!   4. Every `~2 Hz` emits a [`CsiSnapshot`] with the per-subcarrier IQ /
//!      normalized amplitudes and the current motion-spectrum column (the
//!      dashboard's LIVE WATERFALL) — also over the wire.
//!   5. Serves the calibration protocol: RADAR-TX sends `CalCmd`s down the same
//!      wired link; this loop collects the stage window and answers with a
//!      `CalResp` (RF-power sweep, empty-room baseline → NVS, moving test,
//!      fingerprint).
//!
//! The measurement plane is untouched: RATE-1 `DataFrame`s still arrive on the
//! WiFi measurement port. Only the RX→TX data plane moved to the UART, so this
//! board no longer transmits on the 2.4 GHz sensing band.
//!
//! Nothing here ever returns — `run` blocks forever, keeping the leaked
//! `CsiRing`, the wired link and the calibration state alive.

use std::collections::VecDeque;

use radar_calibration::{BaselineCollector, BaselineStats, MIN_BASELINE_SAMPLES};
use radar_csi::{CsiFrame, CsiRing};
use radar_dsp::filter::Biquad;
use radar_dsp::metrics::{circular_variance, dominant_freq_hz, spectral_entropy};
use radar_dsp::pca::Pca;
use radar_dsp::transform::{decode_channel, Normalizer};
use radar_dsp::{Channel, N_SUBCARRIERS};
use radar_protocol::{cal_action, cal_result, cal_stage, frame_type, CalCmd, CalResp};
use radar_protocol::{CsiSnapshot, FeatureReport, MAX_PCA, N_SPEC_BINS};
use radar_storage::nvs::Nvs;
use radar_storage::{RadarConfig, RxLink};
use radar_transport::udp::{now_us, recv_radar_frame, UdpSocket};
use radar_transport::MEASURE_PORT;

use crate::link::role_name;
use crate::wired::WiredLink;

/// Length of the rolling PC1 history that feeds the STFT (matches the
/// protocol's `N_SPEC_BINS` count: an `fft_len` of 128 → 64 magnitude bins).
const STFT_LEN: usize = 128;
/// PCA basis refit cadence: every Nth frame. Refitting is O(W·K²) (see
/// `radar_dsp::pca`); decimating keeps the per-frame cost a few percent of a
/// core while the projection — the part we use every frame — stays live.
const PCA_REFIT_EVERY: u32 = 4;
/// HP cutoff of the human-motion band, Hz (spec §16).
const HP_CUTOFF_HZ: f32 = 0.2;
/// LP cutoff of the human-motion band, Hz.
const LP_CUTOFF_HZ: f32 = 5.0;
/// Snapshot cadence: emit one every `report_every * SNAPSHOT_FACTOR` frames.
const SNAPSHOT_FACTOR: u32 = 5;

/// Everything the CSI worker needs, handed over by the boot task.
pub struct RunParams {
    pub config: RadarConfig,
    pub node_id: u8,
    pub link: RxLink,
    /// Wired UART link to RADAR-TX (reports/snapshots/CAL_RESP up, CAL_CMD down).
    pub wired: WiredLink,
    pub ring: &'static CsiRing,
    pub nvs: Nvs,
    pub ap_bssid: [u8; 6],
}

/// The §16 pipeline as live state: normalizer (from the stored baseline),
/// per-subcarrier band-pass filters, and the streaming PCA.
struct Pipeline {
    normalizer: Normalizer,
    /// One high-pass section per subcarrier.
    hp: Vec<Biquad>,
    /// One low-pass section per subcarrier, in series after the HP.
    lp: Vec<Biquad>,
    pca: Pca,
    /// Rolling band-passed PC1 history for the STFT (capped at `STFT_LEN`).
    pc1_hist: VecDeque<f32>,
    /// The *stored* empty-room baseline (NVS), kept separate from the live
    /// normalizer so `baseline_dev` measures drift against the calibrated
    /// reference even after a normalizer update.
    stored_baseline: Option<BaselineStats>,
    /// PCA projection scores of the most recent observation (descending
    /// eigenvalue order), copied into the next report's `pca_scores`.
    last_pc_scores: [f32; MAX_PCA],
}

impl Pipeline {
    fn new(baseline: Option<BaselineStats>, fs: f32) -> Self {
        // Match on a reference: `BaselineStats` is not `Copy`, and taking
        // `b.mean`/`b.std` by value out of the `Some` arm would partially move
        // it — but `stored_baseline` below needs the whole value.
        let normalizer = match &baseline {
            Some(b) if b.valid => Normalizer {
                base: b.mean,
                scale: b.std,
            },
            _ => Normalizer::default(),
        };
        let hp_cut = (HP_CUTOFF_HZ / fs).clamp(0.001, 0.49);
        let lp_cut = (LP_CUTOFF_HZ / fs).clamp(0.001, 0.49);
        let mut hp = Vec::with_capacity(N_SUBCARRIERS);
        let mut lp = Vec::with_capacity(N_SUBCARRIERS);
        for _ in 0..N_SUBCARRIERS {
            hp.push(Biquad::highpass(hp_cut));
            lp.push(Biquad::lowpass(lp_cut));
        }
        Self {
            normalizer,
            hp,
            lp,
            pca: Pca::new(N_SUBCARRIERS, 32, 2),
            pc1_hist: VecDeque::with_capacity(STFT_LEN),
            stored_baseline: baseline,
            last_pc_scores: [0.0; MAX_PCA],
        }
    }

    /// Reset filter/PCA memory after a baseline change so the old operating
    /// point does not leak a transient into the freshly-calibrated signal.
    fn reset_transients(&mut self) {
        for f in self.hp.iter_mut() {
            f.reset();
        }
        for f in self.lp.iter_mut() {
            f.reset();
        }
        self.pca.reset();
        self.pc1_hist.clear();
    }

    /// Run one frame through normalize → band-pass → PCA. Returns
    /// (band-passed 56-vector, PC1 score).
    fn process(&mut self, ch: &Channel, frame_idx: u32) -> ([f32; N_SUBCARRIERS], f32) {
        let norm = self.normalizer.normalize(&ch.amps);
        let mut bp = [0.0f32; N_SUBCARRIERS];
        for i in 0..N_SUBCARRIERS {
            let x = self.hp[i].process(norm[i]);
            bp[i] = self.lp[i].process(x);
        }

        // Refit the PCA basis on a decimated cadence; project every frame.
        if frame_idx % PCA_REFIT_EVERY == 0 {
            self.pca.update(&bp);
        }
        let scores = self.pca.project(&bp);
        let pc1 = scores.first().copied().unwrap_or(0.0);
        for (dst, &s) in self.last_pc_scores.iter_mut().zip(scores.iter()) {
            *dst = s;
        }
        if self.pc1_hist.len() >= STFT_LEN {
            self.pc1_hist.pop_front();
        }
        self.pc1_hist.push_back(pc1);
        (bp, pc1)
    }
}

/// Statistics accumulated over a calibration-collection window. Fed one frame
/// at a time; turned into a [`CalResp`] when the window's deadline passes.
#[derive(Default)]
struct LinkAccum {
    n: u32,
    rssi_sum: i64,
    snr_sum: i64,
    sat_sum: u64,
    quality_sum: u64,
    dyn_sum: u64,
    noise_sum: i64,
}

impl LinkAccum {
    fn push(&mut self, frame: &CsiFrame, ch: &Channel) {
        if !ch.valid {
            return;
        }
        self.n += 1;
        self.rssi_sum += ch.rssi as i64;
        self.noise_sum += ch.noise_floor as i64;
        let snr = ch.rssi as i32 - ch.noise_floor as i32;
        self.snr_sum += snr as i64;
        self.sat_sum += sat_score(frame) as u64;
        self.quality_sum += csi_quality(snr) as u64;
        self.dyn_sum += dyn_range(ch) as u64;
    }

    /// Means across the window, plus the saturation worst-case (the TX sweep
    /// stops the moment *any* sample clips hard — SAT_STOP semantics).
    fn finish(&self, stage: u8, result: u8) -> CalResp {
        let n = self.n.max(1);
        let snr_mean = (self.snr_sum / n as i64) as i8;
        let noise_floor = self.noise_sum as f32 / n as f32;
        CalResp {
            stage,
            result,
            rssi: (self.rssi_sum / n as i64) as i16,
            snr: snr_mean,
            csi_quality: (self.quality_sum / n as u64) as u8,
            sat_score: (self.sat_sum / n as u64) as u8,
            dyn_range: (self.dyn_sum / n as u64) as u8,
            noise_floor,
            n_samples: self.n,
            ..Default::default()
        }
    }
}

/// Calibration collection state machine. A stage is "armed" by a `CalCmd`,
/// fed one CSI frame at a time, and finalized when its deadline passes.
enum CalCollect {
    None,
    /// CAL 2: RF-power sweep point.
    RfPower { end_us: u64, stats: LinkAccum },
    /// CAL 3: empty-room baseline.
    Baseline { end_us: u64, collector: BaselineCollector },
    /// CAL 4: moving test (TX does not wait on the response).
    Moving { end_us: u64, stats: LinkAccum },
    /// CAL 5: fingerprint capture.
    Fingerprint { end_us: u64, stats: LinkAccum },
}

impl CalCollect {
    fn deadline(&self) -> u64 {
        match self {
            Self::None => u64::MAX,
            Self::RfPower { end_us, .. }
            | Self::Baseline { end_us, .. }
            | Self::Moving { end_us, .. }
            | Self::Fingerprint { end_us, .. } => *end_us,
        }
    }
}

/// Main loop. Blocks forever.
pub fn run(params: RunParams) -> ! {
    let RunParams { config, node_id, link, mut wired, ring, nvs, ap_bssid } = params;
    let fs = config.tx_rate_hz as f32;
    let report_every = config.report_every.max(1) as u32;
    let snapshot_every = (report_every * SNAPSHOT_FACTOR).max(report_every);
    let name = role_name(node_id);

    // The stored empty-room baseline, if CAL 3 ran on this link.
    let baseline = nvs.load_baseline(link).ok();
    if let Some(b) = &baseline {
        log::info!("{name}: loaded baseline ({} samples, rssi {} dBm)", b.n_samples, b.rssi_mean);
    } else {
        log::info!("{name}: no stored baseline; normalizing against raw scale until CAL 3");
    }
    let mut pipe = Pipeline::new(baseline, fs);

    // The measurement plane stays on WiFi: RATE-1 DataFrames arrive on the
    // measurement port. Reports/snapshots/CAL_RESP now go up the wired link
    // and CAL_CMD comes down it, so no report socket is bound — this board no
    // longer transmits on the 2.4 GHz sensing band.
    let mut sock = UdpSocket::bind(MEASURE_PORT).expect("bind measurement port");
    sock.set_recv_timeout(20).expect("set recv timeout");
    let mut rbuf = [0u8; 256];

    let mut cal = CalCollect::None;

    // Report-window state.
    let mut window = ReportWindow::default();
    let mut last_tx_seq = 0u32;
    let mut win_start_seq = 0u32;
    let mut ring_overflow = ring.overflow_count();

    log::info!(
        "{name} radar loop: fs={}Hz report_every={} snapshot_every={} mac={ap_bssid:02x?}",
        config.tx_rate_hz,
        report_every,
        snapshot_every
    );

    let mut frame_idx: u32 = 0;
    loop {
        // -- 1. Drain the CSI ring ------------------------------------------
        while let Some(frame) = ring.pop() {
            if frame.info.mac != ap_bssid {
                continue; // beacon / foreign station — not our measurement link
            }
            frame_idx = frame_idx.wrapping_add(1);
            let ch = decode_channel(
                &frame.buf,
                frame.info.first_word_invalid,
                frame.info.rssi,
                frame.info.noise_floor,
            );
            if !ch.valid {
                continue;
            }

            // Feed any armed calibration stage first, then the report window.
            match &mut cal {
                CalCollect::RfPower { stats, .. }
                | CalCollect::Moving { stats, .. }
                | CalCollect::Fingerprint { stats, .. } => stats.push(&frame, &ch),
                CalCollect::Baseline { collector, .. } => collector.update(&ch, ch.rssi),
                CalCollect::None => {}
            }

            let (_bp, pc1) = pipe.process(&ch, frame_idx);
            window.push(&frame, &ch, &pipe, pc1);

            // Emit a FeatureReport when the window is full.
            if window.n >= report_every {
                let report = window.report(
                    &pipe,
                    fs,
                    last_tx_seq,
                    win_start_seq,
                    ring.overflow_count().saturating_sub(ring_overflow),
                );
                ring_overflow = ring.overflow_count();
                win_start_seq = last_tx_seq;
                if let Err(e) = wired.send_feature_report(node_id, &report) {
                    log::warn!("report send failed: {e}");
                }
                window.reset();

                // Snapshot cadence: every `snapshot_every` frames (~2 Hz at the
                // default 200 Hz rate).
                if frame_idx % snapshot_every == 0 {
                    let snap = make_snapshot(&pipe, &ch, fs, last_tx_seq);
                    if let Err(e) = wired.send_csi_snapshot(node_id, &snap) {
                        log::warn!("snapshot send failed: {e}");
                    }
                }
            }
        }

        // -- 2. Calibration deadline ----------------------------------------
        if let Some(deadline) = active_deadline(&cal) {
            if now_us() >= deadline {
                finalize_cal(&mut cal, &mut wired, node_id, &nvs, link, &mut pipe);
            }
        }

        // -- 3. Incoming measurement frames (WiFi) ----------------------------
        // Only RATE-1 DataFrames arrive on the measurement port now.
        if let Some((kind, _src, seq, _payload, _peer_ip, _peer_port)) =
            recv_radar_frame(&mut sock, &mut rbuf)
        {
            if kind == frame_type::DATA_FRAME {
                last_tx_seq = seq;
            }
        }

        // -- 4. Inbound wired frames ------------------------------------------
        // CAL_CMD arrives down the UART; anything else on the wire is logged
        // and dropped.
        for frame in wired.poll() {
            if frame.kind() == frame_type::CAL_CMD {
                if frame.payload.len() >= core::mem::size_of::<CalCmd>() {
                    let cmd = unsafe { (frame.payload.as_ptr() as *const CalCmd).read_unaligned() };
                    handle_cal_cmd(&mut cal, &cmd, &mut wired, node_id);
                }
            }
        }
    }
}

// -- calibration protocol -----------------------------------------------------

fn active_deadline(cal: &CalCollect) -> Option<u64> {
    match cal {
        CalCollect::None => None,
        other => Some(other.deadline()),
    }
}

/// Handle a `CalCmd` from RADAR-TX. `IDENTITY` is answered immediately; the
/// timed stages arm a [`CalCollect`] that is finalized when its deadline
/// passes.
fn handle_cal_cmd(
    cal: &mut CalCollect,
    cmd: &CalCmd,
    wired: &mut WiredLink,
    node_id: u8,
) {
    if cmd.action == cal_action::ABORT {
        log::warn!("CAL {} aborted by host", cmd.stage);
        *cal = CalCollect::None;
        return;
    }
    let end_us = now_us() + (cmd.collect_ms as u64) * 1000;

    match cmd.stage {
        cal_stage::IDENTITY => {
            // Immediate ack so the host learns both links are alive.
            let resp = CalResp {
                stage: cal_stage::IDENTITY,
                result: cal_result::OK,
                rssi: 0,
                snr: 0,
                csi_quality: 0,
                sat_score: 0,
                dyn_range: 0,
                noise_floor: 0.0,
                n_samples: 0,
                ..Default::default()
            };
            log::info!("CAL 1: identity ack");
            if let Err(e) = wired.send_cal_resp(node_id, &resp) {
                log::warn!("identity ack failed: {e}");
            }
        }
        cal_stage::RF_POWER => {
            *cal = CalCollect::RfPower { end_us, stats: LinkAccum::default() };
            // `CalCmd` is `#[repr(C, packed)]` — copy the wider fields to
            // locals before `format!` takes a reference to them (E0793).
            let tx_power_db = cmd.tx_power_db;
            let collect_ms = cmd.collect_ms;
            log::info!("CAL 2: sweep point @{tx_power_db} dBm for {collect_ms} ms");
        }
        cal_stage::EMPTY_ROOM => {
            *cal = CalCollect::Baseline { end_us, collector: BaselineCollector::new() };
            let collect_ms = cmd.collect_ms;
            log::info!("CAL 3: empty-room baseline for {collect_ms} ms");
        }
        cal_stage::MOVING_TEST => {
            *cal = CalCollect::Moving { end_us, stats: LinkAccum::default() };
            let collect_ms = cmd.collect_ms;
            log::info!("CAL 4: moving test for {collect_ms} ms (reports flow)");
        }
        cal_stage::FINGERPRINT => {
            *cal = CalCollect::Fingerprint { end_us, stats: LinkAccum::default() };
            let collect_ms = cmd.collect_ms;
            log::info!("CAL 5: fingerprint for {collect_ms} ms");
        }
        other => log::warn!("unknown cal stage {other}"),
    }
}

/// Finalize an armed stage once its window elapses: compute the response from
/// the accumulated statistics, persist CAL 3's baseline, and re-arm the
/// normalizer so the freshly-captured empty-room state becomes the reference.
fn finalize_cal(
    cal: &mut CalCollect,
    wired: &mut WiredLink,
    node_id: u8,
    nvs: &Nvs,
    link: RxLink,
    pipe: &mut Pipeline,
) {
    let name = role_name(node_id);
    let (resp, log_line) = match core::mem::replace(cal, CalCollect::None) {
        CalCollect::RfPower { stats, .. } => {
            // Saturated when the front end clipped hard enough that the sweep
            // should stop (TX SAT_STOP checks this).
            let sat = (stats.sat_sum.max(1) / stats.n.max(1) as u64) as u8;
            let result = if sat >= SAT_THRESHOLD { cal_result::SAT } else { cal_result::OK };
            (stats.finish(cal_stage::RF_POWER, result), format!("CAL 2: rssi={} sat={sat} result={result}", stats.rssi_sum / stats.n.max(1) as i64))
        }
        CalCollect::Baseline { collector, .. } => {
            let b = collector.finish();
            // Snapshot the scalars before `b` is moved into the pipeline so the
            // response can reference them afterwards (E0382: `b` is not Copy).
            let valid = b.valid;
            let rssi_mean = b.rssi_mean;
            let n_samples = b.n_samples;
            let noise_floor = b.noise_floor;
            let mean = b.mean;
            let std = b.std;
            if valid {
                match nvs.store_baseline(link, &b) {
                    Ok(()) => log::info!("{name}: stored baseline to NVS"),
                    Err(e) => log::warn!("{name}: baseline store failed: {e}"),
                }
                // The empty-room state just became the reference — update both
                // the live normalizer and the report's drift reference, and
                // drop filter/PCA memory so no old-operating-point transient
                // bleeds into the freshly-calibrated signal.
                pipe.normalizer = Normalizer { base: mean, scale: std };
                pipe.stored_baseline = Some(b);
                pipe.reset_transients();
            }
            let result = if valid { cal_result::OK } else { cal_result::ERR };
            let resp = CalResp {
                stage: cal_stage::EMPTY_ROOM,
                result,
                rssi: rssi_mean,
                snr: 0,
                csi_quality: 0,
                sat_score: 0,
                dyn_range: 0,
                noise_floor,
                n_samples,
                ..Default::default()
            };
            let detail = if valid {
                format!("{n_samples} samples, rssi {rssi_mean} dBm")
            } else {
                format!("only {n_samples} samples (< {MIN_BASELINE_SAMPLES})")
            };
            (resp, format!("CAL 3: baseline {} ({detail})", if valid { "OK" } else { "insufficient" }))
        }
        CalCollect::Moving { stats, .. } => {
            (stats.finish(cal_stage::MOVING_TEST, cal_result::OK), "CAL 4: moving test window elapsed".into())
        }
        CalCollect::Fingerprint { stats, .. } => {
            (stats.finish(cal_stage::FINGERPRINT, cal_result::OK), "CAL 5: fingerprint captured".into())
        }
        CalCollect::None => return,
    };

    log::info!("{name}: {log_line}");
    if let Err(e) = wired.send_cal_resp(node_id, &resp) {
        log::warn!("cal response send failed: {e}");
    }
}

/// Saturation threshold: a stage whose mean clip ratio reaches this answers
/// `SAT` so the TX stops the power sweep early.
const SAT_THRESHOLD: u8 = 70;

// -- per-frame metrics --------------------------------------------------------

/// 0..100, how close the receiver's raw CSI is to clipping (|I| or |Q| at the
/// i8 limit). High values mean the RF front end is being driven into
/// saturation — the driver for CAL 2's SAT_STOP.
fn sat_score(frame: &CsiFrame) -> u8 {
    if frame.buf.is_empty() {
        return 0;
    }
    let clip = frame.buf.iter().filter(|&&b| b >= 120 || b <= -120).count();
    ((clip * 100) / frame.buf.len()).min(100) as u8
}

/// SNR in dB from the radio metadata.
fn snr_db(ch: &Channel) -> i32 {
    ch.rssi as i32 - ch.noise_floor as i32
}

/// 0..100 link-quality heuristic from SNR: -40 dB → 0, +60 dB → 100.
fn csi_quality(snr: i32) -> u8 {
    (snr + 40).clamp(0, 100) as u8
}

/// 0..100 dynamic range of the per-subcarrier amplitude distribution.
fn dyn_range(ch: &Channel) -> u8 {
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for &a in ch.amps.iter() {
        lo = lo.min(a);
        hi = hi.max(a);
    }
    ((hi - lo) / 128.0 * 100.0).clamp(0.0, 100.0) as u8
}

// -- report window ------------------------------------------------------------

/// Per-window accumulators for one [`FeatureReport`]. Reset every
/// `report_every` frames.
struct ReportWindow {
    n: u32,
    rssi_sum: i64,
    snr_sum: i64,
    quality_sum: u64,
    sat_sum: u64,
    dyn_sum: u64,
    amp_sum: f32,
    amp_sq_sum: f32,
    pc1_sumsq: f32,
    last_phase: [f32; N_SUBCARRIERS],
    last_amps: [f32; N_SUBCARRIERS],
    baseline_dev_acc: f32,
}

impl Default for ReportWindow {
    // `[f32; N_SUBCARRIERS]` (> 32 elements) has no `Default` impl yet, so
    // derive can't fill it — spell the default out by hand.
    fn default() -> Self {
        Self {
            n: 0,
            rssi_sum: 0,
            snr_sum: 0,
            quality_sum: 0,
            sat_sum: 0,
            dyn_sum: 0,
            amp_sum: 0.0,
            amp_sq_sum: 0.0,
            pc1_sumsq: 0.0,
            last_phase: [0.0; N_SUBCARRIERS],
            last_amps: [0.0; N_SUBCARRIERS],
            baseline_dev_acc: 0.0,
        }
    }
}

impl ReportWindow {
    fn push(&mut self, frame: &CsiFrame, ch: &Channel, pipe: &Pipeline, pc1: f32) {
        self.n += 1;
        self.rssi_sum += ch.rssi as i64;
        let snr = snr_db(ch);
        self.snr_sum += snr as i64;
        self.quality_sum += csi_quality(snr) as u64;
        self.sat_sum += sat_score(frame) as u64;
        self.dyn_sum += dyn_range(ch) as u64;

        let mean_amp = ch.mean_amp();
        self.amp_sum += mean_amp;
        self.amp_sq_sum += mean_amp * mean_amp;
        self.pc1_sumsq += pc1 * pc1;

        self.last_phase = ch.phase;
        self.last_amps = ch.amps;

        // Deviation from the *stored* empty-room baseline (z-scores), not the
        // live normalizer — this is the static-presence signal the TX fuses.
        if let Some(b) = pipe.stored_baseline.as_ref() {
            let mut acc = 0.0f32;
            for i in 0..N_SUBCARRIERS {
                let dev = (ch.amps[i] - b.mean[i]) / b.std[i].max(1e-6);
                acc += dev.abs();
            }
            self.baseline_dev_acc += acc / N_SUBCARRIERS as f32;
        }
    }

    /// Turn the accumulated window into a [`FeatureReport`].
    fn report(
        &self,
        pipe: &Pipeline,
        fs: f32,
        last_tx_seq: u32,
        win_start_seq: u32,
        overflow_delta: u32,
    ) -> FeatureReport {
        let ni = self.n.max(1) as i64;
        let n = ni as f32;
        let snr_mean = (self.snr_sum / ni) as i8;

        // Motion energy in the human-motion band: RMS of the band-passed PC1
        // series over the window. PC1 is a projection of z-scored, band-passed
        // amplitudes, so a quiet link sits at ~0.1 while active motion reaches
        // 1-3 — the scale the CAL 4 histogram (0..10) and the fused thresholds
        // expect.
        let motion_energy = (self.pc1_sumsq / n).sqrt();

        // Spectral state over the rolling 128-frame history.
        let (entropy, dominant_hz) = spectral(pipe, fs);

        let amp_mean = self.amp_sum / n;
        let amp_var = (self.amp_sq_sum / n - amp_mean * amp_mean).max(0.0);
        let amp_std = amp_var.sqrt();

        let mut report = FeatureReport {
            seq: last_tx_seq,
            n_frames: self.n,
            // Expected frames in the window from the TX counter, minus what we
            // actually processed.
            n_missing: last_tx_seq.wrapping_sub(win_start_seq).saturating_sub(self.n),
            rssi: (self.rssi_sum / ni) as i16,
            snr: snr_mean,
            csi_quality: (self.quality_sum / self.n.max(1) as u64) as u8,
            sat_score: (self.sat_sum / self.n.max(1) as u64) as u8,
            dyn_range: (self.dyn_sum / self.n.max(1) as u64) as u8,
            flags: if overflow_delta > 0 {
                radar_protocol::report_flags::OVERFLOW
            } else {
                0
            },
            amp_mean,
            amp_std,
            motion_energy,
            spectral_entropy: entropy,
            dominant_freq_hz: dominant_hz,
            phase_dispersion: circular_variance(&self.last_phase),
            baseline_dev: if self.n > 0 && pipe.stored_baseline.is_some() {
                self.baseline_dev_acc / self.n as f32
            } else {
                0.0
            },
            ..Default::default()
        };

        // PCA projection scores of the most recent observation (descending
        // eigenvalue order), matching the protocol's slot count.
        report.pca_scores = pipe.last_pc_scores;
        report
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Spectral features from the rolling PC1 history: STFT magnitude over the
/// last `STFT_LEN` samples, squared to power for the entropy/frequency
/// metrics.
fn spectral(pipe: &Pipeline, fs: f32) -> (f32, f32) {
    if pipe.pc1_hist.is_empty() {
        return (0.0, 0.0);
    }
    let mut frame: Vec<f32> = pipe.pc1_hist.iter().copied().collect();
    // Pad to a full STFT window so the first reports aren't degenerate.
    while frame.len() < STFT_LEN {
        frame.push(0.0);
    }
    let mag = radar_dsp::fft::stft_frame(&frame, STFT_LEN);
    let power: Vec<f32> = mag.iter().map(|m| m * m).collect();
    (spectral_entropy(&power), dominant_freq_hz(&power, fs))
}

/// Build a `CsiSnapshot` (2 Hz) for the dashboard's LIVE WATERFALL /
/// PER-SUBCARRIER plots.
fn make_snapshot(pipe: &Pipeline, ch: &Channel, fs: f32, seq: u32) -> CsiSnapshot {
    let mut snap = CsiSnapshot::default();
    snap.seq = seq;
    snap.rssi = ch.rssi;
    snap.snr = snr_db(ch) as i8;
    snap.csi_quality = csi_quality(snr_db(ch));
    snap.noise_floor = ch.noise_floor as f32;
    snap.n_sub = N_SUBCARRIERS as u8;

    // Interleaved I/Q from the *sanitized* phase (the linear slope is removed
    // so the dashboard's PHASE plot is the multipath structure, not CFO).
    for (i, (&a, &p)) in ch.amps.iter().zip(ch.phase.iter()).enumerate() {
        let (s, c) = p.sin_cos();
        let re = a * c;
        let im = a * s;
        snap.iq[2 * i] = (re as i16).clamp(-32768, 32767);
        snap.iq[2 * i + 1] = (im as i16).clamp(-32768, 32767);
    }

    // Baseline-referenced normalized amplitude per subcarrier (0..255): the
    // z-score clamped to ±3σ → 0..255. Without a stored baseline the raw
    // amplitude is used directly (still 0..255 via the same clamp).
    let normalizer = &pipe.normalizer;
    for (i, &a) in ch.amps.iter().enumerate() {
        let z = (a - normalizer.base[i]) / normalizer.scale[i].max(1e-6);
        let v = ((z + 3.0) / 6.0 * 255.0).clamp(0.0, 255.0);
        snap.amp_norm[i] = v as u8;
    }

    // Current motion-spectrum column (STFT magnitude, 0..255), one bin per
    // frequency. RADAR-TX accumulates these into the spectrogram.
    if !pipe.pc1_hist.is_empty() {
        let mut frame: Vec<f32> = pipe.pc1_hist.iter().copied().collect();
        while frame.len() < STFT_LEN {
            frame.push(0.0);
        }
        let mag = radar_dsp::fft::stft_frame(&frame, STFT_LEN);
        for (i, &m) in mag.iter().take(N_SPEC_BINS.min(mag.len())).enumerate() {
            snap.spec[i] = (m * 2.0).min(255.0) as u8;
        }
    }
    let _ = fs;
    snap
}
