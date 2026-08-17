//! Calibration state machine (spec §17).
//!
//! Runs inside the fusion/controller task, which owns the two wired UART
//! links where CalResps land. CalCmds are written down **both** links (each RX
//! reads its own). While a stage is active, the measurement traffic stamps its
//! frames with the CAL flag so the receivers route CSI to their calibration
//! collectors instead of the live pipeline.
//!
//! Stages (§17):
//!   CAL 1  identity + link check      (broadcast START, wait for ACKs)
//!   CAL 2  TX power ↔ RSSI sweep      (per-power COLLECT, fit model, commission)
//!   CAL 3  empty-room baseline        (each RX collects its own B1/B2 in NVS)
//!   CAL 4  moving-person thresholds   (TX observes fused energy over a window)
//!   CAL 5  fingerprint (optional)     (capture + log)
//!
//! Power cycling must not require re-calibration: everything produced here is
//! persisted to NVS and reloaded at boot (§17).

use core::mem;

use radar_calibration::{ClassThresholds, SweepPoint, ThresholdSource, TxPowerModel};
use radar_protocol::{CalResp, cal_action, cal_stage, node};
use radar_storage::nvs::Nvs;
use radar_transport::udp::now_us;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

use crate::wired::WiredLink;

/// RSSI the CAL 2 sweep targets: the highest TX power whose *worst* receiver
/// stays below this is commissioned (spec §5/§6).
pub const TARGET_RSSI_DBM: f32 = -45.0;
/// CAL 2 sweep points (dBm). The head's geometry is centimetre-scale, so the
/// whole sweep stays modest — receivers saturate long before 20 dBm.
pub const SWEEP_POWERS_DBM: [i16; 5] = [4, 8, 12, 16, 20];
/// Collection window per sweep point (ms).
const SWEEP_COLLECT_MS: u32 = 800;
/// How long to wait for both CalResps after a sweep point.
const SWEEP_TIMEOUT_US: u64 = 2_000_000;
/// How long to wait for both links to answer a broadcast command.
const ACK_TIMEOUT_US: u64 = 3_000_000;
/// CAL 3 empty-room collection window (ms).
const CAL3_WINDOW_MS: u32 = 10_000;
/// CAL 4 moving-person collection window (µs).
const CAL4_WINDOW_US: u64 = 15_000_000;
/// CAL 5 fingerprint capture window (ms).
const CAL5_WINDOW_MS: u32 = 5_000;
/// Saturation score at which the sweep stops (pointless to go higher).
const SAT_STOP: u8 = 70;
/// Maximum TX power we will commission to (dBm).
pub const MAX_TX_POWER_DBM: i16 = 20;
/// TX power used before commissioning (dBm).
pub const DEFAULT_TX_POWER_DBM: u8 = 6;

/// Commands the controller feeds into the state machine (from the `/cal` HTTP
/// endpoint or the boot auto-commission path).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalCommand {
    /// Start a calibration stage (`radar_protocol::cal_stage::*`).
    StartStage(u8),
    /// Boot-time: run CAL 2 unless a power model is already stored.
    AutoCommission,
    /// Abort any running stage.
    Abort,
}

/// In-flight response wait: both RX links answered, or the deadline elapsed.
#[derive(Clone, Copy, Debug)]
struct Wait {
    deadline_us: u64,
    got_rx1: bool,
    got_rx2: bool,
    resp_rx1: Option<CalResp>,
    resp_rx2: Option<CalResp>,
}

impl Wait {
    fn new(deadline_us: u64) -> Self {
        Self { deadline_us, got_rx1: false, got_rx2: false, resp_rx1: None, resp_rx2: None }
    }

    fn push(&mut self, src: u8, resp: CalResp) {
        if src == node::RX1 {
            self.resp_rx1 = Some(resp);
            self.got_rx1 = true;
        } else if src == node::RX2 {
            self.resp_rx2 = Some(resp);
            self.got_rx2 = true;
        }
    }

    fn both(&self) -> bool {
        self.got_rx1 && self.got_rx2
    }
    fn any(&self) -> bool {
        self.got_rx1 || self.got_rx2
    }

    /// Worst-case values across whichever links answered: the most exposed
    /// receiver drives the commissioning decision.
    fn rssi(&self) -> i16 {
        self.resp_rx1.iter().map(|r| r.rssi).chain(self.resp_rx2.iter().map(|r| r.rssi)).max().unwrap_or(0)
    }
    fn sat(&self) -> u8 {
        self.resp_rx1.iter().map(|r| r.sat_score).chain(self.resp_rx2.iter().map(|r| r.sat_score)).max().unwrap_or(0)
    }
    fn snr(&self) -> i8 {
        self.resp_rx1.iter().map(|r| r.snr).chain(self.resp_rx2.iter().map(|r| r.snr)).min().unwrap_or(0)
    }
    fn quality(&self) -> u8 {
        self.resp_rx1.iter().map(|r| r.csi_quality).chain(self.resp_rx2.iter().map(|r| r.csi_quality)).min().unwrap_or(0)
    }
    fn dyn_range(&self) -> u8 {
        self.resp_rx1.iter().map(|r| r.dyn_range).chain(self.resp_rx2.iter().map(|r| r.dyn_range)).min().unwrap_or(0)
    }
}

enum Phase {
    Idle,
    /// Waiting for both links to answer a broadcast command (CAL 1/3/5).
    WaitResp { stage: u8, wait: Wait },
    /// CAL 2 sweep: `points[0..=idx]` recorded, waiting on the current one.
    Cal2 { points: Vec<SweepPoint>, idx: usize, wait: Wait },
    /// CAL 4: collecting fused energy over a timed window.
    Cal4 { window_start_us: u64, window_us: u64 },
    Done,
}

pub struct Calibrator {
    phase: Phase,
    stage: u8,
    active: bool,
    has_model: bool,
    pending_thresholds: Option<ClassThresholds>,
    // CAL 4 fused-energy histogram: 200 bins × 0.05 → covers 0..10 energy.
    cal4_hist: [u32; 200],
    cal4_count: u32,
    cal4_max_energy: f32,
    cal4_max_dev: f32,
}

impl Calibrator {
    pub fn new() -> Self {
        Self {
            phase: Phase::Idle,
            stage: 0,
            active: false,
            has_model: false,
            pending_thresholds: None,
            cal4_hist: [0; 200],
            cal4_count: 0,
            cal4_max_energy: 0.0,
            cal4_max_dev: 0.0,
        }
    }

    /// Set whether a power model already exists (from NVS at boot) so the
    /// auto-commission knows whether CAL 2 is needed.
    pub fn mark_commissioned(&mut self, has_model: bool) {
        self.has_model = has_model;
    }

    pub fn current_stage(&self) -> u8 {
        self.stage
    }
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Take any thresholds produced by a finished CAL 4 (once).
    pub fn take_thresholds(&mut self) -> Option<ClassThresholds> {
        self.pending_thresholds.take()
    }

    pub fn start(&mut self, cmd: CalCommand, links: &mut [WiredLink; 2]) {
        match cmd {
            CalCommand::Abort => {
                log::warn!("calibration aborted");
                self.phase = Phase::Idle;
                self.stage = 0;
                self.active = false;
            }
            CalCommand::StartStage(s) => self.begin_stage(s, links),
            CalCommand::AutoCommission => {
                if self.has_model {
                    log::info!("power model already present; auto CAL 2 skipped");
                } else {
                    log::info!("auto-commission: starting CAL 2");
                    self.begin_stage(cal_stage::RF_POWER, links);
                }
            }
        }
    }

    /// Feed a freshly-fused pair during a running CAL 4.
    pub fn on_pair(&mut self, fused_energy: f32, baseline_dev: f32) {
        if let Phase::Cal4 { .. } = self.phase {
            self.cal4_count += 1;
            let idx = ((fused_energy / 0.05) as usize).min(self.cal4_hist.len() - 1);
            self.cal4_hist[idx] += 1;
            if fused_energy > self.cal4_max_energy {
                self.cal4_max_energy = fused_energy;
            }
            if baseline_dev > self.cal4_max_dev {
                self.cal4_max_dev = baseline_dev;
            }
        }
    }

    /// Record a calibration response (dropped when not expecting one).
    pub fn on_resp(&mut self, src: u8, resp: CalResp) {
        match &mut self.phase {
            Phase::WaitResp { wait, .. } => wait.push(src, resp),
            Phase::Cal2 { wait, .. } => wait.push(src, resp),
            _ => {
                log::debug!("stray CalResp (stage={} result={})", resp.stage, resp.result);
            }
        }
    }

    /// Advance the state machine. `tx_power`/`nvs` are used only when a stage
    /// finishes (commissioning / persistence).
    pub fn tick(&mut self, now: u64, tx_power: &Arc<AtomicU8>, nvs: &Nvs, links: &mut [WiredLink; 2]) {
        let phase = mem::replace(&mut self.phase, Phase::Idle);
        self.phase = match phase {
            Phase::Idle => Phase::Idle,

            Phase::WaitResp { stage, wait } => {
                if now >= wait.deadline_us {
                    self.finish_ack_stage(stage);
                    self.active = false;
                    self.stage = 0;
                    Phase::Done
                } else {
                    Phase::WaitResp { stage, wait }
                }
            }

            Phase::Cal2 { mut points, mut idx, wait } => {
                let ready = wait.both() || now >= wait.deadline_us;
                if ready {
                    if wait.any() {
                        points.push(SweepPoint {
                            tx_power_db: SWEEP_POWERS_DBM[idx],
                            rssi: wait.rssi(),
                            snr: wait.snr(),
                            csi_quality: wait.quality(),
                            sat_score: wait.sat(),
                            dyn_range: wait.dyn_range(),
                        });
                    }
                    let saturated = wait.sat() >= SAT_STOP;
                    if idx + 1 >= SWEEP_POWERS_DBM.len() || saturated {
                        self.finish_power_sweep(&points, tx_power, nvs);
                        self.active = false;
                        self.stage = 0;
                        Phase::Done
                    } else {
                        idx += 1;
                        let p = SWEEP_POWERS_DBM[idx];
                        set_tx_power(p);
                        self.broadcast_cmd(cal_stage::RF_POWER, cal_action::COLLECT, SWEEP_COLLECT_MS, p, links);
                        Phase::Cal2 { points, idx, wait: Wait::new(now_us() + SWEEP_TIMEOUT_US) }
                    }
                } else {
                    Phase::Cal2 { points, idx, wait }
                }
            }

            Phase::Cal4 { window_start_us, window_us } => {
                if now >= window_start_us + window_us {
                    let t = self.derive_thresholds();
                    self.pending_thresholds = Some(t);
                    self.active = false;
                    self.stage = 0;
                    log::info!(
                        "CAL 4 complete: move={:.2} strong={:.2} empty={:.3} static={:.2} ({} samples)",
                        t.move_thresh,
                        t.strong_thresh,
                        t.empty_thresh,
                        t.static_thresh,
                        self.cal4_count
                    );
                    Phase::Done
                } else {
                    Phase::Cal4 { window_start_us, window_us }
                }
            }

            Phase::Done => Phase::Done,
        };
    }

    // -- stage drivers -------------------------------------------------------

    fn begin_stage(&mut self, stage: u8, links: &mut [WiredLink; 2]) {
        match stage {
            cal_stage::IDENTITY => {
                self.stage = stage;
                self.active = true;
                self.phase = Phase::WaitResp { stage, wait: Wait::new(now_us() + ACK_TIMEOUT_US) };
                self.broadcast_cmd(stage, cal_action::START, 0, 0, links);
                log::info!("CAL 1 (identity): probing RX1/RX2");
            }
            cal_stage::RF_POWER => {
                self.stage = stage;
                self.active = true;
                self.phase = Phase::Cal2 {
                    points: Vec::new(),
                    idx: 0,
                    wait: Wait::new(now_us() + SWEEP_TIMEOUT_US),
                };
                let p = SWEEP_POWERS_DBM[0];
                set_tx_power(p);
                self.broadcast_cmd(stage, cal_action::COLLECT, SWEEP_COLLECT_MS, p, links);
                log::info!("CAL 2 (RF power): sweeping TX power {:?}", &SWEEP_POWERS_DBM[..]);
            }
            cal_stage::EMPTY_ROOM => {
                self.stage = stage;
                self.active = true;
                self.phase = Phase::WaitResp {
                    stage,
                    wait: Wait::new(now_us() + ACK_TIMEOUT_US + (CAL3_WINDOW_MS as u64) * 1000),
                };
                self.broadcast_cmd(stage, cal_action::COLLECT, CAL3_WINDOW_MS, 0, links);
                log::info!("CAL 3 (empty room): collecting {} s of baseline on both links", CAL3_WINDOW_MS / 1000);
            }
            cal_stage::MOVING_TEST => {
                self.stage = stage;
                self.active = true;
                self.cal4_hist = [0; 200];
                self.cal4_count = 0;
                self.cal4_max_energy = 0.0;
                self.cal4_max_dev = 0.0;
                self.phase = Phase::Cal4 { window_start_us: now_us(), window_us: CAL4_WINDOW_US };
                self.broadcast_cmd(stage, cal_action::COLLECT, (CAL4_WINDOW_US / 1000) as u32, 0, links);
                log::info!("CAL 4 (moving test): observe movement for {} s", CAL4_WINDOW_US / 1_000_000);
            }
            cal_stage::FINGERPRINT => {
                self.stage = stage;
                self.active = true;
                self.phase = Phase::WaitResp {
                    stage,
                    wait: Wait::new(now_us() + ACK_TIMEOUT_US + (CAL5_WINDOW_MS as u64) * 1000),
                };
                self.broadcast_cmd(stage, cal_action::COLLECT, CAL5_WINDOW_MS, 0, links);
                log::info!("CAL 5 (fingerprint): capturing");
            }
            other => {
                log::warn!("unknown calibration stage {other}");
            }
        }
    }

    fn finish_ack_stage(&mut self, stage: u8) {
        match stage {
            cal_stage::IDENTITY => log::info!("CAL 1 done: links probed"),
            cal_stage::EMPTY_ROOM => log::info!("CAL 3 done: empty-room baselines recorded by both RX links"),
            cal_stage::FINGERPRINT => log::info!("CAL 5 done: fingerprint captured"),
            _ => {}
        }
    }

    fn finish_power_sweep(&mut self, points: &[SweepPoint], tx_power: &Arc<AtomicU8>, nvs: &Nvs) {
        let model = match TxPowerModel::fit(points) {
            Some(m) => m,
            None => {
                log::warn!("CAL 2 failed: not enough sweep points ({})", points.len());
                return;
            }
        };
        let power = model.power_for_rssi(TARGET_RSSI_DBM).clamp(4, MAX_TX_POWER_DBM);
        set_tx_power(power);
        tx_power.store(power as u8, Ordering::Relaxed);
        self.has_model = true;
        if let Err(e) = nvs.store_power_model(&model) {
            log::warn!("could not store power model: {e}");
        }
        // Persist the commissioned power so boot doesn't re-run CAL 2.
        let mut cfg = nvs.load_config().unwrap_or_default();
        cfg.tx_power_db = power as u8;
        if let Err(e) = nvs.store_config(&cfg) {
            log::warn!("could not store commissioned power: {e}");
        }
        log::info!(
            "CAL 2 done: slope={:.2} dB/dB r2={:.2} ({} points); commissioning {power} dBm",
            model.slope,
            model.r2,
            model.n_points
        );
    }

    /// Turn the CAL 4 energy histogram into classifier thresholds. Falls back
    /// to factory defaults when no meaningful motion was captured.
    fn derive_thresholds(&self) -> ClassThresholds {
        let n = self.cal4_count.max(1);
        let quiet = self.percentile(0.05);
        let motion = self.percentile(0.95).max(self.cal4_max_energy * 0.5);
        if n < 50 || (motion - quiet) < 0.25 {
            log::warn!(
                "CAL 4: no meaningful motion captured (n={n}, quiet={quiet:.2}, motion={motion:.2}); keeping defaults"
            );
            return ClassThresholds::default();
        }
        let move_thresh = (0.30 * motion).clamp(0.15, 1.5);
        let strong_thresh = (move_thresh * 3.0).max(motion * 0.85);
        let empty_thresh = (quiet * 2.5).clamp(0.01, 0.06);
        let static_thresh = (self.cal4_max_dev * 2.5).clamp(0.5, 4.0);
        ClassThresholds {
            empty_thresh,
            move_thresh,
            strong_thresh,
            static_thresh,
            hold_frames: 6,
            source: ThresholdSource::Calibrated,
        }
    }

    fn percentile(&self, q: f32) -> f32 {
        let target = ((self.cal4_count as f32 * q) as u32).max(1);
        let mut cum = 0u32;
        for (i, &c) in self.cal4_hist.iter().enumerate() {
            cum += c;
            if cum >= target {
                return i as f32 * 0.05;
            }
        }
        (self.cal4_hist.len() as f32 - 1.0) * 0.05
    }

    // -- wire helpers --------------------------------------------------------

    fn broadcast_cmd(
        &mut self,
        stage: u8,
        action: u8,
        collect_ms: u32,
        tx_power_db: i16,
        links: &mut [WiredLink; 2],
    ) {
        // Every RX listens on its own wired link now; write the command down
        // both. Fire-and-forget — the Wait phase handles timeouts.
        for link in links.iter_mut() {
            if let Err(e) = link.send_cal_cmd(stage, action, collect_ms, tx_power_db) {
                log::warn!("cal command write failed: {e}");
            }
        }
    }
}

/// Set the global Wi-Fi TX power (quarter-dBm units). Callable from any task.
pub(crate) fn set_tx_power(power_db: i16) {
    let quarter = (power_db.clamp(4, MAX_TX_POWER_DBM) * 4) as i8;
    let rc = unsafe { esp_idf_sys::esp_wifi_set_max_tx_power(quarter) };
    if rc != esp_idf_sys::ESP_OK {
        log::warn!("esp_wifi_set_max_tx_power({power_db} dBm) failed: 0x{rc:08x}");
    }
}
