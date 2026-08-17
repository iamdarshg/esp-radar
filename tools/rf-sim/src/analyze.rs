//! Error analyzer for the RF sim.
//!
//! Consumes the real firmware's CSI_PHASE telemetry (24 B header + 112 B
//! `[i16; 56]` mrad per-subcarrier phases) and FeatureReports as captured on
//! the wired UART1 by the QEMU harness, correlates it with the scenario's
//! ground truth by `seq` (the scenario frame index the feed stamps), and
//! reports:
//!
//! * timing     — cadence, jitter, drop rate vs the intended `rate_hz`;
//! * phase      — per-subcarrier and coherent-combined phase-increment error
//!                against the analytic floors (CRB + i8 quantization);
//! * motion     — per-frame displacement/velocity error, position-trace
//!                random walk, and windowed slope-fit range/velocity accuracy;
//! * cfo        — the measured DC phase rate vs the injected CFO (+ Doppler),
//!                and the unambiguous-velocity wrap limit;
//! * firmware   — what the firmware's own `phase_motion`/`doppler_hz` report
//!                (the CFO high-pass strips DC phase rate, so constant-velocity
//!                Doppler is intentionally absent there).
//!
//! With `--re-stamp`, `t_us` is replaced by the ideal frame grid before all
//! time-based metrics, isolating DSP/quantization error from QEMU timing
//! jitter.

use std::fs;
use std::path::Path;

use radar_protocol::{frame_type, parse_csi_phase, FeatureReport};
use radar_transport::framer::RadarFrameDecoder;

use crate::scenario::{Scenario, N_SUBC, SPEED_OF_LIGHT};

/// One decoded CSI_PHASE observation.
#[derive(Clone, Debug)]
struct PhaseFrame {
    seq: u32,
    t_us: u64,
    /// Per-subcarrier raw phase, radians.
    phase: [f64; N_SUBC],
}

fn wrap(x: f64) -> f64 {
    x - std::f64::consts::TAU * (x / std::f64::consts::TAU).round()
}

/// `#[repr(C, packed)]` FeatureReport — read straight off the wire payload.
fn parse_report(payload: &[u8]) -> Option<FeatureReport> {
    if payload.len() < core::mem::size_of::<FeatureReport>() {
        return None;
    }
    Some(unsafe { (payload.as_ptr() as *const FeatureReport).read_unaligned() })
}

/// Parse a UART1 capture file into sorted CSI_PHASE frames + FeatureReports.
fn parse_capture(path: &Path) -> std::io::Result<(Vec<PhaseFrame>, Vec<FeatureReport>)> {
    let bytes = fs::read(path)?;
    let mut dec = RadarFrameDecoder::new();
    // The decoder's buffer is bounded (2×MAX_FRAME = 1072 B) for a live UART,
    // so the whole file must not be fed at once — that would drop everything
    // except the tail. Feed in chunks, draining complete frames between each.
    // The chunk must be ≤ MAX_BUFFER − (max residual after drain). After
    // draining, the residual is a partial frame < MAX_FRAME = 536 B, so 512 B
    // chunks never overflow (512 + 535 = 1047 ≤ 1072). A 1024 B chunk would
    // overflow whenever the residual exceeds 48 B — dropping the partial frame
    // at the front of the buffer on nearly every chunk boundary.
    const CHUNK: usize = 512;
    let mut frames = Vec::new();
    for chunk in bytes.chunks(CHUNK) {
        dec.feed(chunk);
        while let Some(f) = dec.next() {
            frames.push(f);
        }
    }
    while let Some(f) = dec.next() {
        frames.push(f);
    }
    let mut phases = Vec::new();
    let mut reports = Vec::new();
    for f in frames {
        match f.kind() {
            frame_type::CSI_PHASE => {
                if let Some(cp) = parse_csi_phase(&f.payload) {
                    let mut ph = [0f64; N_SUBC];
                    for i in 0..N_SUBC {
                        ph[i] = cp.phase[i] as f64 / 1000.0;
                    }
                    phases.push(PhaseFrame {
                        seq: f.header.seq,
                        t_us: f.header.t_us,
                        phase: ph,
                    });
                }
            }
            frame_type::FEATURE_REPORT => {
                if let Some(r) = parse_report(&f.payload) {
                    reports.push(r);
                }
            }
            _ => {}
        }
    }
    phases.sort_by_key(|p| p.seq);
    phases.dedup_by_key(|p| p.seq);
    Ok((phases, reports))
}

/// Coherent combined phase increment (equal-weight conjugate-product), the
/// host analogue of `radar_dsp::transform::phase_increment` under the uniform
/// envelope the generator normalizes to.
fn combined_dphi(a: &[f64; N_SUBC], b: &[f64; N_SUBC]) -> f64 {
    let (mut s, mut c) = (0.0f64, 0.0f64);
    for k in 0..N_SUBC {
        let d = wrap(b[k] - a[k]);
        s += d.sin();
        c += d.cos();
    }
    s.atan2(c)
}

// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
pub struct Timing {
    pub n: usize,
    pub first_seq: u32,
    pub last_seq: u32,
    pub duration_s: f64,
    /// Inter-frame interval stats over consecutive-emitted (gap-1) pairs.
    pub dt_mean_us: f64,
    pub dt_std_us: f64,
    pub cadence_err_ppm: f64,
    pub delivered_rate_hz: f64,
    pub drop_rate: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Phase {
    /// Pooled per-subcarrier phase-increment error σφ (rad).
    pub sigma_phi_rad: f64,
    pub sigma_phi_floor_rad: f64,
    /// Combined (56-subcarrier) phase-increment error σΔφ (rad).
    pub sigma_dphi_rad: f64,
    pub sigma_dphi_floor_rad: f64,
    pub dphi_bias_rad: f64,
    pub n_pairs: usize,
    pub n_aliased: usize,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Motion {
    /// Per-frame displacement error σΔr (mm).
    pub sigma_dr_mm: f64,
    pub dr_floor_mm: f64,
    /// Position-trace random-walk error (mm, end-of-run).
    pub r_final_err_mm: f64,
    pub r_trace_rms_mm: f64,
    pub r_rw_floor_mm: f64,
    /// Instantaneous velocity error (mm/s), ideal cadence vs emitted cadence.
    pub sigma_v_ideal_mms: f64,
    pub sigma_v_phys_mms: f64,
    pub v_floor_mms: f64,
    /// Windowed slope-fit range/velocity accuracy (mm, mm/s).
    pub r_window_mm: f64,
    pub v_window_mms: f64,
    /// Ground-truth velocity at the first window centre (mm/s).
    pub v_gt_mms: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Cfo {
    /// Measured DC phase rate (Hz) = CFO + constant Doppler (a single RX
    /// cannot separate them).
    pub dc_rate_hz: f64,
    pub dc_rate_gt_hz: f64,
    /// Standard deviation of the CFO-removed phase increment (rad) — the
    /// residual a perfect CFO subtraction leaves.
    pub residual_rad: f64,
    pub unambiguous_limit_hz: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Firmware {
    pub n_reports: usize,
    pub phase_motion_mean: f64,
    pub doppler_hz_mean: f64,
    pub doppler_hz_std: f64,
    /// Mean ground-truth Doppler over the capture duration.
    pub doppler_hz_gt: f64,
    pub phase_motion_floor: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Analysis {
    pub timing: Timing,
    pub phase: Phase,
    pub motion: Motion,
    pub cfo: Cfo,
    pub firmware: Firmware,
}

/// Least-squares slope + intercept of `y` vs `x` (both `[f64]`).
fn line_fit(xs: &[f64], ys: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let xm = xs.iter().sum::<f64>() / n;
    let ym = ys.iter().sum::<f64>() / n;
    let mut num = 0.0;
    let mut den = 0.0;
    for (x, y) in xs.iter().zip(ys) {
        num += (x - xm) * (y - ym);
        den += (x - xm) * (x - xm);
    }
    let slope = if den.abs() > 1e-18 { num / den } else { 0.0 };
    (slope, ym - slope * xm)
}

fn std_mean(v: &[f64]) -> (f64, f64) {
    let n = v.len() as f64;
    if n == 0.0 {
        return (0.0, 0.0);
    }
    let m = v.iter().sum::<f64>() / n;
    let var = v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / n;
    (var.max(0.0).sqrt(), m)
}

/// Run the full analysis. `re_stamp` replaces `t_us` with the ideal frame
/// grid first, so every time-based metric isolates DSP error from QEMU timing
/// jitter.
pub fn analyze(sc: &Scenario, capture: &Path, re_stamp: bool) -> Result<Analysis, String> {
    let (mut frames, reports) = parse_capture(capture).map_err(|e| format!("capture: {e}"))?;
    if frames.len() < 2 {
        return Err(format!(
            "capture yielded only {} CSI_PHASE frames — nothing to analyze",
            frames.len()
        ));
    }

    if re_stamp {
        let first_us = frames[0].t_us;
        let first_seq = frames[0].seq;
        for f in frames.iter_mut() {
            f.t_us = first_us + (f.seq - first_seq) as u64 * 1_000_000 / sc.rate_hz as u64;
        }
    }

    let mut a = Analysis::default();
    a.cfo.unambiguous_limit_hz = sc.rate_hz as f64 / 2.0;

    // ---- timing ---------------------------------------------------------
    {
        let t = &mut a.timing;
        t.n = frames.len();
        t.first_seq = frames[0].seq;
        t.last_seq = frames[frames.len() - 1].seq;
        t.duration_s = (frames[frames.len() - 1].t_us - frames[0].t_us) as f64 / 1e6;
        t.delivered_rate_hz = if t.duration_s > 0.0 {
            (t.n - 1) as f64 / t.duration_s
        } else {
            0.0
        };
        let expected = (t.last_seq - t.first_seq + 1) as f64;
        t.drop_rate = if expected > 0.0 {
            1.0 - t.n as f64 / expected
        } else {
            0.0
        };
        let ideal_us = 1_000_000.0 / sc.rate_hz as f64;
        let mut dts = Vec::new();
        for w in frames.windows(2) {
            if w[1].seq == w[0].seq + 1 {
                dts.push((w[1].t_us - w[0].t_us) as f64);
            }
        }
        let (std, mean) = std_mean(&dts);
        t.dt_std_us = std;
        t.dt_mean_us = mean;
        t.cadence_err_ppm = if mean > 0.0 {
            (mean - ideal_us) / ideal_us * 1e6
        } else {
            0.0
        };
    }

    // ---- phase + motion + cfo over consecutive emitted pairs -------------
    let lambda = SPEED_OF_LIGHT / sc.fc_hz;
    let sc_floor_phi = sc.sigma_phi_floor_rad();
    let sc_floor_dphi = sc.sigma_dphi_floor_rad();
    a.phase.sigma_phi_floor_rad = sc_floor_phi;
    a.phase.sigma_dphi_floor_rad = sc_floor_dphi;

    let mut eps_phi: Vec<f64> = Vec::new();
    let mut eps_dphi: Vec<f64> = Vec::new();
    let mut eps_dr: Vec<f64> = Vec::new();
    let mut eps_v_ideal: Vec<f64> = Vec::new();
    let mut eps_v_phys: Vec<f64> = Vec::new();
    let mut dphi_meas_all: Vec<f64> = Vec::new();
    let mut dphi_gt_all: Vec<f64> = Vec::new();
    let mut r_meas = Vec::new(); // cumulative relative displacement (m)
    let mut r_gt = Vec::new();
    let mut pair_t = Vec::new(); // scenario time (s) at pair start, for windows
    let mut n_aliased = 0usize;

    let mut cum_meas = 0.0f64;
    let mut cum_gt = 0.0f64;

    for w in frames.windows(2) {
        let (p, c) = (&w[0], &w[1]);
        let gap = (c.seq - p.seq) as f64;

        // Aliasing test: a pair is aliased when the combined CFO + target
        // Doppler phase advance over the gap reaches ±π, i.e. the wrapped
        // measured increment can no longer be unwrapped by the known CFO
        // (|f_cfo + f_d|·gap ≥ rate/2). Phase metrics would stay well-defined
        // (measured and GT wrap identically), but motion/CFO need the
        // *unwrapped* advance — subtracting `TAU·cfo·gap/rate` from a wrapped
        // increment past ±π picks the wrong 2π multiple. Such pairs are
        // excluded from every accumulator below (a multi-frame gap is a QEMU
        // host-freeze artifact, not firmware pacing).
        let fd_mid =
            sc.target_doppler_hz((p.seq + c.seq) as f64 / (2.0 * sc.rate_hz as f64));
        let turns = (sc.cfo_hz + fd_mid) * gap / sc.rate_hz as f64;
        if turns.abs() >= 0.5 {
            n_aliased += 1;
            continue;
        }

        // Per-subcarrier increments (measured and GT).
        for k in 0..N_SUBC {
            let dm = wrap(c.phase[k] - p.phase[k]);
            let dg = wrap(sc.phase_clean(c.seq as u64, k) - sc.phase_clean(p.seq as u64, k));
            eps_phi.push(wrap(dm - dg));
        }
        let dphi_m = combined_dphi(&p.phase, &c.phase);
        let dphi_g = s_atan2(dph_g(&p, c, sc));
        dphi_meas_all.push(dphi_m);
        dphi_gt_all.push(dphi_g);
        eps_dphi.push(wrap(dphi_m - dphi_g));

        // Motion with CFO removed at ground truth (a single RX cannot separate
        // CFO from constant Doppler, so this is the *coherent* sensing error
        // after CFO calibration).
        let cfo_inc = std::f64::consts::TAU * sc.cfo_hz * gap / sc.rate_hz as f64;
        let dphi_tgt = wrap(dphi_m - cfo_inc);
        let dr_meas = -dphi_tgt * lambda / (4.0 * std::f64::consts::PI);
        let dr_gt = sc.target_range(c.seq as f64 / sc.rate_hz as f64)
            - sc.target_range(p.seq as f64 / sc.rate_hz as f64);
        eps_dr.push(dr_meas - dr_gt);

        let v_gt = if c.seq > p.seq {
            dr_gt * sc.rate_hz as f64 / gap
        } else {
            0.0
        };
        // ideal cadence (what the firmware's fs implies)
        eps_v_ideal.push(dr_meas * sc.rate_hz as f64 / gap - v_gt);
        // emitted cadence (physically correct if t_us is trusted)
        let dt_s = (c.t_us as f64 - p.t_us as f64) / 1e6;
        eps_v_phys.push(if dt_s > 0.0 {
            dr_meas / dt_s - v_gt
        } else {
            0.0
        });

        cum_meas += dr_meas;
        cum_gt += dr_gt;
        r_meas.push(cum_meas);
        r_gt.push(cum_gt);
        pair_t.push(p.seq as f64 / sc.rate_hz as f64);
    }

    // ---- phase metrics ----------------------------------------------------
    {
        let ph = &mut a.phase;
        let (std, mean) = std_mean(&eps_phi);
        ph.sigma_phi_rad = std;
        let (s2, _) = std_mean(&eps_dphi);
        ph.sigma_dphi_rad = s2;
        ph.dphi_bias_rad = mean;
        ph.n_pairs = eps_dphi.len();
        ph.n_aliased = n_aliased;
    }

    // ---- motion metrics ---------------------------------------------------
    {
        let m = &mut a.motion;
        let (s_dr, _) = std_mean(&eps_dr);
        m.sigma_dr_mm = s_dr * 1000.0;
        m.dr_floor_mm = sc_floor_dphi * lambda / (4.0 * std::f64::consts::PI) * 1000.0;
        let (s_vi, _) = std_mean(&eps_v_ideal);
        let (s_vp, _) = std_mean(&eps_v_phys);
        m.sigma_v_ideal_mms = s_vi * 1000.0;
        m.sigma_v_phys_mms = s_vp * 1000.0;
        m.v_floor_mms = sc_floor_dphi * sc.rate_hz as f64 * lambda / (4.0 * std::f64::consts::PI) * 1000.0;
        // Position trace error (second half to skip the start transient).
        let half = r_meas.len() / 2;
        let mut err = Vec::new();
        for (rm, rg) in r_meas.iter().zip(&r_gt).skip(half) {
            err.push((rm - rg).abs());
        }
        let rms = if err.is_empty() {
            0.0
        } else {
            (err.iter().map(|e| e * e).sum::<f64>() / err.len() as f64).sqrt()
        };
        m.r_trace_rms_mm = rms * 1000.0;
        m.r_final_err_mm = (r_meas.last().unwrap_or(&0.0) - r_gt.last().unwrap_or(&0.0)).abs() * 1000.0;
        m.r_rw_floor_mm = m.sigma_dr_mm * (eps_dr.len() as f64).sqrt();
        // Windowed slope fit: slide a W-frame window over the integrated
        // trace; the least-squares slope is a windowed velocity estimate and
        // the intercept is a smoothed range estimate. Both are compared to the
        // GT line fit inside the same window.
        let win = (64usize).min(r_meas.len());
        let mut rw = Vec::new();
        let mut vw = Vec::new();
        let mut gt_v_first = 0.0f64;
        let mut nwin = 0usize;
        let step = win.max(1);
        let mut start = 0usize;
        while start + win <= r_meas.len() {
            let xs: Vec<f64> = pair_t[start..start + win].to_vec();
            let ys: Vec<f64> = r_meas[start..start + win].to_vec();
            let gt_ys: Vec<f64> = r_gt[start..start + win].to_vec();
            let (slope, inter) = line_fit(&xs, &ys);
            let (gt_slope, gt_inter) = line_fit(&xs, &gt_ys);
            if nwin == 0 {
                // Ground-truth velocity at the first window's midpoint.
                let mid = pair_t[start + win / 2];
                gt_v_first = sc.target_velocity(mid);
            }
            rw.push((inter - gt_inter).abs());
            vw.push((slope - gt_slope).abs());
            nwin += 1;
            start += step;
        }
        m.r_window_mm = if nwin > 0 {
            rw.iter().sum::<f64>() / nwin as f64 * 1000.0
        } else {
            0.0
        };
        m.v_window_mms = if nwin > 0 {
            vw.iter().sum::<f64>() / nwin as f64 * 1000.0
        } else {
            0.0
        };
        m.v_gt_mms = gt_v_first * 1000.0;
    }

    // ---- CFO --------------------------------------------------------------
    {
        let cfo = &mut a.cfo;
        let n = dphi_meas_all.len() as f64;
        let mean_rate = if n > 0.0 {
            dphi_meas_all.iter().sum::<f64>() / n * sc.rate_hz as f64 / std::f64::consts::TAU
        } else {
            0.0
        };
        cfo.dc_rate_hz = mean_rate;
        // GT DC = CFO + mean target Doppler over the run.
        let (_, dg_mean) = std_mean(&dphi_gt_all);
        cfo.dc_rate_gt_hz = dg_mean * sc.rate_hz as f64 / std::f64::consts::TAU;
        // Residual after removing the DC: std of the detrended combined increment.
        let mut det: Vec<f64> = Vec::new();
        for d in &dphi_meas_all {
            det.push(wrap(d - mean_rate * std::f64::consts::TAU / sc.rate_hz as f64));
        }
        let (s, _) = std_mean(&det);
        cfo.residual_rad = s;
    }

    // ---- firmware output --------------------------------------------------
    {
        let fw = &mut a.firmware;
        fw.n_reports = reports.len();
        if !reports.is_empty() {
            let pm: Vec<f64> = reports.iter().map(|r| r.phase_motion as f64).collect();
            let dh: Vec<f64> = reports.iter().map(|r| r.doppler_hz as f64).collect();
            let (_, m) = std_mean(&pm);
            fw.phase_motion_mean = m;
            let (s, m2) = std_mean(&dh);
            fw.doppler_hz_mean = m2;
            fw.doppler_hz_std = s;
        }
        fw.phase_motion_floor = sc_floor_dphi;
        // Mean GT doppler over the capture: constant-velocity doppler is a DC
        // phase-rate term, so it is what the firmware's CFO high-pass removes.
        if !frames.is_empty() {
            let t0 = frames[0].seq as f64 / sc.rate_hz as f64;
            let t1 = frames[frames.len() - 1].seq as f64 / sc.rate_hz as f64;
            let mut sum = 0.0;
            let n = 512usize.max(1);
            for i in 0..n {
                let t = t0 + (t1 - t0) * i as f64 / n as f64;
                sum += sc.target_doppler_hz(t);
            }
            fw.doppler_hz_gt = sum / n as f64;
        }
    }

    Ok(a)
}

/// Combined increment from per-subcarrier GT phases (coherent sum, like the
/// measurement).
fn s_atan2(d: [f64; N_SUBC]) -> f64 {
    let (mut s, mut c) = (0.0f64, 0.0f64);
    for k in 0..N_SUBC {
        s += d[k].sin();
        c += d[k].cos();
    }
    s.atan2(c)
}

/// Per-subcarrier ground-truth phase increment between two frames.
fn dph_g(a: &PhaseFrame, b: &PhaseFrame, sc: &Scenario) -> [f64; N_SUBC] {
    let mut out = [0f64; N_SUBC];
    for k in 0..N_SUBC {
        out[k] = wrap(sc.phase_clean(b.seq as u64, k) - sc.phase_clean(a.seq as u64, k));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::{PropPath, Scenario, Trajectory};

    fn lo_scenario() -> Scenario {
        Scenario {
            version: 1,
            name: "test-los".into(),
            rate_hz: 200,
            n_frames: 100,
            fc_hz: 2.437e9,
            sub_spacing_hz: 312_500.0,
            amp: 100.0,
            snr_db: 30.0,
            cfo_hz: 50.0,
            rssi_dbm: -52,
            noise_floor_dbm: -96,
            channel: 6,
            bssid: [36, 15, 40, 1, 2, 3],
            trajectory: Trajectory::ConstVel { v_m_s: 0.35 },
            paths: vec![PropPath {
                amp: 1.0,
                delay_m: 2.0,
                v_m_s: 0.35,
            }],
            target_path: 0,
            seed: 7,
        }
    }

    #[test]
    fn combined_increment_matches_gt_within_noise() {
        let sc = lo_scenario();
        // Noise-free synthesized measurement: firmware quantizes to mrad, so
        // the only error is the 0.5 mrad rounding. Build a fake PhaseFrame
        // stream from the GT phases and check σΔφ is tiny.
        let mut frames = Vec::new();
        for n in 0..10u32 {
            let mut ph = [0f64; N_SUBC];
            for k in 0..N_SUBC {
                let q = (sc.phase_clean(n as u64, k) * 1000.0).round() / 1000.0;
                ph[k] = q;
            }
            frames.push(PhaseFrame {
                seq: n,
                t_us: n as u64 * 5_000,
                phase: ph,
            });
        }
        // reuse the pairwise math via a small manual accumulation
        let mut errs = Vec::new();
        for w in frames.windows(2) {
            let dm = combined_dphi(&w[0].phase, &w[1].phase);
            let mut dg = [0f64; N_SUBC];
            for k in 0..N_SUBC {
                dg[k] = wrap(sc.phase_clean(w[1].seq as u64, k) - sc.phase_clean(w[0].seq as u64, k));
            }
            errs.push(wrap(dm - s_atan2(dg)));
        }
        let (s, _) = std_mean(&errs);
        // mrad rounding → ~0.5 mrad/√56 of quantization + tiny CFO phase step.
        assert!(s < 2e-4, "sigma {s}");
    }
}
