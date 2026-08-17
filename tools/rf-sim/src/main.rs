//! RF-sim: RF scenario generator + error analyzer for the ESP-RADAR head.
//!
//! ```text
//! rf-sim gen <scenario.json> <simdata.bin>          # build a burnable blob
//! rf-sim analyze <scenario.json> <uart1-capture.log> [--re-stamp] [--json]
//! ```
//!
//! `gen` writes a simdata blob in the exact `firmware/radar_rx/src/sim.rs`
//! format for the QEMU harness to flash at the `simdata` partition. `analyze`
//! parses the real firmware's emitted CSI_PHASE telemetry off the wired UART
//! capture and reports phase / displacement / velocity / CFO / timing errors
//! against the analytic floors — the ground truth being recomputed from the
//! scenario JSON alone (its `seed` makes the blob bit-reproducible).
//!
//! See `docs/rf-sim-design.md` for the full error-budget writeup.
//!
//! The scenario model / generator / analyzer live in the library targets
//! (`rf_sim::scenario`, `rf_sim::analyze`) so `tests/` can round-trip a
//! generated blob through the real firmware DSP.

use std::path::Path;
use std::process::ExitCode;

use rf_sim::{analyze, scenario};
use rf_sim::scenario::Scenario;

fn usage() -> ! {
    eprintln!(
        "usage:\n  rf-sim gen <scenario.json> <simdata.bin>\n  \
         rf-sim analyze <scenario.json> <uart1-capture.log> [--re-stamp] [--json]"
    );
    std::process::exit(2);
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        usage();
    }
    match args[1].as_str() {
        "gen" => {
            if args.len() != 4 {
                usage();
            }
            let sc: Scenario = match load_scenario(Path::new(&args[2])) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("scenario: {e}");
                    return ExitCode::FAILURE;
                }
            };
            match scenario::generate(&sc, Path::new(&args[3])) {
                Ok(()) => {
                    let frames = sc.n_frames;
                    println!(
                        "{}: {} frames @ {} Hz ({:.1} KB) -> {}",
                        sc.name,
                        frames,
                        sc.rate_hz,
                        (32 + frames as usize * 128) as f64 / 1024.0,
                        args[3]
                    );
                    ExitCode::SUCCESS
                }
                Err(e) => {
                    eprintln!("write: {e}");
                    ExitCode::FAILURE
                }
            }
        }
        "analyze" => {
            if args.len() < 4 {
                usage();
            }
            let re_stamp = args.iter().any(|a| a == "--re-stamp");
            let json = args.iter().any(|a| a == "--json");
            let sc = match load_scenario(Path::new(&args[2])) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("scenario: {e}");
                    return ExitCode::FAILURE;
                }
            };
            match analyze::analyze(&sc, Path::new(&args[3]), re_stamp) {
                Ok(a) => {
                    if json {
                        print_json(&sc, &a);
                    } else {
                        print_report(&sc, &a, re_stamp);
                    }
                    ExitCode::SUCCESS
                }
                Err(e) => {
                    eprintln!("analyze: {e}");
                    ExitCode::FAILURE
                }
            }
        }
        _ => usage(),
    }
}

fn load_scenario(path: &Path) -> Result<Scenario, String> {
    let s = std::fs::read_to_string(path).map_err(|e| format!("read {path:?}: {e}"))?;
    serde_json::from_str(&s).map_err(|e| format!("parse {path:?}: {e}"))
}

fn print_report(sc: &Scenario, a: &analyze::Analysis, re_stamp: bool) {
    let t = &a.timing;
    let p = &a.phase;
    let m = &a.motion;
    let c = &a.cfo;
    let f = &a.firmware;
    let lambda_mm = scenario::SPEED_OF_LIGHT / sc.fc_hz * 1000.0;

    println!("== rf-sim analysis: {} ==", sc.name);
    println!(
        "   channel model: fc={:.1} MHz  rate={} Hz  amp={:.0}  SNR={:.0} dB  CFO={:.0} Hz",
        sc.fc_hz / 1e6,
        sc.rate_hz,
        sc.amp,
        sc.snr_db,
        sc.cfo_hz
    );
    println!(
        "   lambda={:.1} mm  sub-carriers=56  floor σφ={:.2} mrad  σΔφ={:.2} mrad",
        lambda_mm,
        p.sigma_phi_floor_rad * 1e3,
        p.sigma_dphi_floor_rad * 1e3
    );

    println!("  timing:");
    println!(
        "   frames={}  seq {}..{}  duration={:.2} s  cadence err={:.0} ppm  jitter σt={:.2} us  drop={:.2}%",
        t.n,
        t.first_seq,
        t.last_seq,
        t.duration_s,
        t.cadence_err_ppm,
        t.dt_std_us,
        t.drop_rate * 100.0
    );

    println!("  phase:");
    println!(
        "   σφ (per subcarrier) = {:.3} mrad   [floor {:.3}]   ratio {:.2}x",
        p.sigma_phi_rad * 1e3,
        p.sigma_phi_floor_rad * 1e3,
        p.sigma_phi_rad / p.sigma_phi_floor_rad.max(1e-12)
    );
    println!(
        "   σΔφ (combined 56)   = {:.3} mrad   [floor {:.3}]   ratio {:.2}x   bias {:.2} urad",
        p.sigma_dphi_rad * 1e3,
        p.sigma_dphi_floor_rad * 1e3,
        p.sigma_dphi_rad / p.sigma_dphi_floor_rad.max(1e-12),
        p.dphi_bias_rad * 1e6
    );
    if p.n_aliased > 0 {
        println!(
            "   ⚠  {} / {} phase pairs aliased: |Δφ_GT| > π (|f_cfo + f_d| > {:.0} Hz wrap) — errors below exclude them",
            p.n_aliased, p.n_pairs, c.unambiguous_limit_hz
        );
    }

    println!("  motion (CFO removed at ground truth):");
    println!(
        "   σΔr = {:.3} mm/frame  [floor {:.3}]",
        m.sigma_dr_mm, m.dr_floor_mm
    );
    println!(
        "   position trace: end-of-run |err| = {:.2} mm  (random-walk floor {:.2} mm),  RMS 2nd-half = {:.2} mm",
        m.r_final_err_mm, m.r_rw_floor_mm, m.r_trace_rms_mm
    );
    println!(
        "   velocity: σv(ideal cadence) = {:.3} mm/s  σv(emitted) = {:.3} mm/s  [floor {:.3}]  {}",
        m.sigma_v_ideal_mms,
        m.sigma_v_phys_mms,
        m.v_floor_mms,
        if re_stamp { "(re-stamped)" } else { "" }
    );
    println!(
        "   windowed slope-fit: σr = {:.3} mm,  σv = {:.3} mm/s   (GT v at t0 = {:.1} mm/s)",
        m.r_window_mm, m.v_window_mms, m.v_gt_mms
    );

    println!("  cfo:");
    println!(
        "   DC phase rate: meas {:.2} Hz vs GT {:.2} Hz  (CFO {:.0} + doppler; single RX cannot split them)",
        c.dc_rate_hz, c.dc_rate_gt_hz, sc.cfo_hz
    );
    println!(
        "   residual after DC removal: σΔφ = {:.2} mrad   unambiguous limit |f_cfo+f_d| < {:.0} Hz",
        c.residual_rad * 1e3,
        c.unambiguous_limit_hz
    );

    if f.n_reports > 0 {
        println!("  firmware output ({} FeatureReports):", f.n_reports);
        println!(
            "   phase_motion = {:.3} mrad  (noise floor {:.3})   doppler_hz = {:.2} ± {:.2} Hz  (GT doppler {:.2})",
            f.phase_motion_mean * 1e3,
            f.phase_motion_floor * 1e3,
            f.doppler_hz_mean,
            f.doppler_hz_std,
            f.doppler_hz_gt
        );
        println!(
            "   note: the firmware's CFO high-pass strips the DC phase rate, so constant-velocity doppler and CFO both vanish from doppler_hz (it tracks only non-DC motion)"
        );
    } else {
        println!("  firmware output: no FeatureReports captured");
    }
    println!();
}

fn print_json(sc: &Scenario, a: &analyze::Analysis) {
    // One flat object the sweep harness can column-append.
    let v_m_s = match &sc.trajectory {
        scenario::Trajectory::ConstVel { v_m_s } => *v_m_s,
        _ => 0.0,
    };
    let obj = serde_json::json!({
        "name": sc.name,
        "rate_hz": sc.rate_hz,
        "n_frames": sc.n_frames,
        "amp": sc.amp,
        "snr_db": sc.snr_db,
        "cfo_hz": sc.cfo_hz,
        "v_m_s": v_m_s,
        "frames": a.timing.n,
        "drop_rate": a.timing.drop_rate,
        "dt_std_us": a.timing.dt_std_us,
        "cadence_err_ppm": a.timing.cadence_err_ppm,
        "sigma_phi_mrad": a.phase.sigma_phi_rad * 1e3,
        "sigma_phi_floor_mrad": a.phase.sigma_phi_floor_rad * 1e3,
        "sigma_dphi_mrad": a.phase.sigma_dphi_rad * 1e3,
        "sigma_dphi_floor_mrad": a.phase.sigma_dphi_floor_rad * 1e3,
        "n_aliased": a.phase.n_aliased,
        "sigma_dr_mm": a.motion.sigma_dr_mm,
        "dr_floor_mm": a.motion.dr_floor_mm,
        "r_final_err_mm": a.motion.r_final_err_mm,
        "r_trace_rms_mm": a.motion.r_trace_rms_mm,
        "r_rw_floor_mm": a.motion.r_rw_floor_mm,
        "sigma_v_ideal_mms": a.motion.sigma_v_ideal_mms,
        "sigma_v_phys_mms": a.motion.sigma_v_phys_mms,
        "v_floor_mms": a.motion.v_floor_mms,
        "r_window_mm": a.motion.r_window_mm,
        "v_window_mms": a.motion.v_window_mms,
        "dc_rate_hz": a.cfo.dc_rate_hz,
        "dc_rate_gt_hz": a.cfo.dc_rate_gt_hz,
        "phase_motion_mrad": a.firmware.phase_motion_mean * 1e3,
        "doppler_hz": a.firmware.doppler_hz_mean,
    });
    println!("{obj}");
}
