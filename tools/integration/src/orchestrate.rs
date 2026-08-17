//! Orchestrator: spawns the TX / RX1 / RX2 roles as three child processes,
//! aligns their start (READY/GO handshake), waits out the run window, then
//! asserts the RATE-1/2/3 flows and exits non-zero on any failure.

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::common::{get_str, get_u64, parse_summary, Config, Role, REPORT_EVERY};

/// A spawned role child and its captured stdout lines.
struct ChildHandle {
    role: Role,
    child: Child,
    stdout: Arc<Mutex<Vec<String>>>,
    /// Detached stdout-drain thread; joined before the buffers are scanned so
    /// every line (including the final SUMMARY line) is present. `Option` so the
    /// handle can be `take()`n out (join consumes it).
    reader: Option<JoinHandle<()>>,
}

const READY_TIMEOUT: Duration = Duration::from_secs(6);
const EXIT_GRACE: Duration = Duration::from_secs(12);

/// Run the orchestrator. Returns the process exit code (0 = all assertions pass).
pub fn run(cfg: &Config) -> std::process::ExitCode {
    println!("=== integration --orchestrate: spawning tx, rx1, rx2 ===");
    let exe = match std::env::current_exe() {
        Ok(e) => e,
        Err(e) => {
            eprintln!("cannot locate self exe: {e}");
            return std::process::ExitCode::from(2);
        }
    };

    let mut children = Vec::new();
    for role in [Role::Tx, Role::Rx1, Role::Rx2] {
        let mut cmd = Command::new(&exe);
        cmd.arg("--role")
            .arg(role.as_str())
            .arg("--duration")
            .arg(cfg.duration_secs.to_string())
            .arg("--rate")
            .arg(cfg.rate_hz.to_string())
            .arg("--seed")
            .arg(cfg.seed.to_string())
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit());
        match cmd.spawn() {
            Ok(mut child) => {
                let stdout = Arc::new(Mutex::new(Vec::new()));
                let lines = stdout.clone();
                let role_str = role.as_str().to_string();
                let stdout_pipe = child.stdout.take().expect("piped stdout");
                let reader = std::thread::spawn(move || {
                    let reader = BufReader::new(stdout_pipe);
                    for line in reader.lines() {
                        if let Ok(line) = line {
                            if let Ok(mut g) = lines.lock() {
                                g.push(line);
                            }
                        }
                    }
                });
                children.push(ChildHandle { role, child, stdout, reader: Some(reader) });
                println!("  spawned {} (pid spawned)", role_str);
            }
            Err(e) => {
                eprintln!("failed to spawn {}: {e}", role.as_str());
                return std::process::ExitCode::from(2);
            }
        }
    }

    // Wait for all three READY lines, then GO.
    if !wait_for_ready(&mut children) {
        eprintln!("orchestrate: one or more roles never reported READY");
        dump_and_kill(&mut children);
        return std::process::ExitCode::from(1);
    }
    println!("orchestrate: all roles READY, sending GO");
    for h in children.iter_mut() {
        if let Some(mut s) = h.child.stdin.take() {
            let _ = s.write_all(b"\n");
            let _ = s.flush();
            // `s` is dropped here: the stdin pipe closes, so the child's
            // read-to-EOF returns and it starts its run window.
        }
    }

    // Wait for all children to exit (with grace beyond the run window).
    let deadline = Instant::now() + Duration::from_secs(cfg.duration_secs) + EXIT_GRACE;
    loop {
        let mut all_done = true;
        for h in children.iter_mut() {
            match h.child.try_wait() {
                Ok(Some(_status)) => {}
                Ok(None) => all_done = false,
                Err(_) => {}
            }
        }
        if all_done {
            break;
        }
        if Instant::now() >= deadline {
            eprintln!("orchestrate: timed out waiting for children to exit");
            dump_and_kill(&mut children);
            return std::process::ExitCode::from(1);
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    // All children have exited, so their stdout pipes are closed and each
    // reader thread has drained EOF. Join them so the final SUMMARY line is
    // guaranteed to be in the buffer before we scan for it.
    for h in children.iter_mut() {
        if let Some(reader) = h.reader.take() {
            let _ = reader.join();
        }
    }

    // Collect each role's SUMMARY line.
    let mut summaries: HashMap<String, HashMap<String, String>> = HashMap::new();
    let mut missing: Vec<String> = Vec::new();
    for h in children.iter() {
        let mut found: Option<String> = None;
        {
            let lines = h.stdout.lock().unwrap();
            for line in lines.iter() {
                if line.starts_with("SUMMARY|") {
                    found = Some(line.clone());
                }
            }
        } // lock released
        match found {
            Some(line) => {
                let map = parse_summary(&line).unwrap_or_default();
                println!(
                    "  {} summary: {}",
                    h.role.as_str(),
                    line.trim_start_matches("SUMMARY|")
                );
                summaries.insert(h.role.as_str().to_string(), map);
            }
            None => missing.push(h.role.as_str().to_string()),
        }
    }
    if !missing.is_empty() {
        for role in &missing {
            eprintln!("orchestrate: no SUMMARY line from {role}");
            for h in children.iter() {
                if h.role.as_str() == role {
                    let lines = h.stdout.lock().unwrap();
                    for l in lines.iter() {
                        eprintln!("    | {l}");
                    }
                }
            }
        }
        dump_and_kill(&mut children);
        return std::process::ExitCode::from(1);
    }

    // Exit codes.
    let mut exit_ok = true;
    for h in children.iter_mut() {
        if let Some(st) = h.child.try_wait().ok().flatten() {
            if !st.success() {
                eprintln!("  {} exited with {}", h.role.as_str(), st);
                exit_ok = false;
            }
        }
    }

    // Build the assertion table.
    let mut checks: Vec<(String, bool, String)> = Vec::new();
    run_assertions(&summaries, cfg, &mut checks);

    // Print the table.
    println!();
    println!("=== assertion results ({} checks) ===", checks.len());
    let mut all_pass = true;
    for (name, pass, detail) in &checks {
        let tag = if *pass { "PASS" } else { "FAIL" };
        println!("[{tag}] {name}: {detail}");
        if !*pass {
            all_pass = false;
        }
    }

    if all_pass && exit_ok {
        println!();
        println!("RESULT: ALL PASS");
        std::process::ExitCode::SUCCESS
    } else {
        println!();
        println!("RESULT: FAIL (assertion failure or child exit error)");
        std::process::ExitCode::from(1)
    }
}

fn run_assertions(
    s: &HashMap<String, HashMap<String, String>>,
    cfg: &Config,
    out: &mut Vec<(String, bool, String)>,
) {
    let tx = s.get("tx").expect("tx summary");
    let rx1 = s.get("rx1").expect("rx1 summary");
    let rx2 = s.get("rx2").expect("rx2 summary");

    let frames_sent = get_u64(tx, "frames_sent");
    let rx1_rx = get_u64(rx1, "frames_rx");
    let rx2_rx = get_u64(rx2, "frames_rx");
    let rx1_crc = get_u64(rx1, "crc_fail");
    let rx2_crc = get_u64(rx2, "crc_fail");
    let rx1_lost = get_u64(rx1, "lost");
    let rx2_lost = get_u64(rx2, "lost");
    let rx1_total = get_u64(rx1, "total");
    let rx2_total = get_u64(rx2, "total");
    let rx1_resyncs = get_u64(rx1, "resyncs");
    let rx2_resyncs = get_u64(rx2, "resyncs");
    let rx1_gaps = get_u64(rx1, "gaps");
    let rx2_gaps = get_u64(rx2, "gaps");

    let reports_rx1 = get_u64(tx, "reports_rx1");
    let reports_rx2 = get_u64(tx, "reports_rx2");
    let pairs_total = get_u64(tx, "pairs_total");
    let snapshots_rx = get_u64(tx, "snapshots_rx");
    let fused_outputs = get_u64(tx, "fused_outputs");
    let final_state = get_str(tx, "final_state").to_string();

    let min_frames = 1400u64;
    out.push((
        "RATE-1: tx.frames_sent >= 1400".into(),
        frames_sent >= min_frames,
        format!("frames_sent={frames_sent} (required >= {min_frames})"),
    ));

    let rx1_pct = if frames_sent > 0 { rx1_rx as f64 / frames_sent as f64 } else { 0.0 };
    let rx2_pct = if frames_sent > 0 { rx2_rx as f64 / frames_sent as f64 } else { 0.0 };
    out.push((
        "RATE-1: rx1 received >= 99% of sent".into(),
        rx1_pct >= 0.99,
        format!("rx1={rx1_rx}/{frames_sent} = {:.2}%", rx1_pct * 100.0),
    ));
    out.push((
        "RATE-1: rx2 received >= 99% of sent".into(),
        rx2_pct >= 0.99,
        format!("rx2={rx2_rx}/{frames_sent} = {:.2}%", rx2_pct * 100.0),
    ));
    out.push((
        "RATE-1: zero CRC failures (rx1)".into(),
        rx1_crc == 0,
        format!("crc_fail={rx1_crc}"),
    ));
    out.push((
        "RATE-1: zero CRC failures (rx2)".into(),
        rx2_crc == 0,
        format!("crc_fail={rx2_crc}"),
    ));
    let rx1_loss = if rx1_total > 0 { rx1_lost as f64 / rx1_total as f64 } else { 0.0 };
    let rx2_loss = if rx2_total > 0 { rx2_lost as f64 / rx2_total as f64 } else { 0.0 };
    out.push((
        "RATE-1: seq loss within 1% floor (rx1)".into(),
        rx1_loss <= 0.01,
        format!("lost={rx1_lost}/total={rx1_total} = {:.2}%", rx1_loss * 100.0),
    ));
    out.push((
        "RATE-1: seq loss within 1% floor (rx2)".into(),
        rx2_loss <= 0.01,
        format!("lost={rx2_lost}/total={rx2_total} = {:.2}%", rx2_loss * 100.0),
    ));
    out.push((
        "RATE-1: no out-of-order / resync (rx1)".into(),
        rx1_resyncs == 0,
        format!("resyncs={rx1_resyncs}"),
    ));
    out.push((
        "RATE-1: no out-of-order / resync (rx2)".into(),
        rx2_resyncs == 0,
        format!("resyncs={rx2_resyncs}"),
    ));
    out.push((
        "RATE-1: zero seq gaps (rx1+rx2)".into(),
        rx1_gaps == 0 && rx2_gaps == 0,
        format!("gaps: rx1={rx1_gaps}, rx2={rx2_gaps}"),
    ));

    let expected_reports = 2u64 * (frames_sent / REPORT_EVERY as u64);
    let reports_total = reports_rx1 + reports_rx2;
    let report_pct = if expected_reports > 0 { reports_total as f64 / expected_reports as f64 } else { 0.0 };
    out.push((
        "RATE-2: reports received >= 90% of expected".into(),
        report_pct >= 0.90,
        format!(
            "reports_rx1+rx2={reports_total}/{expected_reports} = {:.1}%",
            report_pct * 100.0
        ),
    ));
    let smaller_link = reports_rx1.min(reports_rx2);
    let pair_pct = if smaller_link > 0 { pairs_total as f64 / smaller_link as f64 } else { 0.0 };
    out.push((
        "RATE-2: pairs >= 60% of smaller link's reports".into(),
        pair_pct >= 0.60,
        format!(
            "pairs_total={pairs_total} / min({reports_rx1},{reports_rx2})={smaller_link} = {:.1}%",
            pair_pct * 100.0
        ),
    ));

    out.push((
        "RATE-3: snapshots received >= 8".into(),
        snapshots_rx >= 8,
        format!("snapshots_rx={snapshots_rx}"),
    ));

    out.push((
        "FUSION: >= 1 fused occupancy output from a real pair".into(),
        fused_outputs >= 1,
        format!("fused_outputs={fused_outputs}, final_state={final_state}"),
    ));

    let _ = cfg;
}

/// Poll the children's stdout buffers until all three roles have printed READY.
fn wait_for_ready(children: &mut [ChildHandle]) -> bool {
    let deadline = Instant::now() + READY_TIMEOUT;
    loop {
        let ready: Vec<&str> = children
            .iter()
            .filter_map(|h| {
                let lines = h.stdout.lock().unwrap();
                let has_ready = lines.iter().any(|l| l.starts_with("READY:"));
                if has_ready { Some(h.role.as_str()) } else { None }
            })
            .collect();
        if ready.len() == 3 {
            return true;
        }
        // Any child exited before ready?
        let mut exited = false;
        for h in children.iter_mut() {
            if let Ok(Some(_)) = h.child.try_wait() {
                exited = true;
            }
        }
        if exited {
            return false;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Best-effort kill of remaining children (used on failure paths).
fn dump_and_kill(children: &mut [ChildHandle]) {
    for h in children.iter_mut() {
        let _ = h.child.kill();
        let _ = h.child.wait();
    }
}
