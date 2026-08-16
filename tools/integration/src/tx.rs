//! The TX role: broadcasts DataFrames to both RX at `rate_hz`, receives
//! RATE-2/3 reports, pairs RX1/RX2 by seq, and fuses into an occupancy state.

use std::io::{Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use radar_features::{Fuser, LinkFeatures, OccupancyEstimator};
use radar_protocol::frame_type;
use radar_transport::{build_data_frame, parse_feature_report, parse_frame, Pairer};

use crate::common::{
    Config, MEASURE_PORT, PAIR_TOLERANCE, REPORT_PORT, RX1_MEASURE_ADDR, RX2_MEASURE_ADDR,
    TX_REPORT_ADDR,
};

/// Run the TX role. Blocks for `cfg.duration_secs`, then prints a SUMMARY line.
pub fn run(cfg: &Config) -> std::process::ExitCode {
    // Sender socket: any local address, used to unicast each DataFrame to both RX.
    let send_sock = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            eprintln!("tx: cannot bind sender socket: {e}");
            return std::process::ExitCode::from(2);
        }
    };
    let rx1_addr: SocketAddr = match format!("{RX1_MEASURE_ADDR}:{MEASURE_PORT}").parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("tx: bad RX1 address: {e}");
            return std::process::ExitCode::from(2);
        }
    };
    let rx2_addr: SocketAddr = match format!("{RX2_MEASURE_ADDR}:{MEASURE_PORT}").parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("tx: bad RX2 address: {e}");
            return std::process::ExitCode::from(2);
        }
    };
    let _ = TX_REPORT_ADDR; // documented; RX unicast to TX on the loopback wildcard

    // Report socket: TX listens here for RATE-2 (FeatureReport) + RATE-3
    // (CsiSnapshot) from both RX.
    let report_sock = match UdpSocket::bind(format!("0.0.0.0:{REPORT_PORT}")) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("tx: cannot bind report socket 0.0.0.0:{REPORT_PORT}: {e}");
            return std::process::ExitCode::from(2);
        }
    };
    let _ = report_sock.set_nonblocking(true);
    let _ = send_sock.set_nonblocking(true);

    println!("READY:tx");
    let _ = std::io::stdout().flush();

    // Wait for the orchestrator's GO (a newline on stdin, then EOF when it
    // drops the pipe). Run standalone (stdin is a terminal) -> start after a
    // short delay so receivers have time to bind.
    if wait_for_go() {
        println!("tx: GO received");
    } else {
        println!("tx: no GO signal (standalone), starting after delay");
        std::thread::sleep(Duration::from_millis(300));
    }

    let frame_interval = Duration::from_micros(1_000_000 / cfg.rate_hz.max(1));
    let start = Instant::now();
    let deadline = start + Duration::from_secs(cfg.duration_secs);

    // Fusion state (radar_features + radar_calibration).
    let params = radar_calibration::ClassThresholds::default().to_params();
    let mut fuser = Fuser::new(16);
    let mut estimator = OccupancyEstimator::new(params);
    let mut pairer = Pairer::new(PAIR_TOLERANCE);

    let mut seq: u32 = 0;
    let mut frames_sent: u64 = 0;
    let mut reports_rx1: u64 = 0;
    let mut reports_rx2: u64 = 0;
    let mut snapshots_rx: u64 = 0;
    let mut fused_outputs: u64 = 0;
    let mut final_state = String::from("UNKNOWN");
    let mut t_us: u64 = 1_000_000;
    let mut frame_buf = [0u8; 128];
    let mut recv_buf = [0u8; 1024];

    let mut next_send = start + frame_interval;
    while Instant::now() < deadline {
        let now = Instant::now();
        if next_send > now {
            std::thread::sleep(next_send - now);
        }
        next_send += frame_interval;

        // Send one DataFrame to each RX (same bytes, same seq).
        let tx_power = 20 + (seq % 15) as u8;
        let n = build_data_frame(&mut frame_buf, radar_protocol::node::TX, seq, t_us, tx_power, false);
        let frame = &frame_buf[..n];
        let _ = send_sock.send_to(frame, rx1_addr);
        let _ = send_sock.send_to(frame, rx2_addr);
        frames_sent += 1;
        seq = seq.wrapping_add(1);
        t_us = crate::common::t_us_since(start);

        // Drain the report socket.
        loop {
            match report_sock.recv_from(&mut recv_buf) {
                Ok((len, _from)) => {
                    let buf = &recv_buf[..len];
                    if let Some((kind, src, _seq, payload)) = parse_frame(buf) {
                        match kind {
                            frame_type::FEATURE_REPORT => {
                                if let Some(fr) = parse_feature_report(payload) {
                                    if src == radar_protocol::node::RX1 {
                                        reports_rx1 += 1;
                                    } else if src == radar_protocol::node::RX2 {
                                        reports_rx2 += 1;
                                    }
                                    pairer.push(src, fr, crate::common::t_us_since(start));
                                }
                            }
                            frame_type::CSI_SNAPSHOT => {
                                snapshots_rx += 1;
                            }
                            _ => {}
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        // Run pairing + fusion as long as pairs are available.
        loop {
            let now_us = crate::common::t_us_since(start);
            match pairer.next_pair(now_us) {
                Some(pair) => {
                    let l1 = link_features(&pair.rx1);
                    let l2 = link_features(&pair.rx2);
                    let metrics = fuser.push(pair.rx1.motion_energy, pair.rx2.motion_energy);
                    let est = estimator.update(&l1, &l2, &metrics);
                    fused_outputs += 1;
                    final_state = est.state.to_string();
                }
                None => break,
            }
        }
    }

    // Tail drain: catch the last RATE-2/3 reports the RX roles send as they
    // finish (RX runs its own 100 ms tail past its deadline). Drain until quiet
    // for ~20 ms or a 150 ms cap, then one last fusion pass.
    let tail_deadline = Instant::now() + Duration::from_millis(150);
    let mut last_activity = Instant::now();
    loop {
        let mut got = false;
        loop {
            match report_sock.recv_from(&mut recv_buf) {
                Ok((len, _)) => {
                    got = true;
                    last_activity = Instant::now();
                    if let Some((kind, src, _seq, payload)) = parse_frame(&recv_buf[..len]) {
                        match kind {
                            frame_type::FEATURE_REPORT => {
                                if let Some(fr) = parse_feature_report(payload) {
                                    if src == radar_protocol::node::RX1 {
                                        reports_rx1 += 1;
                                    } else {
                                        reports_rx2 += 1;
                                    }
                                    pairer.push(src, fr, crate::common::t_us_since(start));
                                }
                            }
                            frame_type::CSI_SNAPSHOT => snapshots_rx += 1,
                            _ => {}
                        }
                    }
                }
                Err(_) => break,
            }
        }
        if !got && last_activity.elapsed() >= Duration::from_millis(20) {
            break;
        }
        if Instant::now() >= tail_deadline {
            break;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    while let Some(pair) = pairer.next_pair(crate::common::t_us_since(start)) {
        let l1 = link_features(&pair.rx1);
        let l2 = link_features(&pair.rx2);
        let metrics = fuser.push(pair.rx1.motion_energy, pair.rx2.motion_energy);
        let est = estimator.update(&l1, &l2, &metrics);
        fused_outputs += 1;
        final_state = est.state.to_string();
    }

    // Emit the machine-readable summary (orchestrator parses this line).
    let pairs_total = pairer.pairs_total;
    println!(
        "SUMMARY|role=tx|frames_sent={frames_sent}|reports_rx1={reports_rx1}|reports_rx2={reports_rx2}|pairs_total={pairs_total}|snapshots_rx={snapshots_rx}|fused_outputs={fused_outputs}|final_state={final_state}"
    );
    let _ = std::io::stdout().flush();
    std::process::ExitCode::SUCCESS
}

/// Map a FeatureReport into `radar_features::LinkFeatures` for the estimator.
fn link_features(fr: &radar_protocol::FeatureReport) -> LinkFeatures {
    LinkFeatures {
        motion_energy: fr.motion_energy,
        baseline_dev: fr.baseline_dev,
        spectral_entropy: fr.spectral_entropy,
        dominant_freq_hz: fr.dominant_freq_hz,
        rssi: fr.rssi,
        sat_score: fr.sat_score,
        pca0: fr.pca_scores[0],
        pca1: fr.pca_scores[1],
        amp_std: fr.amp_std,
    }
}

/// Wait for the orchestrator's GO: a newline on stdin followed by EOF. When run
/// standalone (stdin is a terminal) there is no GO; returns false immediately.
fn wait_for_go() -> bool {
    use std::io::IsTerminal;
    if std::io::stdin().is_terminal() {
        return false;
    }
    let mut line = String::new();
    match std::io::stdin().read_to_string(&mut line) {
        Ok(_) => true, // GO line (or empty EOF) arrived
        Err(_) => false,
    }
}
