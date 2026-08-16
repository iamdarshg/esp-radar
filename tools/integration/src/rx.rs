//! The RX1/RX2 role: binds a measurement socket on its own loopback address,
//! receives DataFrames, validates CRC/seq, runs the DSP pipeline, and sends
//! FeatureReports (every `REPORT_EVERY` frames) + CsiSnapshots (~2 Hz) to TX.

use std::io::{Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use radar_protocol::{frame_type, DataPayload};
use radar_transport::{
    build_csi_snapshot, build_feature_report, parse_frame, SequenceTracker,
};

use crate::common::{
    Config, MEASURE_PORT, REPORT_EVERY, REPORT_PORT, Role, SNAPSHOT_EVERY, TX_REPORT_ADDR,
};
use crate::synth::RxDsp;

/// Run one RX role. Blocks for `cfg.duration_secs`, then prints a SUMMARY line.
pub fn run(cfg: &Config, role: Role) -> std::process::ExitCode {
    let bind_addr = match role {
        Role::Rx1 => "127.0.0.2",
        Role::Rx2 => "127.0.0.3",
        _ => unreachable!("rx::run called for a non-RX role"),
    };

    // Measurement socket: this RX's own loopback address (TX unicasts a copy
    // of every DataFrame here).
    let meas_sock = match UdpSocket::bind(format!("{bind_addr}:{MEASURE_PORT}")) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}: cannot bind {bind_addr}:{MEASURE_PORT}: {e}", role.as_str());
            return std::process::ExitCode::from(2);
        }
    };
    let _ = meas_sock.set_nonblocking(true);

    // Report socket: any local address; unicasts RATE-2/3 to TX.
    let report_sock = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}: cannot bind report sender: {e}", role.as_str());
            return std::process::ExitCode::from(2);
        }
    };
    let _ = report_sock.set_nonblocking(true);
    let tx_addr: SocketAddr = match format!("{TX_REPORT_ADDR}:{REPORT_PORT}").parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{}: bad TX report address: {e}", role.as_str());
            return std::process::ExitCode::from(2);
        }
    };

    println!("READY:{}", role.as_str());
    let _ = std::io::stdout().flush();

    // Wait for GO (see tx.rs) or start after a short delay standalone.
    if wait_for_go() {
        println!("{}: GO received", role.as_str());
    } else {
        println!("{}: no GO signal (standalone), starting after delay", role.as_str());
        std::thread::sleep(Duration::from_millis(300));
    }

    let start = Instant::now();
    let deadline = start + Duration::from_secs(cfg.duration_secs);

    let mut tracker = SequenceTracker::new();
    let mut dsp = RxDsp::new(cfg.rate_hz as f32, role.node_id(), cfg.seed);
    let link = role.node_id();

    let mut frames_rx: u64 = 0;
    let mut crc_fail: u64 = 0;
    let mut non_data: u64 = 0;
    let mut reports_sent: u64 = 0;
    let mut snapshots_sent: u64 = 0;
    let mut last_seq: u32 = 0;
    let mut tx_power: u8 = 20;

    let mut meas_buf = [0u8; 1024];
    let mut report_buf = [0u8; 1024];

    while Instant::now() < deadline {
        // Drain the measurement socket.
        loop {
            match meas_sock.recv_from(&mut meas_buf) {
                Ok((len, _from)) => {
                    let buf = &meas_buf[..len];
                    match parse_frame(buf) {
                        Some((kind, src, seq, payload)) => {
                            if kind != frame_type::DATA_FRAME {
                                non_data += 1;
                                continue;
                            }
                            let _ = src; // TX is the only sender on the measure link
                            let ev = tracker.observe(seq);
                            frames_rx += 1;
                            last_seq = seq;
                            // Use the real DataFrame payload (tx_power_db) to
                            // modulate the synthetic CSI amplitude.
                            if payload.len() >= core::mem::size_of::<DataPayload>() {
                                let dp: DataPayload = unsafe {
                                    (payload.as_ptr() as *const DataPayload).read_unaligned()
                                };
                                tx_power = dp.tx_power_db;
                            }
                            let ch = dsp.synth_channel(seq, tx_power);
                            dsp.process_frame(&ch);
                            let _ = ev;

                            // Emit RATE-2 / RATE-3 on their cadences.
                            if frames_rx % REPORT_EVERY as u64 == 0 {
                                let fr = dsp.finish_window(seq);
                                let n = build_feature_report(
                                    &mut report_buf,
                                    link,
                                    &fr,
                                    crate::common::t_us_since(start),
                                );
                                let _ = report_sock.send_to(&report_buf[..n], tx_addr);
                                reports_sent += 1;
                            }
                            if frames_rx % SNAPSHOT_EVERY as u64 == 0 {
                                let snap = dsp.synth_snapshot(seq);
                                let n = build_csi_snapshot(
                                    &mut report_buf,
                                    link,
                                    &snap,
                                    crate::common::t_us_since(start),
                                );
                                let _ = report_sock.send_to(&report_buf[..n], tx_addr);
                                snapshots_sent += 1;
                            }
                        }
                        None => {
                            crc_fail += 1;
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        // Tiny sleep so the role does not busy-spin when the link is idle.
        std::thread::sleep(Duration::from_micros(200));
    }

    // Tail drain: catch any frames TX buffered right at its own deadline so
    // the boundary frame(s) are not lost to the exit race. Drain until quiet
    // for ~20 ms or a 100 ms hard cap.
    let tail_deadline = Instant::now() + Duration::from_millis(100);
    let mut last_activity = Instant::now();
    loop {
        let mut got = false;
        loop {
            match meas_sock.recv_from(&mut meas_buf) {
                Ok((len, _from)) => {
                    got = true;
                    last_activity = Instant::now();
                    let buf = &meas_buf[..len];
                    if let Some((kind, _src, seq, payload)) = parse_frame(buf) {
                        if kind != frame_type::DATA_FRAME {
                            continue;
                        }
                        let _ = tracker.observe(seq);
                        frames_rx += 1;
                        last_seq = seq;
                        if payload.len() >= core::mem::size_of::<DataPayload>() {
                            let dp: DataPayload =
                                unsafe { (payload.as_ptr() as *const DataPayload).read_unaligned() };
                            tx_power = dp.tx_power_db;
                        }
                        let ch = dsp.synth_channel(seq, tx_power);
                        dsp.process_frame(&ch);
                        if frames_rx % REPORT_EVERY as u64 == 0 {
                            let fr = dsp.finish_window(seq);
                            let n = build_feature_report(
                                &mut report_buf,
                                link,
                                &fr,
                                crate::common::t_us_since(start),
                            );
                            let _ = report_sock.send_to(&report_buf[..n], tx_addr);
                            reports_sent += 1;
                        }
                        if frames_rx % SNAPSHOT_EVERY as u64 == 0 {
                            let snap = dsp.synth_snapshot(seq);
                            let n = build_csi_snapshot(
                                &mut report_buf,
                                link,
                                &snap,
                                crate::common::t_us_since(start),
                            );
                            let _ = report_sock.send_to(&report_buf[..n], tx_addr);
                            snapshots_sent += 1;
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

    let lost = tracker.lost();
    let total = tracker.total();
    let gaps = tracker.gaps();
    let resyncs = tracker.resyncs();
    println!(
        "SUMMARY|role={}|frames_rx={frames_rx}|crc_fail={crc_fail}|non_data={non_data}|lost={lost}|total={total}|gaps={gaps}|resyncs={resyncs}|reports_sent={reports_sent}|snapshots_sent={snapshots_sent}|last_seq={last_seq}",
        role.as_str()
    );
    let _ = std::io::stdout().flush();
    std::process::ExitCode::SUCCESS
}

/// See `tx::wait_for_go`.
fn wait_for_go() -> bool {
    use std::io::IsTerminal;
    if std::io::stdin().is_terminal() {
        return false;
    }
    let mut line = String::new();
    match std::io::stdin().read_to_string(&mut line) {
        Ok(_) => true,
        Err(_) => false,
    }
}
