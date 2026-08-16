//! Integrated multi-node inter-chip comms test (Task 6).
//!
//! Runs the three radar roles — TX (measurement broadcast + fusion), RX1 and
//! RX2 (DSP + report) — as **separate OS processes** communicating over real
//! UDP on the host loopback, and asserts the RATE-1/2/3 flows end-to-end.
//!
//! This is the substitute for QEMU's missing WiFi PHY: it does not touch the
//! firmware at all. It drives the same `radar_transport` serializers/parsers
//! and the same `radar_dsp` / `radar_features` / `radar_calibration` fusion
//! logic over real sockets.
//!
//! ```
//!   integration --role tx                # run the TX role
//!   integration --role rx1               # run the RX1 role
//!   integration --role rx2               # run the RX2 role
//!   integration --orchestrate            # spawn all three, assert, exit 0/1
//! ```
//!
//! Build with the default (debug) profile: `cargo build` from this directory.
//! This tool lives OUTSIDE the repository workspace on purpose (see
//! `Cargo.toml`), so it is unaffected by the firmware's release profile.

mod common;
mod orchestrate;
mod rx;
mod synth;
mod tx;

use std::process::ExitCode;

use common::{Config, Role};

fn usage() -> ExitCode {
    eprintln!(
        "usage: integration --role <tx|rx1|rx2> [--duration <secs>] [--rate <hz>] [--seed <u64>]"
    );
    eprintln!("       integration --orchestrate [--duration <secs>] [--rate <hz>] [--seed <u64>]");
    ExitCode::from(2)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();

    let mut role: Option<Role> = None;
    let mut orchestrate = false;
    let mut cfg = Config::default();

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--role" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("--role requires a value (tx|rx1|rx2)");
                    return usage();
                }
                match Role::parse(&args[i]) {
                    Some(r) => role = Some(r),
                    None => {
                        eprintln!("unknown --role: {} (expected tx|rx1|rx2)", args[i]);
                        return usage();
                    }
                }
            }
            "--orchestrate" => orchestrate = true,
            "--duration" => {
                i += 1;
                match args.get(i).and_then(|s| s.parse::<u64>().ok()) {
                    Some(v) => cfg.duration_secs = v.max(1),
                    None => {
                        eprintln!("--duration requires a number of seconds");
                        return usage();
                    }
                }
            }
            "--rate" => {
                i += 1;
                match args.get(i).and_then(|s| s.parse::<u64>().ok()) {
                    Some(v) => cfg.rate_hz = v.clamp(10, 2000),
                    None => {
                        eprintln!("--rate requires a number in Hz");
                        return usage();
                    }
                }
            }
            "--seed" => {
                i += 1;
                match args.get(i).and_then(|s| s.parse::<u64>().ok()) {
                    Some(v) => cfg.seed = v,
                    None => {
                        eprintln!("--seed requires a u64");
                        return usage();
                    }
                }
            }
            s if s.starts_with('-') => {
                eprintln!("unknown option: {s}");
                return usage();
            }
            _ => {
                eprintln!("unexpected argument: {}", args[i]);
                return usage();
            }
        }
        i += 1;
    }

    if orchestrate && role.is_some() {
        eprintln!("--orchestrate and --role are mutually exclusive");
        return usage();
    }
    if orchestrate {
        return orchestrate::run(&cfg);
    }
    match role {
        Some(Role::Tx) => tx::run(&cfg),
        Some(Role::Rx1) | Some(Role::Rx2) => rx::run(&cfg, role.unwrap()),
        None => {
            eprintln!("no mode selected: use --role <tx|rx1|rx2> or --orchestrate");
            usage()
        }
    }
}
