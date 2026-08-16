//! Synthetic radar test-stream generator (spec §13, tools).
//!
//! Generates radar frames — measurement ([`DataPayload`]) frames, RX
//! [`FeatureReport`]s and low-rate [`CsiSnapshot`]s — with correct CRCs,
//! monotonic sequence numbers and timestamps, and deterministic pseudorandom
//! CSI data (use `--seed` to reproduce a stream exactly). The stream is written
//! to stdout as raw frame bytes, or sent over UDP when `--host`/`--port` are
//! given (for exercising the live pipeline and dashboard without hardware).
//!
//! `--rate` sets the frame rate in Hz (used for timestamps and for UDP pacing);
//! `--frames` limits the number of frames generated.

use std::io::{self, Write};
use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::time::Duration;

use radar_protocol::frame_type;
use radar_protocol::node;
use radar_protocol::{CsiSnapshot, FeatureReport, Header, HEADER_SIZE, N_SPEC_BINS, N_SUBCARRIERS};

/// Which payload a frame carries.
#[derive(Clone, Copy, PartialEq, Eq)]
enum FrameKind {
    Data,
    Feature,
    Snapshot,
}

/// Generation mode: one payload kind, or an interleaved mixture.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Data,
    Feature,
    Snapshot,
    Mixed,
}

fn parse_mode(s: &str) -> Option<Mode> {
    match s {
        "data" => Some(Mode::Data),
        "feature" => Some(Mode::Feature),
        "snapshot" => Some(Mode::Snapshot),
        "mixed" => Some(Mode::Mixed),
        _ => None,
    }
}

/// Deterministic xorshift64 PRNG so streams are reproducible from a seed.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn next_u32(&mut self) -> u32 {
        self.next_u64() as u32
    }

    fn next_i16(&mut self, lo: i16, hi: i16) -> i16 {
        let range = (hi as i32 - lo as i32 + 1) as u32;
        lo + (self.next_u32() % range.max(1)) as i16
    }

    fn next_f32(&mut self, lo: f32, hi: f32) -> f32 {
        let u = self.next_u32() as f32 / u32::MAX as f32;
        lo + (hi - lo) * u
    }
}

/// Destination for the generated stream.
enum Output {
    Stdout(io::Stdout),
    Udp { sock: UdpSocket, addr: SocketAddr },
}

impl Output {
    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        match self {
            Output::Stdout(o) => {
                o.write_all(bytes)?;
                o.flush()
            }
            Output::Udp { sock, addr } => {
                // `addr` is `&mut SocketAddr` from the match on `&mut self`;
                // `send_to` wants a `ToSocketAddrs` value, so copy it out.
                sock.send_to(bytes, *addr).map(|_| ())?;
                Ok(())
            }
        }
    }
}

fn main() -> ExitCode {
    let mut mode_str = "mixed".to_string();
    let mut rate = 200u64;
    let mut frames = 1000u64;
    let mut seed = 1u64;
    let mut host: Option<String> = None;
    let mut port: Option<u16> = None;

    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--kind" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("--kind requires a value (data|feature|snapshot|mixed)");
                    return ExitCode::from(2);
                }
                mode_str = args[i].clone();
            }
            "--rate" => {
                i += 1;
                match args.get(i).and_then(|s| s.parse::<u64>().ok()) {
                    Some(r) => rate = r,
                    None => {
                        eprintln!("--rate requires a number in Hz");
                        return ExitCode::from(2);
                    }
                }
            }
            "--frames" => {
                i += 1;
                match args.get(i).and_then(|s| s.parse::<u64>().ok()) {
                    Some(n) => frames = n,
                    None => {
                        eprintln!("--frames requires a number");
                        return ExitCode::from(2);
                    }
                }
            }
            "--seed" => {
                i += 1;
                match args.get(i).and_then(|s| s.parse::<u64>().ok()) {
                    Some(s) => seed = s,
                    None => {
                        eprintln!("--seed requires a number");
                        return ExitCode::from(2);
                    }
                }
            }
            "--host" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("--host requires an address");
                    return ExitCode::from(2);
                }
                host = Some(args[i].clone());
            }
            "--port" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("--port requires a number");
                    return ExitCode::from(2);
                }
                match args[i].parse::<u16>() {
                    Ok(p) => port = Some(p),
                    Err(_) => {
                        eprintln!("invalid port: {}", args[i]);
                        return ExitCode::from(2);
                    }
                }
            }
            s if s.starts_with('-') => {
                eprintln!("unknown option: {s}");
                eprintln!(
                    "usage: test-generator [--kind data|feature|snapshot|mixed] [--rate <hz>] [--frames <n>] [--seed <u64>] [--host <ip>] [--port <u16>]"
                );
                return ExitCode::from(2);
            }
            _ => {
                eprintln!("unexpected argument: {}", args[i]);
                return ExitCode::from(2);
            }
        }
        i += 1;
    }

    let mode = match parse_mode(&mode_str) {
        Some(m) => m,
        None => {
            eprintln!("unknown --kind: {mode_str} (expected data|feature|snapshot|mixed)");
            return ExitCode::from(2);
        }
    };

    let rate = rate.max(1);
    let period_us = 1_000_000u64 / rate;
    let udp = host.is_some() || port.is_some();

    let mut out = if udp {
        let host = host.unwrap_or_else(|| radar_transport::Ipv4Addr::AP.to_string());
        let port = port.unwrap_or(radar_transport::REPORT_PORT);
        let addr: SocketAddr = match format!("{host}:{port}").parse() {
            Ok(a) => a,
            Err(e) => {
                eprintln!("cannot parse target {host}:{port}: {e}");
                return ExitCode::from(2);
            }
        };
        let sock = match UdpSocket::bind("0.0.0.0:0") {
            Ok(s) => s,
            Err(e) => {
                eprintln!("cannot bind UDP socket: {e}");
                return ExitCode::from(2);
            }
        };
        Output::Udp { sock, addr }
    } else {
        Output::Stdout(io::stdout())
    };

    let target = match &out {
        Output::Udp { addr, .. } => addr.to_string(),
        Output::Stdout(_) => "stdout".to_string(),
    };
    eprintln!(
        "generating {frames} frame(s) at {rate} Hz, mode={mode_str}, seed={seed}, rate_us={period_us}, target={target}"
    );

    let mut rng = Rng::new(seed);
    let mut tx_seq = 0u32;
    let mut feat_idx = 0u64;
    let mut snap_idx = 0u64;
    let t0 = 1_000_000u64; // start timestamps at t = 1 s so they stay positive

    for i in 0..frames {
        let t_us = t0 + i * period_us;
        let seq = tx_seq;
        tx_seq = tx_seq.wrapping_add(1);

        let frame = match pick(mode, i) {
            FrameKind::Data => {
                let tx_power = 20u8 + (i % 15) as u8;
                let flags = if i % 500 == 0 {
                    radar_protocol::data_flags::SYNC
                } else {
                    0
                };
                let payload = [tx_power, flags];
                let hdr = Header::new(frame_type::DATA_FRAME, node::TX, 0, seq, t_us, payload.len() as u16);
                build_frame(&hdr, &payload)
            }
            FrameKind::Feature => {
                let src = if feat_idx % 2 == 0 { node::RX1 } else { node::RX2 };
                feat_idx += 1;
                let fr = synthetic_feature(&mut rng, seq, i);
                let payload = feature_bytes(&fr);
                let hdr = Header::new(frame_type::FEATURE_REPORT, src, node::TX, seq, t_us, payload.len() as u16);
                build_frame(&hdr, &payload)
            }
            FrameKind::Snapshot => {
                let src = if snap_idx % 2 == 0 { node::RX1 } else { node::RX2 };
                snap_idx += 1;
                let snap = synthetic_snapshot(&mut rng, seq, i);
                let payload = snapshot_bytes(&snap);
                let hdr = Header::new(frame_type::CSI_SNAPSHOT, src, node::TX, seq, t_us, payload.len() as u16);
                build_frame(&hdr, &payload)
            }
        };

        if let Output::Udp { .. } = &out {
            if i > 0 && period_us > 0 {
                std::thread::sleep(Duration::from_micros(period_us));
            }
        }
        if let Err(e) = out.send(&frame) {
            eprintln!("error writing frame {i}: {e}");
            return ExitCode::from(1);
        }
    }

    ExitCode::SUCCESS
}

/// Choose the payload kind for frame `i` in the given mode.
fn pick(mode: Mode, i: u64) -> FrameKind {
    match mode {
        Mode::Data => FrameKind::Data,
        Mode::Feature => FrameKind::Feature,
        Mode::Snapshot => FrameKind::Snapshot,
        Mode::Mixed => {
            if i % 200 == 0 {
                FrameKind::Snapshot
            } else if i % 50 == 0 {
                FrameKind::Feature
            } else {
                FrameKind::Data
            }
        }
    }
}

/// Serialize `hdr` + `payload` into a complete frame with a correct CRC.
fn build_frame(hdr: &Header, payload: &[u8]) -> Vec<u8> {
    let mut buf = vec![0u8; HEADER_SIZE + payload.len()];
    let n = radar_protocol::build(&mut buf, hdr, payload);
    debug_assert_eq!(n, buf.len());
    buf
}

/// Safe little-endian byte writer for the packed payload structs.
struct W {
    buf: Vec<u8>,
}

impl W {
    fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    fn i8(&mut self, v: i8) {
        self.buf.push(v as u8);
    }
    fn i16(&mut self, v: i16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
}

/// Serialize a [`FeatureReport`] in its declared field order. `repr(C, packed)`
/// guarantees there is no padding, so this byte-exactly matches the struct
/// layout the receivers (and `radar_transport::parse_feature_report`) expect.
fn feature_bytes(fr: &FeatureReport) -> Vec<u8> {
    // Copy packed array fields out by value before iterating (E0793).
    let pca = fr.pca_scores;
    let mut w = W { buf: Vec::with_capacity(core::mem::size_of::<FeatureReport>()) };
    w.u32(fr.seq);
    w.u32(fr.n_frames);
    w.u32(fr.n_missing);
    w.i16(fr.rssi);
    w.i8(fr.snr);
    w.u8(fr.csi_quality);
    w.u8(fr.sat_score);
    w.u8(fr.dyn_range);
    w.u8(fr.flags);
    w.f32(fr.amp_mean);
    w.f32(fr.amp_std);
    w.f32(fr.motion_energy);
    w.f32(fr.spectral_entropy);
    w.f32(fr.dominant_freq_hz);
    w.f32(fr.phase_dispersion);
    w.f32(fr.baseline_dev);
    for &v in &pca {
        w.f32(v);
    }
    w.buf
}

/// Serialize a [`CsiSnapshot`] in its declared field order (15-byte fixed head,
/// then `iq`, `amp_norm`, `spec`).
fn snapshot_bytes(s: &CsiSnapshot) -> Vec<u8> {
    // Copy packed array fields out by value before iterating (E0793).
    let iq = s.iq;
    let amp = s.amp_norm;
    let spec = s.spec;
    let mut w = W { buf: Vec::with_capacity(core::mem::size_of::<CsiSnapshot>()) };
    w.u32(s.seq);
    w.i16(s.rssi);
    w.i8(s.snr);
    w.u8(s.csi_quality);
    w.f32(s.noise_floor);
    w.u8(s.flags);
    w.u8(s.n_sub);
    w.u8(s.reserved);
    for &v in &iq {
        w.i16(v);
    }
    for &v in &amp {
        w.u8(v);
    }
    for &v in &spec {
        w.u8(v);
    }
    w.buf
}

/// A plausible RX feature report. `activity` is a slow sine so the stream shows
/// a periodic "moving person" envelope that is easy to see in the dashboard.
fn synthetic_feature(rng: &mut Rng, seq: u32, i: u64) -> FeatureReport {
    let activity = 0.5 + 0.5 * (i as f32 * 0.05).sin(); // 0..=1
    let motion_energy = 0.1 + 2.5 * activity + rng.next_f32(0.0, 0.3);
    let dominant = (0.3 + 2.5 * activity + rng.next_f32(-0.2, 0.2)).max(0.0);
    let pca0 = motion_energy * 0.6 + rng.next_f32(0.0, 0.2);
    let pca1 = pca0 * 0.3 + rng.next_f32(0.0, 0.05);
    FeatureReport {
        seq,
        n_frames: 50,
        n_missing: rng.next_u32() % 3,
        rssi: rng.next_i16(-62, -52),
        snr: (20 + rng.next_u32() % 10) as i8,
        csi_quality: (70 + rng.next_u32() % 26) as u8,
        sat_score: (rng.next_u32() % 6) as u8,
        dyn_range: (60 + rng.next_u32() % 31) as u8,
        flags: if rng.next_u32() % 100 == 0 {
            radar_protocol::report_flags::OVERFLOW
        } else {
            0
        },
        amp_mean: 120.0 + activity * 80.0 + rng.next_f32(-10.0, 10.0),
        amp_std: 20.0 + rng.next_f32(0.0, 40.0),
        motion_energy,
        spectral_entropy: (0.2 + 0.5 * activity + rng.next_f32(-0.05, 0.05)).clamp(0.0, 1.0),
        dominant_freq_hz: dominant,
        phase_dispersion: 0.2 + rng.next_f32(0.0, 0.6),
        baseline_dev: 0.1 + rng.next_f32(0.0, 0.9),
        pca_scores: [pca0, pca1, pca1 * 0.4, 0.05, 0.02, 0.01, 0.0, 0.0],
    }
}

/// A plausible low-rate CSI snapshot: a sinusoidal amplitude envelope over
/// subcarriers with slow temporal drift, plus a motion-spectrum column that
/// has a moving Gaussian peak.
fn synthetic_snapshot(rng: &mut Rng, seq: u32, i: u64) -> CsiSnapshot {
    let mut iq = [0i16; N_SUBCARRIERS * 2];
    let mut amp = [0u8; N_SUBCARRIERS];
    let mut spec = [0u8; N_SPEC_BINS];

    let base_amp = 140.0 + 60.0 * (i as f32 * 0.03).sin();
    for k in 0..N_SUBCARRIERS {
        let a = base_amp + 50.0 * (k as f32 * 0.3).sin() + rng.next_f32(-15.0, 15.0);
        let phase = k as f32 * 0.7 + i as f32 * 0.02 + rng.next_f32(-0.3, 0.3);
        iq[2 * k] = (a * phase.cos()) as i16;
        iq[2 * k + 1] = (a * phase.sin()) as i16;
        let v = (128.0 + 60.0 * (k as f32 * 0.4 + i as f32 * 0.05).sin() + rng.next_f32(-20.0, 20.0))
            .clamp(0.0, 255.0);
        amp[k] = v as u8;
    }

    let dom_bin = (16.0 + 12.0 * (i as f32 * 0.03).sin()).clamp(1.0, 62.0);
    for b in 0..N_SPEC_BINS {
        let d = b as f32 - dom_bin;
        let g = (d * d / 6.0).exp();
        let v = 20.0 + 200.0 * g + rng.next_f32(0.0, 12.0);
        spec[b] = v.clamp(0.0, 255.0) as u8;
    }

    CsiSnapshot {
        seq,
        rssi: rng.next_i16(-62, -52),
        snr: (22 + rng.next_u32() % 8) as i8,
        csi_quality: (75 + rng.next_u32() % 21) as u8,
        noise_floor: -97.5,
        flags: 0,
        n_sub: N_SUBCARRIERS as u8,
        reserved: 0,
        iq,
        amp_norm: amp,
        spec,
    }
}
