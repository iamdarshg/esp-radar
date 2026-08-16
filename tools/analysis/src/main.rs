//! Offline analysis of a captured radar stream (spec §13, tools).
//!
//! Reads a raw captured byte stream (a file, or stdin) and prints summary
//! statistics to stdout: total frames, frames/second, per-kind counts,
//! per-link (source node) counts, CRC failure count, RSSI over time
//! (min/max/mean), a packet-loss estimate from sequence-number gaps, and
//! motion_energy / dominant_freq summaries for feature reports.
//!
//! `--csv <path>` additionally writes a per-frame CSV
//! (`seq,t_us,kind,src,dst,rssi,snr,motion_energy,amp_mean,dominant_freq_hz`)
//! to the given path; fields that a frame kind does not carry are left empty.

use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::process::ExitCode;

use radar_protocol::frame_type;
use radar_protocol::{Header, MAX_PAYLOAD, HEADER_SIZE};
use radar_transport::SequenceTracker;

/// Magic as it appears on the wire (little-endian bytes of `0x5244_5231`).
const MAGIC_BYTES: [u8; 4] = radar_protocol::MAGIC.to_le_bytes();
/// Upper bound on a plausible payload length, used to reject false magic hits.
const MAX_SANE_PAYLOAD: usize = MAX_PAYLOAD * 8;

/// One item produced while scanning the captured stream.
enum Event<'a> {
    Frame { hdr: Header, payload: &'a [u8] },
    BadFrame,
    Truncated,
}

/// Running aggregate over the frames we have seen.
struct Agg {
    total: u64,
    crc_errors: u64,
    truncated: u64,
    kinds: HashMap<u8, u64>,
    per_src: HashMap<u8, u64>,
    first_t: Option<u64>,
    last_t: Option<u64>,
    rssi_count: u64,
    rssi_min: i32,
    rssi_max: i32,
    rssi_sum: i64,
    motion: Vec<(u64, f32)>,
    dominant: Vec<f32>,
    trackers: HashMap<u8, SequenceTracker>,
    csv: Option<BufWriter<File>>,
    csv_ok: bool,
}

impl Agg {
    fn new() -> Self {
        Self {
            total: 0,
            crc_errors: 0,
            truncated: 0,
            kinds: HashMap::new(),
            per_src: HashMap::new(),
            first_t: None,
            last_t: None,
            rssi_count: 0,
            rssi_min: 0,
            rssi_max: 0,
            rssi_sum: 0,
            motion: Vec::new(),
            dominant: Vec::new(),
            trackers: HashMap::new(),
            csv: None,
            csv_ok: true,
        }
    }

    fn feed(&mut self, hdr: &Header, payload: &[u8]) {
        self.total += 1;
        *self.kinds.entry(hdr.kind).or_insert(0) += 1;
        *self.per_src.entry(hdr.src_node).or_insert(0) += 1;

        self.first_t = Some(self.first_t.map_or(hdr.t_us, |f| f.min(hdr.t_us)));
        self.last_t = Some(self.last_t.map_or(hdr.t_us, |l| l.max(hdr.t_us)));

        self.trackers.entry(hdr.src_node).or_default().observe(hdr.seq);

        match hdr.kind {
            frame_type::FEATURE_REPORT => {
                if let Some(fr) = radar_transport::parse_feature_report(payload) {
                    self.add_rssi(fr.rssi as i32);
                    self.motion.push((hdr.t_us, fr.motion_energy));
                    self.dominant.push(fr.dominant_freq_hz);
                }
            }
            frame_type::CSI_SNAPSHOT => {
                if let Some(s) = radar_protocol::parse_csi_snapshot(payload) {
                    self.add_rssi(s.rssi as i32);
                }
            }
            _ => {}
        }

        if let Some(w) = self.csv.as_mut() {
            if writeln!(w, "{}", csv_row(hdr, payload)).is_err() {
                self.csv_ok = false;
            }
        }
    }

    fn add_rssi(&mut self, v: i32) {
        if self.rssi_count == 0 {
            self.rssi_min = v;
            self.rssi_max = v;
        } else {
            self.rssi_min = self.rssi_min.min(v);
            self.rssi_max = self.rssi_max.max(v);
        }
        self.rssi_sum += v as i64;
        self.rssi_count += 1;
    }
}

fn main() -> ExitCode {
    let mut csv_path: Option<String> = None;
    let mut file: Option<String> = None;
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--csv" => {
                i += 1;
                if i >= args.len() {
                    eprintln!("--csv requires a path");
                    return ExitCode::from(2);
                }
                csv_path = Some(args[i].clone());
            }
            s if s.starts_with('-') => {
                eprintln!("unknown option: {s}");
                eprintln!("usage: analysis [--csv <path>] [capture-file]");
                return ExitCode::from(2);
            }
            _ => {
                if file.is_some() {
                    eprintln!("unexpected extra argument: {}", args[i]);
                    return ExitCode::from(2);
                }
                file = Some(args[i].clone());
            }
        }
        i += 1;
    }

    let buf = match read_input(file.as_deref()) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("error reading input: {e}");
            return ExitCode::from(2);
        }
    };

    let mut agg = Agg::new();
    if let Some(p) = &csv_path {
        match File::create(p) {
            Ok(f) => {
                let mut w = BufWriter::new(f);
                let _ = writeln!(w, "seq,t_us,kind,src,dst,rssi,snr,motion_energy,amp_mean,dominant_freq_hz");
                agg.csv = Some(w);
            }
            Err(e) => {
                eprintln!("cannot open {p}: {e}");
                return ExitCode::from(2);
            }
        }
    }

    for ev in scan(&buf) {
        match ev {
            Event::Frame { hdr, payload } => agg.feed(&hdr, payload),
            Event::BadFrame => agg.crc_errors += 1,
            Event::Truncated => agg.truncated += 1,
        }
    }

    if let Some(w) = agg.csv.as_mut() {
        if w.flush().is_err() || !agg.csv_ok {
            eprintln!("error writing csv");
            return ExitCode::from(2);
        }
    }

    print_summary(&agg);
    ExitCode::SUCCESS
}

fn read_input(path: Option<&str>) -> io::Result<Vec<u8>> {
    match path {
        Some(p) => std::fs::read(p),
        None => {
            let mut buf = Vec::new();
            io::stdin().read_to_end(&mut buf)?;
            Ok(buf)
        }
    }
}

/// Scan `buf` for radar frames, tolerating gaps and recovering after corrupt
/// frames by skipping past their claimed length.
fn scan(buf: &[u8]) -> Vec<Event<'_>> {
    let mut events = Vec::new();
    let mut pos = 0usize;
    while pos + 4 <= buf.len() {
        let rel = buf[pos..].windows(4).position(|w| w == MAGIC_BYTES);
        let Some(off) = rel else { break };
        let start = pos + off;
        if buf.len() - start < HEADER_SIZE {
            events.push(Event::Truncated);
            break;
        }
        let hdr = decode_header(&buf[start..start + HEADER_SIZE]);
        let plen = hdr.payload_len as usize;
        if plen > MAX_SANE_PAYLOAD {
            pos = start + 1;
            continue;
        }
        let total = HEADER_SIZE + plen;
        if buf.len() - start < total {
            events.push(Event::Truncated);
            break;
        }
        match radar_protocol::parse(&buf[start..]) {
            Some((hdr, payload)) => {
                events.push(Event::Frame { hdr, payload });
                pos = start + total;
            }
            None => {
                events.push(Event::BadFrame);
                pos = start + total;
            }
        }
    }
    events
}

fn decode_header(b: &[u8]) -> Header {
    Header {
        magic: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
        version: b[4],
        kind: b[5],
        src_node: b[6],
        dst_node: b[7],
        seq: u32::from_le_bytes([b[8], b[9], b[10], b[11]]),
        t_us: u64::from_le_bytes([b[12], b[13], b[14], b[15], b[16], b[17], b[18], b[19]]),
        payload_len: u16::from_le_bytes([b[20], b[21]]),
        crc16: u16::from_le_bytes([b[22], b[23]]),
    }
}

fn kind_name(k: u8) -> &'static str {
    match k {
        frame_type::DATA_FRAME => "DATA",
        frame_type::FEATURE_REPORT => "FEATURE_REPORT",
        frame_type::CAL_CMD => "CAL_CMD",
        frame_type::CAL_RESP => "CAL_RESP",
        frame_type::STATUS => "STATUS",
        frame_type::CSI_SNAPSHOT => "CSI_SNAPSHOT",
        frame_type::CP_MESSAGE => "CP_MESSAGE",
        _ => "UNKNOWN",
    }
}

fn node_name(n: u8) -> &'static str {
    match n {
        radar_protocol::node::TX => "TX",
        radar_protocol::node::RX1 => "RX1",
        radar_protocol::node::RX2 => "RX2",
        radar_protocol::node::RP2350 => "RP2350",
        _ => "?",
    }
}

/// Build one CSV row for a frame. Missing fields stay empty.
fn csv_row(hdr: &Header, payload: &[u8]) -> String {
    // `Header` is `#[repr(C, packed)]`, so taking a reference to a field to
    // format it is E0793 (misaligned reference). Copy the scalars to locals
    // first (E0793 pattern).
    let seq = hdr.seq;
    let t_us = hdr.t_us;
    let kind = hdr.kind;
    let src = hdr.src_node;
    let dst = hdr.dst_node;
    let mut f: [String; 10] = std::array::from_fn(|_| String::new());
    f[0] = seq.to_string();
    f[1] = t_us.to_string();
    f[2] = kind.to_string();
    f[3] = src.to_string();
    f[4] = dst.to_string();
    match kind {
        frame_type::FEATURE_REPORT => {
            if let Some(fr) = radar_transport::parse_feature_report(payload) {
                let rssi = fr.rssi;
                let snr = fr.snr;
                let me = fr.motion_energy;
                let amp_mean = fr.amp_mean;
                let dom = fr.dominant_freq_hz;
                f[5] = rssi.to_string();
                f[6] = snr.to_string();
                f[7] = format!("{me}");
                f[8] = format!("{amp_mean}");
                f[9] = format!("{dom}");
            }
        }
        frame_type::CSI_SNAPSHOT => {
            if let Some(s) = radar_protocol::parse_csi_snapshot(payload) {
                let rssi = s.rssi;
                let snr = s.snr;
                f[5] = rssi.to_string();
                f[6] = snr.to_string();
                let amp = s.amp_norm;
                let n = s.n_sub as usize;
                if n > 0 {
                    let mean = amp[..n.min(amp.len())]
                        .iter()
                        .map(|&v| v as f64)
                        .sum::<f64>()
                        / n as f64;
                    f[8] = format!("{mean:.3}");
                }
            }
        }
        _ => {}
    }
    f.join(",")
}

fn print_summary(a: &Agg) {
    println!("analysis of radar capture");
    println!("total frames: {}", a.total);
    let span_s = match (a.first_t, a.last_t) {
        (Some(f), Some(l)) if l > f => (l - f) as f64 / 1_000_000.0,
        _ => 0.0,
    };
    println!("time span: {span_s:.3} s");
    if span_s > 0.0 {
        println!("frames/sec: {:.1}", a.total as f64 / span_s);
    } else {
        println!("frames/sec: N/A (single timestamp)");
    }
    println!("crc/version errors: {}", a.crc_errors);
    println!("truncated frames: {}", a.truncated);

    println!("per-kind counts:");
    let mut kinds: Vec<(&u8, &u64)> = a.kinds.iter().collect();
    kinds.sort_by_key(|(k, _)| **k);
    for (k, c) in kinds {
        println!("  {} (0x{k:02X}): {c}", kind_name(*k));
    }

    println!("per-source counts:");
    let mut srcs: Vec<(&u8, &u64)> = a.per_src.iter().collect();
    srcs.sort_by_key(|(s, _)| **s);
    for (s, c) in srcs {
        println!("  {} ({s}): {c}", node_name(*s));
    }

    if a.rssi_count > 0 {
        let mean = a.rssi_sum as f64 / a.rssi_count as f64;
        println!(
            "rssi (dBm): {} sample(s) min={} max={} mean={mean:.1}",
            a.rssi_count, a.rssi_min, a.rssi_max
        );
    } else {
        println!("rssi (dBm): no samples");
    }

    println!("packet loss estimate by seq gaps (per source):");
    let mut tr: Vec<(&u8, &SequenceTracker)> = a.trackers.iter().collect();
    tr.sort_by_key(|(s, _)| **s);
    for (s, t) in tr {
        println!(
            "  {} ({s}): lost={} total={} ratio={:.3} gaps={} resyncs={}",
            node_name(*s),
            t.lost(),
            t.total(),
            t.loss_ratio(),
            t.gaps(),
            t.resyncs()
        );
    }

    println!("motion_energy (feature reports): {} sample(s)", a.motion.len());
    if !a.motion.is_empty() {
        let (mn, mx, mean) = f32_stats(a.motion.iter().map(|&(_, v)| v));
        println!("  min={mn:.3} max={mx:.3} mean={mean:.3}");
        println!("  first 10 samples (t_us, value):");
        for (i, (t, v)) in a.motion.iter().take(10).enumerate() {
            println!("    [{i}] {t} {v:.3}");
        }
    }

    println!("dominant_freq_hz (feature reports): {} sample(s)", a.dominant.len());
    if !a.dominant.is_empty() {
        let (mn, mx, mean) = f32_stats(a.dominant.iter().copied());
        println!("  min={mn:.3} max={mx:.3} mean={mean:.3} last={:.3}", a.dominant[a.dominant.len() - 1]);
    }
}

fn f32_stats(iter: impl Iterator<Item = f32>) -> (f32, f32, f32) {
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    let mut sum = 0.0f64;
    let mut n = 0u64;
    for v in iter {
        min = min.min(v);
        max = max.max(v);
        sum += v as f64;
        n += 1;
    }
    if n == 0 {
        (0.0, 0.0, 0.0)
    } else {
        (min, max, (sum / n as f64) as f32)
    }
}
