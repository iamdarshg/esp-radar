//! Offline radar frame decoder (spec §13, tools).
//!
//! Reads a raw captured byte stream — a file given on the command line, or
//! stdin when no path is provided — and prints each [`radar_protocol`] frame as
//! human-readable text: decoded header fields, CRC validity, and a field dump
//! for the known payload kinds ([`DataPayload`], [`FeatureReport`],
//! [`CsiSnapshot`], calibration and status payloads).
//!
//! The captured stream may hold frames back-to-back, with arbitrary gaps
//! between them, or spanning many frames. The decoder scans for the frame magic
//! and resyncs after a corrupt frame. `--json` emits one JSON object per frame
//! on stdout instead of text (diagnostics still go to stderr). The process
//! exits nonzero if any CRC/version error or unknown frame kind was seen,
//! unless `--tolerate` is given.

use std::io::{self, Read};
use std::process::ExitCode;

use radar_protocol::frame_type;
use radar_protocol::{DataPayload, Header, MAX_PAYLOAD, HEADER_SIZE};

/// Copy of the header's scalar fields, extracted by value so the packed
/// `Header` is never borrowed (E0793).
#[derive(Clone, Copy)]
struct FlatHdr {
    magic: u32,
    version: u8,
    kind: u8,
    src_node: u8,
    dst_node: u8,
    seq: u32,
    t_us: u64,
    payload_len: u16,
    crc16: u16,
}

impl From<&Header> for FlatHdr {
    fn from(h: &Header) -> Self {
        // Reading a packed field by value is fine; only *borrowing* one is not.
        Self {
            magic: h.magic,
            version: h.version,
            kind: h.kind,
            src_node: h.src_node,
            dst_node: h.dst_node,
            seq: h.seq,
            t_us: h.t_us,
            payload_len: h.payload_len,
            crc16: h.crc16,
        }
    }
}

/// Magic as it appears on the wire (little-endian bytes of `0x5244_5231`).
const MAGIC_BYTES: [u8; 4] = radar_protocol::MAGIC.to_le_bytes();
/// Upper bound on a plausible payload length. Used to reject false magic hits
/// inside another frame's payload before we trust `payload_len` to resync.
const MAX_SANE_PAYLOAD: usize = MAX_PAYLOAD * 8;

/// One item produced while scanning the captured stream.
enum Event<'a> {
    /// A fully validated frame (magic, version and CRC all good).
    Frame {
        offset: usize,
        hdr: Header,
        payload: &'a [u8],
        #[allow(dead_code)]
        raw: &'a [u8],
    },
    /// Magic matched but the frame failed validation (version or CRC).
    BadFrame { offset: usize, hdr: Header, reason: &'static str },
    /// The stream ended in the middle of a frame header or body.
    Truncated { offset: usize },
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut json = false;
    let mut tolerate = false;
    let mut file: Option<String> = None;
    for a in &args {
        match a.as_str() {
            "--json" => json = true,
            "--tolerate" => tolerate = true,
            s if s.starts_with('-') => {
                eprintln!("unknown option: {s}");
                eprintln!("usage: decoder [--json] [--tolerate] [capture-file]");
                return ExitCode::from(2);
            }
            _ => {
                if file.is_some() {
                    eprintln!("unexpected extra argument: {a}");
                    return ExitCode::from(2);
                }
                file = Some(a.clone());
            }
        }
    }

    let buf = match read_input(file.as_deref()) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("error reading input: {e}");
            return ExitCode::from(2);
        }
    };

    let events = scan(&buf);
    let mut frame_idx = 0u64;
    let mut crc_errors = 0u64;
    let mut unknown_kinds = 0u64;

    for ev in &events {
        match ev {
            Event::Truncated { offset } => {
                if json {
                    println!("{}", json_truncated(*offset));
                } else {
                    println!("TRUNCATED header at offset {offset}: stream ends mid-frame");
                }
            }
            Event::BadFrame { offset, hdr, reason } => {
                crc_errors += 1;
                if json {
                    println!("{}", json_bad(*offset, hdr, reason));
                } else {
                    let h = FlatHdr::from(hdr);
                    println!(
                        "BAD FRAME at offset {offset}: kind={} src={} dst={} seq={} t_us={} claimed_len={} ({reason})",
                        kind_name(h.kind),
                        node_name(h.src_node),
                        node_name(h.dst_node),
                        h.seq,
                        h.t_us,
                        h.payload_len
                    );
                }
            }
            Event::Frame { offset, hdr, payload, .. } => {
                if !is_known_kind(hdr.kind) {
                    unknown_kinds += 1;
                }
                if json {
                    println!("{}", json_frame(frame_idx, *offset, hdr, payload));
                } else {
                    print_text_frame(frame_idx, *offset, hdr, payload);
                }
                frame_idx += 1;
            }
        }
    }

    eprintln!(
        "decoded {frame_idx} frame(s), {crc_errors} CRC/version error(s), {unknown_kinds} unknown kind(s), stream {} byte(s)",
        buf.len()
    );

    if !tolerate && (crc_errors > 0 || unknown_kinds > 0) {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}

/// Read the whole input: the given file, or stdin when `path` is `None`.
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
            events.push(Event::Truncated { offset: start });
            break;
        }
        let hdr = decode_header(&buf[start..start + HEADER_SIZE]);
        let plen = hdr.payload_len as usize;
        if plen > MAX_SANE_PAYLOAD {
            // A magic-looking byte sequence that does not sit at a plausible
            // frame boundary; rescan from the next byte rather than trusting
            // an absurd payload_len.
            pos = start + 1;
            continue;
        }
        let total = HEADER_SIZE + plen;
        if buf.len() - start < total {
            events.push(Event::Truncated { offset: start });
            break;
        }
        match radar_protocol::parse(&buf[start..]) {
            Some((hdr, payload)) => {
                events.push(Event::Frame {
                    offset: start,
                    hdr,
                    payload,
                    raw: &buf[start..start + total],
                });
                pos = start + total;
            }
            None => {
                let reason = if hdr.version != radar_protocol::VERSION {
                    "version mismatch"
                } else {
                    "crc failure"
                };
                events.push(Event::BadFrame { offset: start, hdr, reason });
                pos = start + total;
            }
        }
    }
    events
}

/// Decode a header from its little-endian wire bytes without forming a
/// reference into the packed struct (the protocol crate reads it internally
/// for the parse path; we need a second read for the corrupt-frame resync).
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

fn is_known_kind(k: u8) -> bool {
    matches!(
        k,
        frame_type::DATA_FRAME
            | frame_type::FEATURE_REPORT
            | frame_type::CAL_CMD
            | frame_type::CAL_RESP
            | frame_type::STATUS
            | frame_type::CSI_SNAPSHOT
            | frame_type::CP_MESSAGE
    )
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

// ---------------------------------------------------------------------------
// Human-readable output
// ---------------------------------------------------------------------------

fn print_text_frame(idx: u64, offset: usize, hdr: &Header, payload: &[u8]) {
    let h = FlatHdr::from(hdr);
    println!(
        "FRAME #{idx} offset={offset} magic=0x{:08X} version={} kind={} src={} dst={} seq={} t_us={} payload_len={} crc16=0x{:04X} CRC=OK",
        h.magic,
        h.version,
        kind_name(h.kind),
        node_name(h.src_node),
        node_name(h.dst_node),
        h.seq,
        h.t_us,
        h.payload_len,
        h.crc16
    );
    dump_payload(hdr.kind, payload, 2);
}

fn dump_payload(kind: u8, p: &[u8], indent: usize) {
    let pad = " ".repeat(indent);
    match kind {
        frame_type::DATA_FRAME => dump_data(p, &pad),
        frame_type::FEATURE_REPORT => dump_feature(p, &pad),
        frame_type::CSI_SNAPSHOT => dump_snapshot(p, &pad),
        frame_type::CAL_CMD => dump_cal_cmd(p, &pad),
        frame_type::CAL_RESP => dump_cal_resp(p, &pad),
        frame_type::STATUS => dump_status(p, &pad),
        _ => println!("{pad}payload: {} byte(s) hex={}", p.len(), hex_capped(p)),
    }
}

fn dump_data(p: &[u8], pad: &str) {
    if p.len() < core::mem::size_of::<DataPayload>() {
        println!("{pad}payload: short data payload ({} bytes)", p.len());
        return;
    }
    let flags = p[1];
    let mut tag = String::new();
    if flags & radar_protocol::data_flags::CAL != 0 {
        tag.push_str(" CAL");
    }
    if flags & radar_protocol::data_flags::SYNC != 0 {
        tag.push_str(" SYNC");
    }
    println!("{pad}payload: DataPayload tx_power_db={} flags=0x{flags:02X}{tag}", p[0]);
}

fn dump_feature(p: &[u8], pad: &str) {
    let Some(fr) = radar_transport::parse_feature_report(p) else {
        println!("{pad}payload: short feature report ({} bytes)", p.len());
        return;
    };
    // Copy every packed field out by value before formatting (E0793).
    let (seq, n_frames, n_missing, rssi, snr, csi_quality, sat_score, dyn_range, flags) =
        (fr.seq, fr.n_frames, fr.n_missing, fr.rssi, fr.snr, fr.csi_quality, fr.sat_score, fr.dyn_range, fr.flags);
    let (amp_mean, amp_std, motion_energy, spectral_entropy, dominant_freq_hz, phase_dispersion, baseline_dev) =
        (fr.amp_mean, fr.amp_std, fr.motion_energy, fr.spectral_entropy, fr.dominant_freq_hz, fr.phase_dispersion, fr.baseline_dev);
    let pca = fr.pca_scores;
    let mut overflow = String::new();
    if flags & radar_protocol::report_flags::OVERFLOW != 0 {
        overflow.push_str(" OVERFLOW");
    }
    println!(
        "{pad}payload: FeatureReport seq={seq} n_frames={n_frames} n_missing={n_missing} rssi={rssi} dBm snr={snr} dB csi_quality={csi_quality} sat_score={sat_score} dyn_range={dyn_range} flags=0x{flags:02X}{overflow}"
    );
    println!(
        "{pad}  amp_mean={amp_mean:.3} amp_std={amp_std:.3} motion_energy={motion_energy:.3} spectral_entropy={spectral_entropy:.3} dominant_freq_hz={dominant_freq_hz:.3} phase_dispersion={phase_dispersion:.3} baseline_dev={baseline_dev:.3}"
    );
    let scores: Vec<String> = pca.iter().map(|v| format!("{v:.3}")).collect();
    println!("{pad}  pca_scores=[{}]", scores.join(", "));
}

fn dump_snapshot(p: &[u8], pad: &str) {
    let Some(s) = radar_protocol::parse_csi_snapshot(p) else {
        println!("{pad}payload: short csi snapshot ({} bytes)", p.len());
        return;
    };
    let (seq, rssi, snr, csi_quality, noise_floor, flags, n_sub) =
        (s.seq, s.rssi, s.snr, s.csi_quality, s.noise_floor, s.flags, s.n_sub);
    let iq = s.iq;
    let amp = s.amp_norm;
    let spec = s.spec;
    let n = n_sub as usize;
    let amp_mean = mean_u8(&amp[..n.min(amp.len())]);
    let iq_mean = mean_abs_i16(&iq[..(n * 2).min(iq.len())]);
    let spec_mean = mean_u8(&spec);
    println!(
        "{pad}payload: CsiSnapshot seq={seq} rssi={rssi} dBm snr={snr} dB csi_quality={csi_quality} noise_floor={noise_floor:.1} flags=0x{flags:02X} n_sub={n_sub}"
    );
    println!(
        "{pad}  amp_norm mean={amp_mean:.1} first8={:?}",
        &amp[..amp.len().min(8)]
    );
    println!(
        "{pad}  iq mean|v|={iq_mean:.1} first4pairs={:?}",
        &iq[..iq.len().min(8)]
    );
    println!("{pad}  spec mean={spec_mean:.1} first8={:?}", &spec[..spec.len().min(8)]);
}

fn dump_cal_cmd(p: &[u8], pad: &str) {
    if p.len() < 8 {
        println!("{pad}payload: short cal cmd ({} bytes)", p.len());
        return;
    }
    let collect_ms = u32::from_le_bytes([p[2], p[3], p[4], p[5]]);
    let tx_power_db = i16::from_le_bytes([p[6], p[7]]);
    println!(
        "{pad}payload: CalCmd stage={} action={} collect_ms={} tx_power_db={}",
        p[0], p[1], collect_ms, tx_power_db
    );
}

fn dump_cal_resp(p: &[u8], pad: &str) {
    if p.len() < 17 {
        println!("{pad}payload: short cal resp ({} bytes)", p.len());
        return;
    }
    let rssi = i16::from_le_bytes([p[2], p[3]]);
    let noise_floor = f32::from_le_bytes([p[9], p[10], p[11], p[12]]);
    let n_samples = u32::from_le_bytes([p[13], p[14], p[15], p[16]]);
    println!(
        "{pad}payload: CalResp stage={} result={} rssi={} dBm snr={} dB csi_quality={} sat_score={} dyn_range={} noise_floor={:.1} n_samples={}",
        p[0], p[1], rssi, p[4] as i8, p[5], p[6], p[7], noise_floor, n_samples
    );
}

fn dump_status(p: &[u8], pad: &str) {
    if p.len() < 22 {
        println!("{pad}payload: short status ({} bytes)", p.len());
        return;
    }
    let uptime_s = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
    let tx_seq = u32::from_le_bytes([p[4], p[5], p[6], p[7]]);
    let rx_packets = u32::from_le_bytes([p[8], p[9], p[10], p[11]]);
    let rx_drops = u32::from_le_bytes([p[12], p[13], p[14], p[15]]);
    let heap_free = u16::from_le_bytes([p[16], p[17]]);
    println!(
        "{pad}payload: Status uptime_s={uptime_s} tx_seq={tx_seq} rx_packets={rx_packets} rx_drops={rx_drops} heap_free={heap_free} wifi_connected={} csi_enabled={} paired_frames_per_s={}",
        p[18], p[19], p[20]
    );
}

// ---------------------------------------------------------------------------
// JSON output
// ---------------------------------------------------------------------------

fn json_truncated(offset: usize) -> String {
    format!("{{\"truncated\":true,\"offset\":{offset}}}")
}

fn json_bad(offset: usize, hdr: &Header, reason: &str) -> String {
    let h = FlatHdr::from(hdr);
    format!(
        "{{\"bad\":true,\"offset\":{offset},\"kind\":{},\"kind_name\":\"{}\",\"src\":{},\"src_name\":\"{}\",\"dst\":{},\"seq\":{},\"t_us\":{},\"claimed_payload_len\":{},\"crc16\":{},\"reason\":\"{reason}\"}}",
        h.kind,
        kind_name(h.kind),
        h.src_node,
        node_name(h.src_node),
        h.dst_node,
        h.seq,
        h.t_us,
        h.payload_len,
        h.crc16
    )
}

fn json_frame(idx: u64, offset: usize, hdr: &Header, payload: &[u8]) -> String {
    let h = FlatHdr::from(hdr);
    format!(
        "{{\"frame\":{idx},\"offset\":{offset},\"magic\":\"0x{:08X}\",\"version\":{},\"kind\":{},\"kind_name\":\"{}\",\"src\":{},\"src_name\":\"{}\",\"dst\":{},\"dst_name\":\"{}\",\"seq\":{},\"t_us\":{},\"payload_len\":{},\"crc16\":{},\"crc_ok\":true,\"payload\":{}}}",
        h.magic,
        h.version,
        h.kind,
        kind_name(h.kind),
        h.src_node,
        node_name(h.src_node),
        h.dst_node,
        node_name(h.dst_node),
        h.seq,
        h.t_us,
        h.payload_len,
        h.crc16,
        json_payload(h.kind, payload)
    )
}

fn json_payload(kind: u8, p: &[u8]) -> String {
    match kind {
        frame_type::DATA_FRAME => {
            if p.len() < 2 {
                return format!("{{\"short\":{}}}", p.len());
            }
            format!(
                "{{\"tx_power_db\":{},\"flags\":{},\"flags_cal\":{},\"flags_sync\":{}}}",
                p[0],
                p[1],
                (p[1] & radar_protocol::data_flags::CAL != 0) as u8,
                (p[1] & radar_protocol::data_flags::SYNC != 0) as u8
            )
        }
        frame_type::FEATURE_REPORT => {
            let Some(fr) = radar_transport::parse_feature_report(p) else {
                return format!("{{\"short\":{}}}", p.len());
            };
            let (seq, n_frames, n_missing, rssi, snr, csi_quality, sat_score, dyn_range, flags) =
                (fr.seq, fr.n_frames, fr.n_missing, fr.rssi, fr.snr, fr.csi_quality, fr.sat_score, fr.dyn_range, fr.flags);
            let (amp_mean, amp_std, motion_energy, spectral_entropy, dominant_freq_hz, phase_dispersion, baseline_dev) =
                (fr.amp_mean, fr.amp_std, fr.motion_energy, fr.spectral_entropy, fr.dominant_freq_hz, fr.phase_dispersion, fr.baseline_dev);
            let pca = fr.pca_scores;
            format!(
                "{{\"seq\":{seq},\"n_frames\":{n_frames},\"n_missing\":{n_missing},\"rssi\":{rssi},\"snr\":{snr},\"csi_quality\":{csi_quality},\"sat_score\":{sat_score},\"dyn_range\":{dyn_range},\"flags\":{flags},\"amp_mean\":{amp_mean},\"amp_std\":{amp_std},\"motion_energy\":{motion_energy},\"spectral_entropy\":{spectral_entropy},\"dominant_freq_hz\":{dominant_freq_hz},\"phase_dispersion\":{phase_dispersion},\"baseline_dev\":{baseline_dev},\"pca_scores\":{}}}",
                json_f32_array(&pca)
            )
        }
        frame_type::CSI_SNAPSHOT => {
            let Some(s) = radar_protocol::parse_csi_snapshot(p) else {
                return format!("{{\"short\":{}}}", p.len());
            };
            let (seq, rssi, snr, csi_quality, noise_floor, flags, n_sub) =
                (s.seq, s.rssi, s.snr, s.csi_quality, s.noise_floor, s.flags, s.n_sub);
            let iq = s.iq;
            let amp = s.amp_norm;
            let spec = s.spec;
            let n = n_sub as usize;
            let amp_mean = mean_u8(&amp[..n.min(amp.len())]);
            let spec_mean = mean_u8(&spec);
            format!(
                "{{\"seq\":{seq},\"rssi\":{rssi},\"snr\":{snr},\"csi_quality\":{csi_quality},\"noise_floor\":{noise_floor},\"flags\":{flags},\"n_sub\":{n_sub},\"amp_mean\":{amp_mean:.1},\"spec_mean\":{spec_mean:.1},\"amp_first\":{},\"spec_first\":{},\"iq_first\":{}}}",
                json_u8_array(&amp[..amp.len().min(8)]),
                json_u8_array(&spec[..spec.len().min(8)]),
                json_i16_array(&iq[..iq.len().min(8)])
            )
        }
        frame_type::CAL_CMD => {
            if p.len() < 8 {
                return format!("{{\"short\":{}}}", p.len());
            }
            let collect_ms = u32::from_le_bytes([p[2], p[3], p[4], p[5]]);
            let tx_power_db = i16::from_le_bytes([p[6], p[7]]);
            format!(
                "{{\"stage\":{},\"action\":{},\"collect_ms\":{collect_ms},\"tx_power_db\":{tx_power_db}}}",
                p[0], p[1]
            )
        }
        frame_type::CAL_RESP => {
            if p.len() < 17 {
                return format!("{{\"short\":{}}}", p.len());
            }
            let rssi = i16::from_le_bytes([p[2], p[3]]);
            let noise_floor = f32::from_le_bytes([p[9], p[10], p[11], p[12]]);
            let n_samples = u32::from_le_bytes([p[13], p[14], p[15], p[16]]);
            format!(
                "{{\"stage\":{},\"result\":{},\"rssi\":{rssi},\"snr\":{},\"csi_quality\":{},\"sat_score\":{},\"dyn_range\":{},\"noise_floor\":{noise_floor},\"n_samples\":{n_samples}}}",
                p[0], p[1], p[4] as i8, p[5], p[6], p[7]
            )
        }
        frame_type::STATUS => {
            if p.len() < 22 {
                return format!("{{\"short\":{}}}", p.len());
            }
            let uptime_s = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
            let tx_seq = u32::from_le_bytes([p[4], p[5], p[6], p[7]]);
            let rx_packets = u32::from_le_bytes([p[8], p[9], p[10], p[11]]);
            let rx_drops = u32::from_le_bytes([p[12], p[13], p[14], p[15]]);
            let heap_free = u16::from_le_bytes([p[16], p[17]]);
            format!(
                "{{\"uptime_s\":{uptime_s},\"tx_seq\":{tx_seq},\"rx_packets\":{rx_packets},\"rx_drops\":{rx_drops},\"heap_free\":{heap_free},\"wifi_connected\":{},\"csi_enabled\":{},\"paired_frames_per_s\":{}}}",
                p[18], p[19], p[20]
            )
        }
        _ => format!("{{\"hex\":\"{}\"}}", hex_capped(p)),
    }
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

fn mean_u8(vals: &[u8]) -> f64 {
    if vals.is_empty() {
        0.0
    } else {
        vals.iter().map(|&v| v as f64).sum::<f64>() / vals.len() as f64
    }
}

fn mean_abs_i16(vals: &[i16]) -> f64 {
    if vals.is_empty() {
        0.0
    } else {
        vals.iter().map(|&v| (v as i64).unsigned_abs() as f64).sum::<f64>() / vals.len() as f64
    }
}

fn json_u8_array(vals: &[u8]) -> String {
    let items: Vec<String> = vals.iter().map(|v| v.to_string()).collect();
    format!("[{}]", items.join(","))
}

fn json_i16_array(vals: &[i16]) -> String {
    let items: Vec<String> = vals.iter().map(|v| v.to_string()).collect();
    format!("[{}]", items.join(","))
}

fn json_f32_array(vals: &[f32]) -> String {
    let items: Vec<String> = vals.iter().map(|v| format!("{v}")).collect();
    format!("[{}]", items.join(","))
}

/// Uppercase hex of `bytes`, capped at 128 bytes with an ellipsis marker.
fn hex_capped(bytes: &[u8]) -> String {
    const CAP: usize = 128;
    let n = bytes.len().min(CAP);
    let mut s = String::with_capacity(n * 2 + 3);
    for b in &bytes[..n] {
        s.push_str(&format!("{b:02X}"));
    }
    if bytes.len() > CAP {
        s.push_str("...");
    }
    s
}
