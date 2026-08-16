//! Replay a captured radar stream over UDP (spec §13, tools).
//!
//! Reads a raw captured byte stream (a file, or stdin), validates and extracts
//! every intact frame, and re-sends each frame's original bytes as a UDP
//! datagram to a target host:port. Inter-frame timing is preserved from the
//! frames' `t_us` header timestamps unless `--fast` is given (send as fast as
//! possible). `--loop` repeats the stream forever, keeping timing monotonic.
//!
//! The default target is RADAR-TX's fusion receive port (`REPORT_PORT`, where
//! TX listens for RX feature reports) at the RADAR-TX AP address; override with
//! `--host`/`--port`.

use std::io::{self, Read};
use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use radar_protocol::{Header, MAX_PAYLOAD, HEADER_SIZE};

/// Magic as it appears on the wire (little-endian bytes of `0x5244_5231`).
const MAGIC_BYTES: [u8; 4] = radar_protocol::MAGIC.to_le_bytes();
/// Upper bound on a plausible payload length, used to reject false magic hits.
const MAX_SANE_PAYLOAD: usize = MAX_PAYLOAD * 8;

fn main() -> ExitCode {
    let mut host: Option<String> = None;
    let mut port: Option<u16> = None;
    let mut fast = false;
    let mut looping = false;
    let mut file: Option<String> = None;

    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
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
            "--fast" => fast = true,
            "--loop" => looping = true,
            s if s.starts_with('-') => {
                eprintln!("unknown option: {s}");
                eprintln!(
                    "usage: replay [--host <ip>] [--port <u16>] [--fast] [--loop] [capture-file]"
                );
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

    // Extract only intact frames, keeping their original bytes for replay.
    let mut frames: Vec<(u64, Vec<u8>)> = Vec::new();
    let mut bad = 0u64;
    for ev in scan(&buf) {
        match ev {
            Event::Frame { t_us, raw } => frames.push((t_us, raw.to_vec())),
            Event::BadFrame | Event::Truncated => bad += 1,
        }
    }

    if frames.is_empty() {
        eprintln!("no intact radar frames found in input");
        return ExitCode::from(2);
    }
    eprintln!(
        "replaying {} frame(s) ({} corrupt/truncated skipped), {} byte(s) of input",
        frames.len(),
        bad,
        buf.len()
    );

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

    if let Err(e) = replay(&sock, addr, &frames, fast, looping) {
        eprintln!("replay error: {e}");
        return ExitCode::from(1);
    }
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

/// Send `frames` to `addr`, pacing on `t_us` unless `fast`. With `looping`,
/// restart from the top, accumulating a monotonic time offset per iteration so
/// sleeps never go backwards.
fn replay(
    sock: &UdpSocket,
    addr: SocketAddr,
    frames: &[(u64, Vec<u8>)],
    fast: bool,
    looping: bool,
) -> io::Result<()> {
    let first_t = frames[0].0;
    let span = frames.last().map(|(t, _)| t.saturating_sub(first_t)).unwrap_or(0);
    let start = Instant::now();
    let mut loop_offset: u64 = 0;

    loop {
        for (t_us, raw) in frames {
            if !fast {
                let rel = t_us.saturating_sub(first_t).saturating_add(loop_offset);
                let target = start + Duration::from_micros(rel);
                let now = Instant::now();
                if target > now {
                    std::thread::sleep(target - now);
                }
            }
            sock.send_to(raw, addr)?;
        }
        if !looping {
            break;
        }
        loop_offset = loop_offset.wrapping_add(span);
    }
    Ok(())
}

/// One item produced while scanning the captured stream.
enum Event<'a> {
    Frame { t_us: u64, raw: &'a [u8] },
    BadFrame,
    Truncated,
}

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
            Some((hdr, _payload)) => {
                events.push(Event::Frame { t_us: hdr.t_us, raw: &buf[start..start + total] });
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
