//! Byte-stream frame extractor for the wired inter-board UART links.
//!
//! [`radar_protocol::parse`] consumes *complete* frames; a UART gives us a
//! byte stream with no guarantees about packet boundaries. [`RadarFrameDecoder`]
//! buffers bytes, hunts for the "RDR1" magic, validates the CRC, and yields
//! complete frames — resyncing by dropping one byte at a time whenever a bad
//! magic or a CRC failure means the stream is out of alignment. It never blocks
//! and never allocates more than a bounded buffer.
//!
//! This is the wired sibling of the UDP path in `udp.rs`: both share the same
//! `radar_protocol` frame builders/parsers, so a frame written over the wire by
//! `build_feature_report` is decoded here byte-for-byte. The two halves of the
//! head use it on the device side only (gated in the firmware, not here — this
//! module is pure logic and host-tested).

use radar_protocol::{Header, MAX_PAYLOAD};

/// Largest frame the link will ever carry (header + max payload).
pub const MAX_FRAME: usize = radar_protocol::HEADER_SIZE + MAX_PAYLOAD;
/// Internal buffer cap: older bytes are dropped if a garbage burst exceeds
/// this, so memory stays bounded even on a fully noisy line.
const MAX_BUFFER: usize = MAX_FRAME * 2;

/// A complete, validated radar frame (payload copied out of the buffer).
#[derive(Clone, Debug, PartialEq)]
pub struct RadarFrame {
    pub header: Header,
    pub payload: Vec<u8>,
}

impl RadarFrame {
    /// Frame type (a `radar_protocol::frame_type::*` constant).
    pub fn kind(&self) -> u8 {
        self.header.kind
    }
    /// Source node (a `radar_protocol::node::*` constant).
    pub fn src(&self) -> u8 {
        self.header.src_node
    }
    /// TX measurement sequence this frame relates to.
    pub fn seq(&self) -> u32 {
        self.header.seq
    }
}

/// Consumes a byte stream and yields aligned [`RadarFrame`]s.
#[derive(Debug, Default)]
pub struct RadarFrameDecoder {
    buf: Vec<u8>,
}

impl RadarFrameDecoder {
    pub fn new() -> Self {
        Self {
            buf: Vec::with_capacity(256),
        }
    }

    /// Append freshly-received bytes. No framing work happens here; call
    /// [`next`](Self::next) after feeding to collect any complete frames.
    pub fn feed(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
        if self.buf.len() > MAX_BUFFER {
            let overflow = self.buf.len() - MAX_BUFFER;
            self.buf.drain(..overflow);
        }
    }

    /// Pull the next complete, CRC-valid frame out of the buffer, if any.
    /// Scans forward past garbage and never blocks. Named `next` for the
    /// frame-stream caller, not `Iterator::next`.
    #[allow(clippy::should_implement_trait)]
    pub fn next(&mut self) -> Option<RadarFrame> {
        loop {
            if self.buf.len() < radar_protocol::HEADER_SIZE {
                return None;
            }
            // Peek the header without consuming. `Header` is `repr(C, packed)`
            // so it must be read unaligned and by value.
            let hdr: Header =
                unsafe { (self.buf.as_ptr() as *const Header).read_unaligned() };
            let magic = hdr.magic;
            let version = hdr.version;
            let payload_len = hdr.payload_len as usize;
            let total = radar_protocol::HEADER_SIZE + payload_len;

            if magic != radar_protocol::MAGIC
                || version != radar_protocol::VERSION
                || total > MAX_FRAME
            {
                // Not a valid header (garbage, or a stray byte between frames):
                // drop one byte and re-scan.
                self.buf.remove(0);
                continue;
            }
            if self.buf.len() < total {
                return None; // frame still arriving
            }
            match radar_protocol::parse(&self.buf[..total]) {
                Some((header, payload)) => {
                    // Copy the payload out so the borrow of `self.buf` ends
                    // before we mutate it below.
                    let payload_owned = payload.to_vec();
                    self.buf.drain(..total);
                    return Some(RadarFrame {
                        header,
                        payload: payload_owned,
                    });
                }
                None => {
                    // False magic match or corrupted frame: resync one byte.
                    self.buf.remove(0);
                }
            }
        }
    }

    /// Number of bytes currently buffered (diagnostics).
    pub fn buffered(&self) -> usize {
        self.buf.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{build_feature_report, FeatureReport};

    /// Encode a frame using the real `radar_protocol::build` encoder, so tests
    /// exercise the decoder against the exact wire format.
    fn frame_bytes(kind: u8, src: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; MAX_FRAME];
        let hdr = Header::new(kind, src, radar_protocol::node::TX, 7, 0, payload.len() as u16);
        let n = radar_protocol::build(&mut buf, &hdr, payload);
        buf[..n].to_vec()
    }

    #[test]
    fn single_frame() {
        let mut d = RadarFrameDecoder::new();
        d.feed(&frame_bytes(
            radar_protocol::frame_type::FEATURE_REPORT,
            radar_protocol::node::RX1,
            &[1, 2, 3, 4],
        ));
        let f = d.next().expect("one frame");
        assert_eq!(f.kind(), radar_protocol::frame_type::FEATURE_REPORT);
        assert_eq!(f.src(), radar_protocol::node::RX1);
        assert_eq!(f.seq(), 7);
        assert_eq!(f.payload, vec![1, 2, 3, 4]);
        assert!(d.next().is_none());
        assert_eq!(d.buffered(), 0);
    }

    #[test]
    fn split_across_feeds() {
        let bytes = frame_bytes(
            radar_protocol::frame_type::CAL_RESP,
            radar_protocol::node::RX2,
            &[9, 9, 9],
        );
        let mut d = RadarFrameDecoder::new();
        assert!(d.next().is_none());
        d.feed(&bytes[..5]);
        assert!(d.next().is_none(), "header incomplete");
        d.feed(&bytes[5..]);
        let f = d.next().expect("frame completes once last bytes arrive");
        assert_eq!(f.kind(), radar_protocol::frame_type::CAL_RESP);
        assert_eq!(f.src(), radar_protocol::node::RX2);
        assert_eq!(f.payload, vec![9, 9, 9]);
        assert!(d.next().is_none());
    }

    #[test]
    fn garbage_prefix_resyncs() {
        let bytes = frame_bytes(
            radar_protocol::frame_type::CSI_SNAPSHOT,
            radar_protocol::node::RX1,
            &[0xAA],
        );
        let mut noisy = vec![0u8; 3];
        noisy.extend_from_slice(&[0xFF, 0x00, 0x42]); // junk before the frame
        noisy.extend_from_slice(&bytes);
        let mut d = RadarFrameDecoder::new();
        d.feed(&noisy);
        let f = d.next().expect("rescans past garbage");
        assert_eq!(f.kind(), radar_protocol::frame_type::CSI_SNAPSHOT);
        assert_eq!(f.payload, vec![0xAA]);
    }

    #[test]
    fn concatenated_frames() {
        let a = frame_bytes(
            radar_protocol::frame_type::FEATURE_REPORT,
            radar_protocol::node::RX1,
            &[1, 1],
        );
        let b = frame_bytes(
            radar_protocol::frame_type::CAL_CMD,
            radar_protocol::node::TX,
            &[2, 2, 2],
        );
        let mut d = RadarFrameDecoder::new();
        d.feed(&a);
        d.feed(&b);
        assert_eq!(d.next().unwrap().kind(), radar_protocol::frame_type::FEATURE_REPORT);
        assert_eq!(d.next().unwrap().kind(), radar_protocol::frame_type::CAL_CMD);
        assert!(d.next().is_none());
    }

    #[test]
    fn corrupt_frame_dropped_and_next_survives() {
        let mut bad = frame_bytes(
            radar_protocol::frame_type::CSI_SNAPSHOT,
            radar_protocol::node::RX2,
            &[0x55],
        );
        let n = bad.len();
        bad[n - 1] ^= 0xFF; // flip the last payload byte → CRC fails
        let good = frame_bytes(
            radar_protocol::frame_type::FEATURE_REPORT,
            radar_protocol::node::RX1,
            &[0x42],
        );
        let mut d = RadarFrameDecoder::new();
        d.feed(&bad);
        d.feed(&good);
        let f = d.next().expect("resyncs past bad-CRC frame");
        assert_eq!(f.kind(), radar_protocol::frame_type::FEATURE_REPORT);
        assert_eq!(f.payload, vec![0x42]);
    }

    #[test]
    fn buffer_stays_bounded_under_garbage_burst() {
        let mut d = RadarFrameDecoder::new();
        for _ in 0..(MAX_BUFFER / 16 + 4) {
            d.feed(&[0x11; 16]);
        }
        assert!(d.buffered() <= MAX_BUFFER, "buffered = {}", d.buffered());
        assert!(d.next().is_none());
    }

    #[test]
    fn empty_payload_roundtrips() {
        let bytes = frame_bytes(
            radar_protocol::frame_type::STATUS,
            radar_protocol::node::TX,
            &[],
        );
        let mut d = RadarFrameDecoder::new();
        d.feed(&bytes);
        let f = d.next().expect("empty-payload frame");
        assert_eq!(f.kind(), radar_protocol::frame_type::STATUS);
        assert!(f.payload.is_empty());
    }

    #[test]
    fn real_feature_report_roundtrips() {
        // Decode a frame produced by the crate's own builder — the exact bytes
        // an RX node writes over the wire.
        let report = FeatureReport {
            seq: 12_345,
            n_frames: 20,
            n_missing: 1,
            rssi: -54,
            motion_energy: 0.37,
            ..Default::default()
        };
        let report_seq = report.seq; // packed struct — copy out before comparing
        let mut buf = [0u8; MAX_FRAME];
        let n = build_feature_report(
            &mut buf,
            radar_protocol::node::RX1,
            &report,
            1_000_000,
        );
        let mut d = RadarFrameDecoder::new();
        d.feed(&buf[..n]);
        let f = d.next().expect("real report frame");
        assert_eq!(f.kind(), radar_protocol::frame_type::FEATURE_REPORT);
        assert_eq!(f.src(), radar_protocol::node::RX1);
        assert_eq!(f.seq(), report_seq);
        let decoded = crate::parse_feature_report(&f.payload).expect("parse payload");
        // `FeatureReport` is repr(C, packed) — copy fields out by value before
        // comparing (E0793: field refs are unaligned).
        let decoded_seq = decoded.seq;
        let decoded_rssi = decoded.rssi;
        let decoded_energy = decoded.motion_energy;
        assert_eq!(decoded_seq, 12_345);
        assert_eq!(decoded_rssi, -54);
        assert!((decoded_energy - 0.37).abs() < 1e-6);
    }
}
