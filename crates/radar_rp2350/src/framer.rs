//! Byte-stream frame extractor for the coprocessor UART link.
//!
//! [`radar_protocol::cp::parse`] consumes *complete* frames; a UART gives us a
//! byte stream with no guarantees about packet boundaries. [`FrameDecoder`]
//! buffers bytes, hunts for the magic, validates the CRC, and yields complete
//! frames — resyncing by dropping one byte at a time whenever a bad magic or a
//! CRC failure means the stream is out of alignment. It never blocks and never
//! allocates more than a bounded buffer.

use radar_protocol::cp;

/// Largest frame the link will ever carry (header + max payload).
pub const MAX_FRAME: usize = cp::HEADER_SIZE + cp::MAX_PAYLOAD;
/// Internal buffer cap: older bytes are dropped if a garbage burst exceeds
/// this, so memory stays bounded even on a fully noisy line.
const MAX_BUFFER: usize = MAX_FRAME * 2;

/// A complete, validated coprocessor frame (payload copied out of the buffer).
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub header: cp::Header,
    pub payload: Vec<u8>,
}

impl Frame {
    /// Message type (a `cp::msg_type::*` constant).
    pub fn kind(&self) -> u8 {
        self.header.msg_type
    }
    /// Coprocessor-local sequence number (independent of the RF `seq`).
    pub fn seq(&self) -> u32 {
        self.header.seq
    }
}

/// Consumes a byte stream and yields aligned [`Frame`]s.
#[derive(Debug, Default)]
pub struct FrameDecoder {
    buf: Vec<u8>,
}

impl FrameDecoder {
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
    pub fn next(&mut self) -> Option<Frame> {
        loop {
            if self.buf.len() < cp::HEADER_SIZE {
                return None;
            }
            // Peek the header without consuming. `Header` is `repr(C, packed)`
            // so it must be read unaligned and by value.
            let hdr: cp::Header =
                unsafe { (self.buf.as_ptr() as *const cp::Header).read_unaligned() };
            let magic = hdr.magic;
            let version = hdr.version;
            let payload_len = hdr.payload_len as usize;
            let total = cp::HEADER_SIZE + payload_len;

            if magic != cp::MAGIC || version != cp::VERSION || total > MAX_FRAME {
                // Not a valid header (garbage, or a stray byte between frames):
                // drop one byte and re-scan.
                self.buf.remove(0);
                continue;
            }
            if self.buf.len() < total {
                return None; // frame still arriving
            }
            match cp::parse(&self.buf[..total]) {
                Some((header, payload)) => {
                    // Copy the payload out so the borrow of `self.buf` ends
                    // before we mutate it below.
                    let payload_owned = payload.to_vec();
                    self.buf.drain(..total);
                    return Some(Frame {
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

    /// Encode a frame using the real `cp::build` encoder, so tests exercise the
    /// decoder against the exact wire format.
    fn frame_bytes(kind: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; MAX_FRAME];
        let hdr = cp::Header {
            magic: cp::MAGIC,
            version: cp::VERSION,
            msg_type: kind,
            flags: 0,
            reserved: 0,
            seq: 7,
            payload_len: 0,
            crc16: 0,
        };
        let n = cp::build(&mut buf, &hdr, payload);
        buf[..n].to_vec()
    }

    #[test]
    fn single_frame() {
        let mut d = FrameDecoder::new();
        d.feed(&frame_bytes(cp::msg_type::FEATURES, &[1, 2, 3, 4]));
        let f = d.next().expect("one frame");
        assert_eq!(f.kind(), cp::msg_type::FEATURES);
        assert_eq!(f.seq(), 7);
        assert_eq!(f.payload, vec![1, 2, 3, 4]);
        assert!(d.next().is_none());
        assert_eq!(d.buffered(), 0);
    }

    #[test]
    fn split_across_feeds() {
        let bytes = frame_bytes(cp::msg_type::STATUS, &[9, 9, 9]);
        let mut d = FrameDecoder::new();
        assert!(d.next().is_none());
        d.feed(&bytes[..5]);
        assert!(d.next().is_none(), "header incomplete");
        d.feed(&bytes[5..]);
        let f = d.next().expect("frame completes once last bytes arrive");
        assert_eq!(f.kind(), cp::msg_type::STATUS);
        assert_eq!(f.payload, vec![9, 9, 9]);
        assert!(d.next().is_none());
    }

    #[test]
    fn garbage_prefix_resyncs() {
        let bytes = frame_bytes(cp::msg_type::CONFIG, &[0xAA]);
        let mut noisy = vec![0u8; 3];
        noisy.extend_from_slice(&[0xFF, 0x00, 0x42]); // junk before the frame
        noisy.extend_from_slice(&bytes);
        let mut d = FrameDecoder::new();
        d.feed(&noisy);
        let f = d.next().expect("rescans past garbage");
        assert_eq!(f.kind(), cp::msg_type::CONFIG);
        assert_eq!(f.payload, vec![0xAA]);
    }

    #[test]
    fn concatenated_frames() {
        let a = frame_bytes(cp::msg_type::RAW_CSI, &[1, 1]);
        let b = frame_bytes(cp::msg_type::SPECTROGRAM, &[2, 2, 2]);
        let mut d = FrameDecoder::new();
        d.feed(&a);
        d.feed(&b);
        assert_eq!(d.next().unwrap().kind(), cp::msg_type::RAW_CSI);
        assert_eq!(d.next().unwrap().kind(), cp::msg_type::SPECTROGRAM);
        assert!(d.next().is_none());
    }

    #[test]
    fn corrupt_frame_dropped_and_next_survives() {
        let mut bad = frame_bytes(cp::msg_type::ERROR, &[0x55]);
        let n = bad.len();
        bad[n - 1] ^= 0xFF; // flip the last payload byte → CRC fails
        let good = frame_bytes(cp::msg_type::STATUS, &[0x42]);
        let mut d = FrameDecoder::new();
        d.feed(&bad);
        d.feed(&good);
        let f = d.next().expect("resyncs past bad-CRC frame");
        assert_eq!(f.kind(), cp::msg_type::STATUS);
        assert_eq!(f.payload, vec![0x42]);
    }

    #[test]
    fn buffer_stays_bounded_under_garbage_burst() {
        let mut d = FrameDecoder::new();
        for _ in 0..(MAX_BUFFER / 16 + 4) {
            d.feed(&[0x11; 16]);
        }
        assert!(d.buffered() <= MAX_BUFFER, "buffered = {}", d.buffered());
        assert!(d.next().is_none());
    }

    #[test]
    fn empty_payload_roundtrips() {
        let bytes = frame_bytes(cp::msg_type::HELLO, &[]);
        let mut d = FrameDecoder::new();
        d.feed(&bytes);
        let f = d.next().expect("empty-payload frame");
        assert_eq!(f.kind(), cp::msg_type::HELLO);
        assert!(f.payload.is_empty());
    }
}
