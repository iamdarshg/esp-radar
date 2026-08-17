//! Versioned wired coprocessor protocol for the optional RP2350 DSP/compute
//! coprocessor (spec §12).
//!
//! The RP2350 is a NON-Wi-Fi compute node with no RF responsibility. It
//! connects to RADAR-TX over UART2 (GPIO17 TX / GPIO16 RX, 3V3 logic) or,
//! optionally, SPI. RADAR-TX must operate correctly with no coprocessor
//! present: the link is best-effort and self-healing.

use crate::crc::crc16_ext;

pub const MAGIC: u32 = 0x5243_4F50; // "RCOP"
pub const VERSION: u8 = 1;
pub const MAX_PAYLOAD: usize = 1024;

/// Message types.
pub mod msg_type {
    pub const HELLO: u8 = 1;
    pub const HELLO_ACK: u8 = 2;
    pub const FEATURES: u8 = 3;
    pub const SPECTROGRAM: u8 = 4;
    pub const CONFIG: u8 = 5;
    pub const STATUS: u8 = 6;
    pub const RAW_CSI: u8 = 7;
    pub const ERROR: u8 = 8;
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Header {
    pub magic: u32,
    pub version: u8,
    pub msg_type: u8,
    pub flags: u8,
    pub reserved: u8,
    pub seq: u32, // coprocessor-local sequence (independent of RF seq)
    pub payload_len: u16,
    pub crc16: u16,
}

pub const HEADER_SIZE: usize = core::mem::size_of::<Header>();

/// Coprocessor capability flags.
pub mod cap {
    pub const STFT: u16 = 0x01;
    pub const PCA: u16 = 0x02;
    pub const FILTER: u16 = 0x04;
    pub const COMPRESS: u16 = 0x08;
    pub const DISPLAY: u16 = 0x10;
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HelloAck {
    pub caps: u16,         // cap::* bitmask
    pub max_rate_hz: u16,  // max stream rate it can sustain
    pub fft_size_log2: u8, // largest supported FFT, as log2
    pub reserved: [u8; 3],
    pub fw_version: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Config {
    pub stream_rate_hz: u16,
    pub fft_size_log2: u8,
    pub spectrogram_rate_hz: u8,
    pub n_pca_components: u8,
    pub reserved: [u8; 3],
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    pub uptime_s: u32,
    pub heap_free: u16,
    pub queue_depth_pct: u8,
    pub dsp_load_pct: u8,
}

/// Serialize `header` + `payload` into `dst` (fills CRC). Returns bytes
/// written, or 0 if `dst` is too small.
pub fn build(dst: &mut [u8], hdr: &Header, payload: &[u8]) -> usize {
    let mut h = *hdr;
    h.payload_len = payload.len() as u16;
    let total = HEADER_SIZE + h.payload_len as usize;
    if dst.len() < total {
        return 0;
    }
    unsafe {
        let src = core::slice::from_raw_parts((&h as *const Header) as *const u8, HEADER_SIZE);
        dst[..HEADER_SIZE].copy_from_slice(src);
    }
    dst[HEADER_SIZE..total].copy_from_slice(payload);
    let mut crc_hdr = h;
    crc_hdr.crc16 = 0;
    let crc = unsafe {
        let src =
            core::slice::from_raw_parts((&crc_hdr as *const Header) as *const u8, HEADER_SIZE);
        crc16_ext(crc16_ext(0, src), payload)
    };
    dst[HEADER_SIZE - 2..HEADER_SIZE].copy_from_slice(&crc.to_le_bytes());
    total
}

/// Parse and validate a coprocessor frame.
pub fn parse(src: &[u8]) -> Option<(Header, &[u8])> {
    if src.len() < HEADER_SIZE {
        return None;
    }
    let hdr: Header = unsafe { (src.as_ptr() as *const Header).read_unaligned() };
    if hdr.magic != MAGIC || hdr.version != VERSION {
        return None;
    }
    let total = HEADER_SIZE + hdr.payload_len as usize;
    if src.len() < total {
        return None;
    }
    let payload = &src[HEADER_SIZE..total];
    let stored = hdr.crc16;
    let mut crc_hdr = hdr;
    crc_hdr.crc16 = 0;
    let crc = unsafe {
        let src =
            core::slice::from_raw_parts((&crc_hdr as *const Header) as *const u8, HEADER_SIZE);
        crc16_ext(crc16_ext(0, src), payload)
    };
    if crc != stored {
        return None;
    }
    Some((hdr, payload))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let mut buf = [0u8; 256];
        let hdr = Header {
            magic: MAGIC,
            version: VERSION,
            msg_type: msg_type::HELLO_ACK,
            ..Default::default()
        };
        let ack = HelloAck {
            caps: cap::STFT | cap::PCA,
            max_rate_hz: 100,
            fft_size_log2: 8,
            reserved: [0; 3],
            fw_version: 0x0100_0001,
        };
        let pl = unsafe {
            core::slice::from_raw_parts(
                (&ack as *const HelloAck) as *const u8,
                core::mem::size_of::<HelloAck>(),
            )
        };
        let n = build(&mut buf, &hdr, pl);
        let (parsed, pl2) = parse(&buf[..n]).expect("parse");
        let parsed_type = parsed.msg_type;
        assert_eq!(parsed_type, msg_type::HELLO_ACK);
        let back: HelloAck = unsafe { (pl2.as_ptr() as *const HelloAck).read_unaligned() };
        let caps = back.caps;
        assert_eq!(caps, cap::STFT | cap::PCA);
    }
}
