//! QEMU RF-sim CSI source: feeds the [`CsiRing`] from a raw flash partition.
//!
//! In sim mode (`RadarConfig::sim_mode == 1`) the firmware cannot associate
//! with a real AP, so there is no WiFi CSI callback. Instead the boot task
//! locates the `simdata` flash partition — a raw blob written by the host
//! rf-sim scenario generator — and this module streams its ESP32-CSI-format
//! frames into the ring at the configured rate, paced by `esp_timer` so the
//! radar loop sees real per-packet arrival jitter. Every pushed frame carries
//! `timestamp_us` stamped at push time and `rx_seq` = the scenario frame
//! index, so the analyzer can correlate emitted CSI_PHASE telemetry with the
//! generator's ground truth even across dropped frames.
//!
//! Blob layout (partition label `simdata`, little-endian):
//!
//! ```text
//! offset 0   u32 magic = 0x444D4953 ("SIMD")
//! offset 4   u8  version (= 1)
//! offset 5   u8  channel  (primary Wi-Fi channel the scenario simulates)
//! offset 6   u16 reserved
//! offset 8   u32 rate_hz  (intended frame rate)
//! offset 12  u32 n_frames (number of raw CSI records)
//! offset 16  i16 rssi     (constant per-frame metadata)
//! offset 18  i16 noise_floor
//! offset 20  u8  mac[6]   (AP BSSID the scenario emulates — the ring filter)
//! offset 26  u8  cwb
//! offset 27  u8  sig_mode
//! offset 28  u16 frame_len (= 128 for v1)
//! offset 30  u16 reserved
//! offset 32  n_frames × frame_len bytes of raw CSI (interleaved i8 I/Q,
//!             bin = subcarrier + 32, exactly the ESP32 HT20 CSI layout)
//! ```
//!
//! 128 B × n_frames: a 512 KB partition holds ~4092 frames (~20 s at 200 Hz).
//! Frames are streamed one `esp_partition_read` at a time — never buffered in
//! RAM — so the whole scenario does not have to fit in the ESP32 heap.

use radar_csi::{CsiInfo, CsiRing};
use radar_transport::udp::now_us;

/// Blob magic ("SIMD").
pub const SIM_MAGIC: u32 = 0x444D_4953;
/// Blob schema version.
pub const SIM_VERSION: u8 = 1;
/// Fixed header bytes before the frame records.
pub const SIM_HEADER_LEN: usize = 32;
/// Raw CSI buffer length per frame (64 FFT bins × interleaved i8 I/Q).
pub const SIM_FRAME_LEN: usize = 128;

/// BSSID label of the `simdata` partition, NUL-terminated for `CString`.
const SIMDATA_LABEL: [u8; 8] = *b"simdata\0";

/// Streams a pre-written RF scenario into the CSI ring at `rate_hz`, paced by
/// the esp-timer clock (QEMU virtual time). Owns no heap beyond one 128 B
/// scratch frame.
pub struct SimSource {
    /// `esp_partition_t` pointer from `esp_partition_find_first` — valid for
    /// the program lifetime (documented by the API). Raw pointers are not
    /// `Send`, so the handle gets a scoped `unsafe impl Send` below.
    part: *const esp_idf_sys::esp_partition_t,
    rate_hz: u32,
    n_frames: u32,
    channel: u8,
    cwb: u8,
    sig_mode: u8,
    rssi: i16,
    noise_floor: i16,
    /// Scenario MAC the ring filter matches against (`ap_bssid`).
    pub mac: [u8; 6],
    /// Next scenario frame index to push.
    next_idx: u32,
    /// esp-timer instant of the first push; the frame grid starts here.
    start_us: u64,
    /// First-push latch (start_us must be stamped on the first due tick).
    first: bool,
    /// Scratch frame buffer reused across reads.
    cur: [i8; SIM_FRAME_LEN],
}

impl SimSource {
    /// Locate the `simdata` partition and validate its header. `None` when the
    /// partition is missing or the blob is not one we understand — the caller
    /// logs loudly and the loop simply idles (no CSI arrives).
    pub fn open() -> Option<SimSource> {
        let part = unsafe {
            esp_idf_sys::esp_partition_find_first(
                esp_idf_sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
                esp_idf_sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
                SIMDATA_LABEL.as_ptr() as *const core::ffi::c_char,
            )
        };
        if part.is_null() {
            return None;
        }
        let mut hdr = [0u8; SIM_HEADER_LEN];
        let r = unsafe {
            esp_idf_sys::esp_partition_read(
                part,
                0,
                hdr.as_mut_ptr() as *mut core::ffi::c_void,
                SIM_HEADER_LEN,
            )
        };
        if r != esp_idf_sys::ESP_OK {
            return None;
        }
        let magic = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        if magic != SIM_MAGIC {
            log::error!("simdata: bad magic {magic:#x} (want {SIM_MAGIC:#x})");
            return None;
        }
        if hdr[4] != SIM_VERSION {
            log::error!("simdata: unsupported version {}", hdr[4]);
            return None;
        }
        let frame_len = u16::from_le_bytes([hdr[28], hdr[29]]) as usize;
        if frame_len != SIM_FRAME_LEN {
            log::error!("simdata: frame_len {frame_len} != {SIM_FRAME_LEN}");
            return None;
        }
        let mut mac = [0u8; 6];
        mac.copy_from_slice(&hdr[20..26]);
        Some(SimSource {
            part,
            rate_hz: u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]),
            n_frames: u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]),
            channel: hdr[5],
            cwb: hdr[26],
            sig_mode: hdr[27],
            rssi: i16::from_le_bytes([hdr[16], hdr[17]]),
            noise_floor: i16::from_le_bytes([hdr[18], hdr[19]]),
            mac,
            next_idx: 0,
            start_us: 0,
            first: true,
            cur: [0i8; SIM_FRAME_LEN],
        })
    }

    /// Number of frames the scenario holds (for the boot log).
    pub fn n_frames(&self) -> u32 {
        self.n_frames
    }

    /// Intended frame rate (for the boot log).
    pub fn rate_hz(&self) -> u32 {
        self.rate_hz
    }

    /// Push every scenario frame whose due instant has passed. Called at the
    /// top of the radar loop; `timestamp_us` is stamped here, at push time, so
    /// the emitted telemetry carries real arrival cadence (not consume-time).
    pub fn pump(&mut self, ring: &CsiRing) {
        if self.next_idx >= self.n_frames {
            return; // drained
        }
        let now = now_us();
        if self.first {
            self.first = false;
            self.start_us = now;
        }
        while self.next_idx < self.n_frames {
            let due = self.start_us + (self.next_idx as u64) * 1_000_000 / (self.rate_hz as u64);
            if now < due {
                break; // nothing due yet
            }
            let roff = SIM_HEADER_LEN + (self.next_idx as usize) * SIM_FRAME_LEN;
            let n = self.read_frame(roff);
            if n == 0 {
                log::error!("simdata: partition read failed at offset {roff}");
                self.next_idx = self.n_frames; // stop the feed on I/O error
                break;
            }
            let info = CsiInfo {
                rssi: self.rssi,
                noise_floor: self.noise_floor,
                channel: self.channel as i8,
                sig_mode: self.sig_mode,
                mcs: 7,
                cwb: self.cwb,
                timestamp_us: now_us() as u32,
                sig_len: n as u16,
                first_word_invalid: false,
                mac: self.mac,
                // Scenario frame index, so the analyzer can correlate emitted
                // CSI_PHASE with ground truth across any dropped frames.
                rx_seq: (self.next_idx & 0xFFFF) as u16,
            };
            if !ring.push(info, &self.cur[..n]) {
                // Ring full (the consumer is slower than the due grid — a
                // QEMU artifact). Backpressure: do NOT advance past this
                // frame; retry it on the next pump so every scenario frame is
                // delivered in order. The WiFi path drops on overflow (fresh
                // is better than stale), but the sim feed must be lossless —
                // the analyzer correlates emitted phase with ground truth by
                // `rx_seq`, so a silently-dropped frame is a permanent gap.
                break;
            }
            self.next_idx += 1;
        }
    }
}

// Safety: `part` points into the flash partition-table region and is valid for
// the program lifetime (documented by `esp_partition_find_first`); the handle
// never owns or frees it. `SimSource` is only ever touched by the single radar
// thread, so moving the handle across the spawn boundary is sound.
unsafe impl Send for SimSource {}

impl SimSource {
    /// Read one 128 B raw CSI record into `self.cur`. Returns bytes read
    /// (0 on error).
    fn read_frame(&mut self, offset: usize) -> usize {
        let r = unsafe {
            esp_idf_sys::esp_partition_read(
                self.part,
                offset,
                self.cur.as_mut_ptr() as *mut core::ffi::c_void,
                SIM_FRAME_LEN,
            )
        };
        if r != esp_idf_sys::ESP_OK {
            return 0;
        }
        SIM_FRAME_LEN
    }
}
