//! ESP-IDF Wi-Fi CSI capture for the radar receiver (spec §14).
//!
//! The CSI callback runs in the Wi-Fi task context and MUST stay extremely
//! short: it only `memcpy`s the raw bytes into a preallocated slot of a
//! lock-free single-producer/single-consumer ring and advances an atomic index.
//! All parsing, decoding and DSP happens on the radar task, which pops frames
//! from the other end.
//!
//! Decoding of the raw interleaved I/Q buffer into per-subcarrier
//! amplitude/phase lives in [`radar_dsp::transform::decode_channel`]; this
//! crate stays thin and only carries the metadata + bytes.

/// ESP-IDF binding (WiFi CSI config/callback). Only present when the `device`
/// feature is enabled (the ring itself is host-testable without it).
#[cfg(feature = "device")]
pub mod wifi;

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};

/// Upper bound on one HT20 CSI frame (64 FFT bins × I/Q i8 pairs, plus slack
/// for the HT-LTF structure). The real length is reported by `info.len`.
pub const MAX_CSI_LEN: usize = 256;

/// Metadata extracted in the CSI callback.
#[derive(Clone, Copy, Debug, Default)]
pub struct CsiInfo {
    pub rssi: i16,           // dBm (from rx_ctrl bitfield)
    pub noise_floor: i16,    // dBm (from rx_ctrl bitfield)
    pub channel: i8,         // primary channel
    pub sig_mode: u8,        // 0 = non-HT, 1 = HT
    pub mcs: u8,
    pub cwb: u8,             // 0 = 20 MHz, 1 = 40 MHz
    pub timestamp_us: u32,
    pub sig_len: u16,
    pub first_word_invalid: bool,
    pub mac: [u8; 6],        // source MAC of the CSI packet
    pub rx_seq: u16,         // packet rx sequence number (pairs with TX seq)
}

/// One raw CSI observation, owned by the consumer task.
#[derive(Clone, Debug)]
pub struct CsiFrame {
    pub info: CsiInfo,
    pub buf: Vec<i8>,
}

struct Slot {
    info: CsiInfo,
    buf: Vec<i8>, // capacity = MAX_CSI_LEN, preallocated at construction
    len: usize,
}

/// Lock-free SPSC ring of raw CSI frames.
///
/// * Producer: the Wi-Fi task (CSI callback).
/// * Consumer: the radar DSP task.
///
/// The two sides only ever touch disjoint slots (guarded by `head`/`tail`), so
/// sharing the ring across tasks is sound. Slots are preallocated so the
/// callback never allocates, locks, or blocks.
pub struct CsiRing {
    slots: UnsafeCell<Box<[Slot]>>,
    cap: usize,
    head: AtomicUsize,   // next slot the producer fills
    tail: AtomicUsize,   // next slot the consumer reads
    overflow: AtomicUsize,
}

// Safety: producer and consumer access disjoint slots; all shared state is
// guarded by the atomic head/tail indexes.
unsafe impl Sync for CsiRing {}

impl CsiRing {
    pub fn new(cap: usize) -> Self {
        let cap = cap.max(2);
        let slots: Vec<Slot> = (0..cap)
            .map(|_| Slot {
                info: CsiInfo::default(),
                buf: alloc::vec![0i8; MAX_CSI_LEN],
                len: 0,
            })
            .collect();
        Self {
            slots: UnsafeCell::new(slots.into_boxed_slice()),
            cap,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            overflow: AtomicUsize::new(0),
        }
    }

    pub fn capacity(&self) -> usize {
        self.cap
    }

    /// Monotonic count of frames dropped because the ring was full.
    pub fn overflow_count(&self) -> u32 {
        self.overflow.load(Ordering::Relaxed) as u32
    }

    /// Producer side — call ONLY from the CSI callback.
    pub fn push(&self, info: CsiInfo, buf: &[i8]) {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        let next = (head + 1) % self.cap;
        if next == tail {
            // Full: drop and count. The radar may over-run at high CSI rates.
            self.overflow.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let slot = unsafe { &mut (*self.slots.get())[head] };
        slot.info = info;
        let n = buf.len().min(MAX_CSI_LEN);
        slot.buf[..n].copy_from_slice(&buf[..n]);
        slot.len = n;
        self.head.store(next, Ordering::Release);
    }

    /// Consumer side — call from the radar task. `None` when empty.
    pub fn pop(&self) -> Option<CsiFrame> {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        let slot = unsafe { &(*self.slots.get())[tail] };
        let frame = CsiFrame {
            info: slot.info,
            buf: slot.buf[..slot.len].to_vec(),
        };
        self.tail.store((tail + 1) % self.cap, Ordering::Release);
        Some(frame)
    }

    pub fn is_empty(&self) -> bool {
        self.tail.load(Ordering::Acquire) == self.head.load(Ordering::Acquire)
    }

    pub fn len(&self) -> usize {
        let h = self.head.load(Ordering::Acquire);
        let t = self.tail.load(Ordering::Acquire);
        (h + self.cap - t) % self.cap
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_pop_roundtrip() {
        let ring = CsiRing::new(8);
        let info = CsiInfo {
            rssi: -55,
            noise_floor: -98,
            channel: 6,
            mac: [1, 2, 3, 4, 5, 6],
            rx_seq: 7,
            ..Default::default()
        };
        let buf: Vec<i8> = (0..40).map(|i| (i % 7) as i8).collect();
        ring.push(info, &buf);
        assert_eq!(ring.len(), 1);
        let f = ring.pop().expect("frame");
        assert_eq!(f.info.rssi, -55);
        assert_eq!(f.info.rx_seq, 7);
        assert_eq!(f.buf.len(), 40);
        assert_eq!(f.buf[39], (39 % 7) as i8);
        assert!(ring.pop().is_none());
    }

    #[test]
    fn overflow_drops_without_panic() {
        let ring = CsiRing::new(2); // cap 2 → at most 1 occupied slot
        for i in 0..100u32 {
            ring.push(CsiInfo { rx_seq: i as u16, ..Default::default() }, &[1, 2, 3]);
        }
        // Ring can hold at most 1 frame; 99 were dropped.
        assert_eq!(ring.overflow_count(), 99);
        assert_eq!(ring.len(), 1);
    }

    #[test]
    fn long_buf_is_truncated() {
        let ring = CsiRing::new(4);
        let big = alloc::vec![5i8; MAX_CSI_LEN + 100];
        ring.push(CsiInfo::default(), &big);
        let f = ring.pop().unwrap();
        assert_eq!(f.buf.len(), MAX_CSI_LEN);
    }
}
