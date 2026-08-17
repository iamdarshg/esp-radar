//! Transport between the three nodes of the compact radar head (spec §15).
//!
//! Physical topology is fixed: RADAR-TX is the AP, RADAR-RX1 and RADAR-RX2 are
//! stations, all within a few centimetres of each other.
//!
//! ```text
//! TX ── broadcast DataFrame (seq n) ──▶ RX1  ── FeatureReport ──▶ TX
//!                                     └─▶ RX2  ── FeatureReport ──▶ TX
//! ```
//!
//! * **TX → RX** measurement traffic is a single UDP broadcast per sequence
//!   number (200/s); both receivers observe the SAME packets, so their CSI is
//!   naturally sequence-aligned (spec §15).
//! * **RX → TX** is a unicast `FeatureReport` every `report_every` frames. The
//!   report carries the last processed `seq`, so TX can pair the two links'
//!   observations even though the receivers are not sample-synchronised.
//! * TX does NOT assume the two ESP32s have coherent RF oscillators — pairing
//!   is purely sequence-number based (spec §15).
//!
//! This crate is split in two:
//!   * **Pure logic** (host-tested): [`SequenceTracker`] (gap/loss tracking),
//!     [`Pairer`] (cross-link feature pairing), [`WindowCounter`] (events/s),
//!     [`TransportStats`].
//!   * **Device binding** ([`udp`], `feature = "device"`): thin lwIP UDP socket
//!     wrapper plus [`udp::TrafficSender`] / [`udp::FeatureReporter`].

extern crate alloc;

use alloc::collections::VecDeque;
use core::fmt;

pub use radar_protocol::{node, CalResp, CsiSnapshot, FeatureReport, Header};

/// UDP port for TX → RX measurement traffic (both RX listen here).
pub const MEASURE_PORT: u16 = 4444;
/// UDP port for RX → TX feature reports (TX listens here).
pub const REPORT_PORT: u16 = 4445;

/// IPv4 address helper (octets in memory order).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Ipv4Addr(pub [u8; 4]);

impl Ipv4Addr {
    pub const UNSPECIFIED: Ipv4Addr = Ipv4Addr([0, 0, 0, 0]);
    /// RADAR-TX AP address on its own subnet (192.168.4.1).
    pub const AP: Ipv4Addr = Ipv4Addr([192, 168, 4, 1]);
    /// Subnet-directed broadcast for the AP's subnet.
    pub const AP_BROADCAST: Ipv4Addr = Ipv4Addr([192, 168, 4, 255]);

    pub fn octets(&self) -> [u8; 4] {
        self.0
    }
}

impl fmt::Display for Ipv4Addr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}.{}", self.0[0], self.0[1], self.0[2], self.0[3])
    }
}

/// Result of feeding a sequence number to a [`SequenceTracker`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SeqEvent {
    /// The seq that was observed.
    pub seq: u32,
    /// True when this seq was NOT the expected next one (a gap or a resync).
    pub jumped: bool,
    /// Number of sequences skipped (0 for in-order).
    pub gap: u32,
    /// True when we resynchronised (the observed seq was *behind* expected,
    /// e.g. a stream restart or a late frame).
    pub resync: bool,
}

/// Monitors a monotonically increasing sequence-number stream (TX's global
/// packet counter) and reports gaps. Handles u32 wraparound.
pub struct SequenceTracker {
    expected: u32,
    initialized: bool,
    total: u64,
    gaps: u64,
    lost: u64,
    resyncs: u64,
}

impl Default for SequenceTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl SequenceTracker {
    pub fn new() -> Self {
        Self {
            expected: 0,
            initialized: false,
            total: 0,
            gaps: 0,
            lost: 0,
            resyncs: 0,
        }
    }

    /// Feed the next observed sequence number. Returns what happened.
    pub fn observe(&mut self, seq: u32) -> SeqEvent {
        if !self.initialized {
            self.initialized = true;
            self.expected = seq.wrapping_add(1);
            self.total += 1;
            return SeqEvent {
                seq,
                jumped: false,
                gap: 0,
                resync: false,
            };
        }
        let gap = seq.wrapping_sub(self.expected);
        if gap == 0 {
            // In order.
            self.expected = seq.wrapping_add(1);
            self.total += 1;
            SeqEvent {
                seq,
                jumped: false,
                gap: 0,
                resync: false,
            }
        } else if gap > 0x8000_0000 {
            // Observed seq behind expected: stream restart or a very late
            // frame. Treat as resync, not a giant gap.
            self.resyncs += 1;
            self.total += 1;
            self.expected = seq.wrapping_add(1);
            SeqEvent {
                seq,
                jumped: true,
                gap: 0,
                resync: true,
            }
        } else {
            // We skipped `gap` frames.
            self.gaps += 1;
            self.lost += gap as u64;
            self.total += 1 + gap as u64;
            self.expected = seq.wrapping_add(1);
            SeqEvent {
                seq,
                jumped: true,
                gap,
                resync: false,
            }
        }
    }

    /// Packet loss over the whole stream, 0..=1.
    pub fn loss_ratio(&self) -> f32 {
        if self.total == 0 {
            0.0
        } else {
            self.lost as f32 / self.total as f32
        }
    }

    pub fn lost(&self) -> u64 {
        self.lost
    }

    pub fn total(&self) -> u64 {
        self.total
    }

    pub fn gaps(&self) -> u64 {
        self.gaps
    }

    pub fn resyncs(&self) -> u64 {
        self.resyncs
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// A fixed-width sliding-window counter; computes events per second.
pub struct WindowCounter {
    samples: VecDeque<(u64, u64)>, // (timestamp_us, running total)
    window_us: u64,
    total: u64,
}

impl WindowCounter {
    pub fn new(window_us: u64) -> Self {
        Self {
            samples: VecDeque::with_capacity(8),
            window_us,
            total: 0,
        }
    }

    pub fn push(&mut self, now_us: u64) {
        self.total += 1;
        self.samples.push_back((now_us, self.total));
        let cutoff = now_us.saturating_sub(self.window_us);
        while self.samples.front().is_some_and(|&(t, _)| t < cutoff) {
            self.samples.pop_front();
        }
    }

    /// Number of events inside the current window.
    pub fn count(&self, now_us: u64) -> u64 {
        let cutoff = now_us.saturating_sub(self.window_us);
        self.samples.iter().filter(|&&(t, _)| t >= cutoff).count() as u64
    }

    /// Events per second, measured over the window.
    pub fn rate(&self, now_us: u64) -> f32 {
        self.count(now_us) as f32 * 1_000_000.0 / self.window_us.max(1) as f32
    }

    pub fn total(&self) -> u64 {
        self.total
    }

    pub fn reset(&mut self) {
        self.samples.clear();
        self.total = 0;
    }
}

/// A report from one RX node, keyed by the TX sequence it covers.
#[derive(Clone, Copy, Debug)]
pub struct StampedReport {
    /// Last TX seq this report covers.
    pub seq: u32,
    /// Report body.
    pub report: FeatureReport,
    /// Local receive timestamp (µs) for latency tracking.
    pub rx_t_us: u64,
}

/// One time-aligned pair of RX1/RX2 reports, ready for fusion.
#[derive(Clone, Copy, Debug)]
pub struct PairedFrames {
    /// Common sequence number the pair is aligned on (the later of the two).
    pub seq: u32,
    pub rx1: FeatureReport,
    pub rx2: FeatureReport,
    /// Difference between the two reports' seq numbers (alignment quality:
    /// smaller is better).
    pub seq_delta: u32,
}

/// Pairs RX1 and RX2 [`FeatureReport`]s by TX sequence number.
///
/// The two receivers are independent and report at their own cadence, so the
/// reports arrive interleaved with a phase offset of up to one report window.
/// This buffer holds the most recent reports per link, greedily pairs the
/// nearest neighbours within `tolerance` frames, and eventually drops reports
/// that can never be paired (partner already far ahead).
pub struct Pairer {
    rx1: VecDeque<StampedReport>,
    rx2: VecDeque<StampedReport>,
    /// Max seq distance for two reports to be considered the "same" window.
    tolerance: u32,
    max_buf: usize,
    pub pairs_total: u64,
    pub dropped_unpaired: u64,
    /// Per-second pair throughput.
    pub pair_rate: WindowCounter,
}

impl Pairer {
    /// `tolerance` — in TX sequence units, the maximum distance between an RX1
    /// report's seq and an RX2 report's seq for them to be paired. With a
    /// report window of `W` frames, a tolerance of `W / 2` covers the worst
    /// phase offset between the two links.
    pub fn new(tolerance: u32) -> Self {
        Self {
            rx1: VecDeque::new(),
            rx2: VecDeque::new(),
            tolerance,
            max_buf: 64,
            pairs_total: 0,
            dropped_unpaired: 0,
            pair_rate: WindowCounter::new(1_000_000),
        }
    }

    /// Buffer a report from one link (`node::RX1` or `node::RX2`).
    pub fn push(&mut self, node_id: u8, report: FeatureReport, now_us: u64) {
        let stamped = StampedReport {
            seq: report.seq,
            report,
            rx_t_us: now_us,
        };
        let q = match node_id {
            n if n == node::RX1 => &mut self.rx1,
            n if n == node::RX2 => &mut self.rx2,
            _ => return,
        };
        // Insert keeping seq order (reports arrive roughly in order; rare
        // reordering is handled by the linear insert on a small buffer).
        let pos = q
            .iter()
            .position(|r| r.seq > stamped.seq)
            .unwrap_or(q.len());
        q.insert(pos, stamped);
        while q.len() > self.max_buf {
            q.pop_front();
        }
    }

    /// Extract the next time-aligned pair, if one is available.
    ///
    /// Greedy nearest-neighbour: look at the oldest RX1 report, find the RX2
    /// report with the smallest |seq delta| within tolerance. If the oldest
    /// RX2 report is already *ahead* of the RX1 report by more than tolerance,
    /// the RX1 report can never pair → drop it. Symmetric for the other side.
    pub fn next_pair(&mut self, now_us: u64) -> Option<PairedFrames> {
        loop {
            if self.rx1.is_empty() || self.rx2.is_empty() {
                return None;
            }
            let r1 = self.rx1.front().copied()?;
            let r2 = self.rx2.front().copied()?;

            // Signed difference handles seq wraparound and the "which side is
            // ahead" question without unsigned-wrap surprises.
            let front_diff = (r2.seq as i64) - (r1.seq as i64);
            let tol = self.tolerance as i64;
            // One side is already far ahead of the other's oldest report →
            // the lagging report is unpaired for good.
            if front_diff > tol {
                self.rx1.pop_front();
                self.dropped_unpaired += 1;
                continue;
            }
            if front_diff < -tol {
                self.rx2.pop_front();
                self.dropped_unpaired += 1;
                continue;
            }

            // Find the closest RX2 report to the oldest RX1 report.
            let mut best: Option<usize> = None;
            let mut best_delta = u32::MAX;
            for (i, cand) in self.rx2.iter().enumerate() {
                let delta = cand.seq.abs_diff(r1.seq);
                if delta <= self.tolerance && delta < best_delta {
                    best_delta = delta;
                    best = Some(i);
                }
                // RX2 is sorted ascending; once a candidate runs ahead past
                // tolerance, no later candidate can match either.
                if (cand.seq as i64) - (r1.seq as i64) > tol {
                    break;
                }
            }

            match best {
                Some(i) => {
                    let partner = self.rx2.remove(i).unwrap();
                    let older = self.rx1.pop_front().unwrap();
                    let (earlier, later) = if partner.seq >= older.seq {
                        (&older, &partner)
                    } else {
                        (&partner, &older)
                    };
                    self.pairs_total += 1;
                    self.pair_rate.push(now_us);
                    return Some(PairedFrames {
                        seq: later.seq,
                        rx1: older.report,
                        rx2: partner.report,
                        seq_delta: later.seq - earlier.seq,
                    });
                }
                None => {
                    // No partner within tolerance for the oldest RX1 report.
                    self.rx1.pop_front();
                    self.dropped_unpaired += 1;
                }
            }
        }
    }

    pub fn buffered(&self) -> (usize, usize) {
        (self.rx1.len(), self.rx2.len())
    }

    pub fn reset(&mut self) {
        self.rx1.clear();
        self.rx2.clear();
        self.pairs_total = 0;
        self.dropped_unpaired = 0;
        self.pair_rate.reset();
    }
}

/// Transport-level telemetry for the dashboard (spec §6 "LIVE STATUS").
#[derive(Clone, Copy, Debug, Default)]
pub struct TransportStats {
    pub tx_frames: u64,
    pub tx_seq: u32,
    pub rx_frames: u64,
    pub rx_drops: u64,
    pub paired_frames: u64,
    /// RX1 loss ratio (0..1) over its whole report stream.
    pub rx1_loss: f32,
    /// RX2 loss ratio (0..1).
    pub rx2_loss: f32,
}

/// Build a `DataFrame` measurement packet into `dst`. Returns bytes written,
/// or 0 if `dst` is too small.
pub fn build_data_frame(
    dst: &mut [u8],
    src: u8,
    seq: u32,
    t_us: u64,
    tx_power_db: u8,
    cal: bool,
) -> usize {
    let payload = radar_protocol::DataPayload {
        tx_power_db,
        flags: if cal {
            radar_protocol::data_flags::CAL
        } else {
            0
        },
    };
    let pl = unsafe {
        core::slice::from_raw_parts(
            (&payload as *const radar_protocol::DataPayload) as *const u8,
            core::mem::size_of::<radar_protocol::DataPayload>(),
        )
    };
    let hdr = Header::new(
        radar_protocol::frame_type::DATA_FRAME,
        src,
        0,
        seq,
        t_us,
        pl.len() as u16,
    );
    radar_protocol::build(dst, &hdr, pl)
}

/// Parse a received radar frame from `buf`. Returns (kind, src, seq, payload).
/// Validates magic/version/CRC via `radar_protocol::parse`.
pub fn parse_frame(buf: &[u8]) -> Option<(u8, u8, u32, &[u8])> {
    let (hdr, payload) = radar_protocol::parse(buf)?;
    Some((hdr.kind, hdr.src_node, hdr.seq, payload))
}

/// Serialize a feature report (as sent by an RX node).
pub fn build_feature_report(dst: &mut [u8], src: u8, report: &FeatureReport, t_us: u64) -> usize {
    let pl = unsafe {
        core::slice::from_raw_parts(
            (report as *const FeatureReport) as *const u8,
            core::mem::size_of::<FeatureReport>(),
        )
    };
    let hdr = Header::new(
        radar_protocol::frame_type::FEATURE_REPORT,
        src,
        node::TX,
        report.seq,
        t_us,
        pl.len() as u16,
    );
    radar_protocol::build(dst, &hdr, pl)
}

/// Deserialize a feature report payload.
pub fn parse_feature_report(payload: &[u8]) -> Option<FeatureReport> {
    if payload.len() < core::mem::size_of::<FeatureReport>() {
        return None;
    }
    Some(unsafe { (payload.as_ptr() as *const FeatureReport).read_unaligned() })
}

/// Build a `CalResp` (RX -> TX calibration reply) into `dst`. Returns bytes
/// written, or 0 if `dst` is too small. The header `seq` carries the stage so
/// the response is traceable in TX logs; the TX matches on the payload's
/// `stage` field, not the header.
pub fn build_cal_resp(dst: &mut [u8], src: u8, resp: &CalResp, t_us: u64) -> usize {
    let pl = unsafe {
        core::slice::from_raw_parts(
            (resp as *const CalResp) as *const u8,
            core::mem::size_of::<CalResp>(),
        )
    };
    let hdr = Header::new(
        radar_protocol::frame_type::CAL_RESP,
        src,
        node::TX,
        resp.stage as u32,
        t_us,
        pl.len() as u16,
    );
    radar_protocol::build(dst, &hdr, pl)
}

/// Build a `CsiSnapshot` (RX → TX, a few Hz) into `dst`. Returns bytes
/// written, or 0 if `dst` is too small. Header `seq` mirrors the snapshot's
/// own `seq` (the last TX measurement sequence covered) so TX logs trace it
/// against the report stream.
pub fn build_csi_snapshot(dst: &mut [u8], src: u8, snap: &CsiSnapshot, t_us: u64) -> usize {
    let pl = unsafe {
        core::slice::from_raw_parts(
            (snap as *const CsiSnapshot) as *const u8,
            core::mem::size_of::<CsiSnapshot>(),
        )
    };
    let hdr = Header::new(
        radar_protocol::frame_type::CSI_SNAPSHOT,
        src,
        node::TX,
        snap.seq,
        t_us,
        pl.len() as u16,
    );
    radar_protocol::build(dst, &hdr, pl)
}

#[cfg(feature = "device")]
pub mod udp;

/// Byte-stream framing for the wired inter-board UART links (host-pure).
///
/// The UDP path consumes whole datagrams; a UART gives a byte stream with no
/// packet boundaries. [`framer::RadarFrameDecoder`] buffers bytes, hunts the
/// "RDR1" magic, validates CRC, and resyncs on garbage — the wired sibling of
/// the [`udp`] module, sharing the same `radar_protocol` frame format.
pub mod framer;

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_report(seq: u32, energy: f32) -> FeatureReport {
        FeatureReport {
            seq,
            motion_energy: energy,
            rssi: -55,
            ..Default::default()
        }
    }

    #[test]
    fn sequence_tracker_in_order() {
        let mut t = SequenceTracker::new();
        assert!(!t.observe(0).jumped);
        assert!(!t.observe(1).jumped);
        assert!(!t.observe(2).jumped);
        assert_eq!(t.gaps(), 0);
        assert_eq!(t.lost(), 0);
        assert_eq!(t.total(), 3);
        assert_eq!(t.loss_ratio(), 0.0);
    }

    #[test]
    fn sequence_tracker_gap() {
        let mut t = SequenceTracker::new();
        t.observe(10);
        let ev = t.observe(15);
        assert!(ev.jumped);
        assert_eq!(ev.gap, 4); // 11..14 skipped
        assert_eq!(t.lost(), 4);
        assert_eq!(t.total(), 6);
        assert!((t.loss_ratio() - 4.0 / 6.0).abs() < 1e-6);
    }

    #[test]
    fn sequence_tracker_wraparound() {
        let mut t = SequenceTracker::new();
        t.observe(u32::MAX - 1);
        assert!(!t.observe(u32::MAX).jumped); // in order right up to the top
        let ev = t.observe(0); // wraps cleanly, nothing skipped
        assert!(!ev.jumped);
        assert_eq!(t.gaps(), 0);
        assert_eq!(t.lost(), 0);
        // A genuine backward jump is a resync, not a huge gap.
        let mut t2 = SequenceTracker::new();
        t2.observe(100);
        let ev2 = t2.observe(5);
        assert!(ev2.resync);
        assert_eq!(ev2.gap, 0);
    }

    #[test]
    fn window_counter_rates() {
        let mut wc = WindowCounter::new(1_000_000);
        for i in 0..5 {
            wc.push(i * 100_000); // 5 pushes over 0.4 s
        }
        assert_eq!(wc.rate(500_000), 5.0);
        // Old samples age out of the 1 s window.
        assert_eq!(wc.rate(1_500_000), 0.0);
    }

    #[test]
    fn pairer_pairs_aligned_reports() {
        let mut p = Pairer::new(10);
        for i in 0..5u32 {
            p.push(
                node::RX1,
                sample_report(i * 20, i as f32),
                (i * 1000) as u64,
            );
            p.push(
                node::RX2,
                sample_report(i * 20, i as f32 * 2.0),
                (i * 1000) as u64,
            );
        }
        let mut pairs = 0;
        while let Some(_) = p.next_pair(1_000_000) {
            pairs += 1;
        }
        assert_eq!(pairs, 5);
        assert_eq!(p.pairs_total, 5);
        assert_eq!(p.dropped_unpaired, 0);
    }

    #[test]
    fn pairer_tolerates_phase_offset() {
        let mut p = Pairer::new(10);
        // RX2 reports half a window ahead of RX1.
        for i in 0..4u32 {
            p.push(node::RX1, sample_report(i * 20, 1.0), (i * 1000) as u64);
            p.push(
                node::RX2,
                sample_report(i * 20 + 10, 2.0),
                (i * 1000) as u64,
            );
        }
        let mut pairs = 0;
        while let Some(_) = p.next_pair(1_000_000) {
            pairs += 1;
        }
        // All 4 RX1 reports pair with the matching RX2 report.
        assert_eq!(pairs, 4);
        assert_eq!(p.dropped_unpaired, 0);
    }

    #[test]
    fn pairer_drops_orphans() {
        let mut p = Pairer::new(5);
        p.push(node::RX1, sample_report(100, 1.0), 0);
        // RX2 races far ahead.
        for i in 0..4 {
            p.push(node::RX2, sample_report(200 + i * 20, 2.0), 0);
        }
        // The RX1 report is far behind RX2's front → unpaired.
        let mut pairs = 0;
        while let Some(_) = p.next_pair(1_000_000) {
            pairs += 1;
        }
        assert_eq!(pairs, 0);
        assert_eq!(p.dropped_unpaired, 1);
        // RX2 reports remain buffered (no RX1 partner yet).
        assert_eq!(p.buffered(), (0, 4));
    }

    #[test]
    fn build_data_frame_roundtrip() {
        let mut buf = [0u8; 128];
        let n = build_data_frame(&mut buf, node::TX, 7, 1234, 40, true);
        assert!(n > 0);
        let (kind, src, seq, payload) = parse_frame(&buf[..n]).expect("parse");
        assert_eq!(kind, radar_protocol::frame_type::DATA_FRAME);
        assert_eq!(src, node::TX);
        assert_eq!(seq, 7);
        let dp: radar_protocol::DataPayload =
            unsafe { (payload.as_ptr() as *const radar_protocol::DataPayload).read_unaligned() };
        let power = dp.tx_power_db;
        let flags = dp.flags;
        assert_eq!(power, 40);
        assert_ne!(flags & radar_protocol::data_flags::CAL, 0);
    }

    #[test]
    fn feature_report_roundtrip() {
        let mut buf = [0u8; 512];
        let r = sample_report(99, 1.25);
        let n = build_feature_report(&mut buf, node::RX1, &r, 555);
        assert!(n > 0);
        let (kind, src, _seq, payload) = parse_frame(&buf[..n]).expect("parse");
        assert_eq!(kind, radar_protocol::frame_type::FEATURE_REPORT);
        assert_eq!(src, node::RX1);
        let back = parse_feature_report(payload).unwrap();
        let b_seq = back.seq;
        let b_energy = back.motion_energy;
        assert_eq!(b_seq, 99);
        assert!((b_energy - 1.25).abs() < 1e-6);
    }
}
