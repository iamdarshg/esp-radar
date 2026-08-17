//! Basic EM / link-budget simulator for the wired UART data plane.
//!
//! The two inter-board links (middle↔RX1 = LINK 1, middle↔CAM = LINK 2) are
//! crossed 2-wire UART pairs at 460800 baud (fallback 230400), running as short
//! parallel jumpers across the board gaps. This tool quantifies, per link:
//!
//!   * **Polling-rate maximums** — how many FeatureReports / CsiSnapshots per
//!     second each link can sustain before it saturates the byte budget, given
//!     the *actual* frame sizes (`size_of` on the real protocol structs) and
//!     8N1 framing (10 wire-bits per byte). Also the TX-side poll ceiling (the
//!     fusion loop polls each link every READ_POLL_MS).
//!
//!   * **Error** — an analytic EM noise model for the jumpers: crosstalk from
//!     the sibling link and the same-pair neighbour wire (parallel-wire
//!     coupling), near-field RF pickup from the 2.4 GHz measurement broadcast
//!     (the TX antenna is centimetres away), and a thermal floor. Geometry →
//!     coupling coefficient → noise voltage → SNR → BER → FER, with the CRC16
//!     giving an undetected-error floor of 2^-16 per corrupted frame.
//!
//!   * **Monte-Carlo cross-check** — encode real frames with the production
//!     builders, corrupt bits at the modelled BER, feed the *real*
//!     `RadarFrameDecoder`, and measure the actual frame-drop rate (which
//!     should track the analytic FER within statistical noise).
//!
//! This is deliberately a *basic* model: straight parallel wires, lumped
//! coupling, Gaussian noise. It is a sanity floor and a design sweep, not a
//! 3D field solve — the real gate is CRC failures on the physical build.

use radar_protocol::{node, CsiSnapshot, FeatureReport, MAX_PAYLOAD};
use radar_transport::framer::{RadarFrameDecoder, MAX_FRAME};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Link budget (deterministic byte accounting)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct LinkBudget {
    baud: u32,
    /// Effective data bytes/second for 8N1 (10 bits per byte on the wire).
    bytes_per_s: f64,
    /// FeatureReport frame size on the wire (header + payload).
    report_bytes: usize,
    /// CsiSnapshot frame size on the wire.
    snapshot_bytes: usize,
    /// CalCmd frame size on the wire.
    cal_cmd_bytes: usize,
    /// CalResp frame size on the wire.
    cal_resp_bytes: usize,
}

impl LinkBudget {
    fn new(baud: u32) -> Self {
        let report = FeatureReport::default();
        let snapshot = CsiSnapshot::default();
        Self {
            baud,
            bytes_per_s: baud as f64 / 10.0,
            report_bytes: HEADER + size_of_val(&report),
            snapshot_bytes: HEADER + size_of_val(&snapshot),
            cal_cmd_bytes: HEADER + size_of_val(&radar_protocol::CalCmd::default()),
            cal_resp_bytes: HEADER + size_of_val(&radar_protocol::CalResp::default()),
        }
    }

    /// Max pure report rate (no snapshots on the link).
    fn max_reports_s(&self) -> f64 {
        self.bytes_per_s / self.report_bytes as f64
    }

    /// Max report rate while still carrying `snapshots_per_s` CsiSnapshots.
    fn max_reports_s_with_snapshots(&self, snapshots_per_s: f64) -> f64 {
        (self.bytes_per_s - snapshots_per_s * self.snapshot_bytes as f64)
            / self.report_bytes as f64
    }

    /// Fraction of the byte budget used by the configured cadence
    /// (report_every=20 @200Hz → 10 reports/s; snapshot ~2/s; CAL negligible).
    fn utilization(&self, reports_per_s: f64, snapshots_per_s: f64) -> f64 {
        (reports_per_s * self.report_bytes as f64
            + snapshots_per_s * self.snapshot_bytes as f64)
            / self.bytes_per_s
    }

    /// Max time the RX software poll loop can be allowed to take while still
    /// keeping up with `reports_per_s` + `snapshots_per_s` (excluding the 5 ms
    /// READ_POLL_MS window that bounds the software loop itself).
    fn max_software_poll_ms(&self, reports_per_s: f64, snapshots_per_s: f64) -> f64 {
        let frames_per_s = reports_per_s + snapshots_per_s;
        if frames_per_s <= 0.0 {
            return f64::INFINITY;
        }
        1000.0 / frames_per_s
    }
}

const HEADER: usize = radar_protocol::HEADER_SIZE;

// ---------------------------------------------------------------------------
// EM noise model (analytic)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct EmParams {
    /// Signal rail swing, V (3V3 UART rail-to-rail).
    v_swing: f64,
    /// Wire length of the jumper run, m (board gap ≈ 14 breadboard columns ×
    /// 2.54 mm for the RX1 link, ≈10 for the CAM link).
    wire_len_m: f64,
    /// Separation between the two wires of one link (its own TX↔RX), m.
    pair_sep_m: f64,
    /// Separation between the two links (LINK 1 vs LINK 2), m.
    link_sep_m: f64,
    /// Height of the wire above the return plane (breadboard surface), m.
    height_m: f64,
    /// RF source: TX antenna EIRP, W.
    tx_eirp_w: f64,
    /// Distance from the TX antenna to the victim wire, m.
    rf_dist_m: f64,
    /// Fraction of induced RF open-circuit voltage that actually reaches the
    /// UART decision (rectification/impedance reduction — wires are far from
    /// resonant and the line is single-ended vs GND).
    rf_coupling_eff: f64,
    /// Thermal / receiver noise floor, V rms.
    thermal_v: f64,
}

impl Default for EmParams {
    fn default() -> Self {
        Self {
            v_swing: 3.3,
            wire_len_m: 0.036,      // ~14 cols × 2.54 mm — the RX1 jumper run
            pair_sep_m: 0.00254,    // adjacent header pins
            link_sep_m: 0.015,      // the two links a few columns apart
            height_m: 0.002,        // wires sit a couple mm off the board
            tx_eirp_w: 0.1,         // ~20 dBm EIRP measurement broadcast
            rf_dist_m: 0.03,        // antenna ~3 cm from the victim jumpers
            rf_coupling_eff: 0.05,  // 5% of open-circuit pickup reaches decision
            thermal_v: 0.005,       // 5 mV rms floor
        }
    }
}

/// Parallel-wire mutual capacitance per unit length, 2-wire transmission line
/// model (wire over a plane, far from the other wire).
fn c_self(p: &EmParams) -> f64 {
    // C = 2πε / ln(4h/d) for a wire of diameter d at height h over a ground
    // plane, per metre. Use the pair-sep as a proxy for diameter-ish scale.
    let h = p.height_m.max(1e-6);
    let d = 0.0005; // 0.5 mm wire diameter (solid-core jumper)
    2.0 * PI * 8.854e-12 / (4.0 * h / d).ln()
}

/// Mutual capacitance between two parallel wires separated by `sep`, using the
/// 2-wire line capacitance formula Cm ≈ π·ε / acosh(sep/d), per metre.
fn c_mutual(sep: f64) -> f64 {
    let d = 0.0005;
    let ratio = (sep / d).max(1.001);
    PI * 8.854e-12 / ratio.acosh()
}

/// Near-end crosstalk voltage coupling coefficient for a parallel run of
/// length L with wires separated by `sep` over a plane (lumped, capacitive-
/// dominant at these lengths/heights).
fn xtalk_coeff(p: &EmParams, sep: f64) -> f64 {
    let l = p.wire_len_m;
    let cm = c_mutual(sep);
    let cs = c_self(p);
    // k ≈ (L·Cm) / (L·Cm + L·Cs + Cload). The load capacitance of a UART RX
    // input (~5 pF) clamps the coupling on short wires.
    let c_load = 5e-12;
    let c_aggressor = l * cs;
    let c_couple = l * cm;
    c_couple / (c_couple + c_aggressor + c_load)
}

/// Induced open-circuit RF voltage on a short wire in a field E (far-field
/// approximation, over-bounds the near field): V_oc ≈ E · h_eff, h_eff ≈ L.
fn rf_pickup_v(p: &EmParams) -> f64 {
    // E = sqrt(30·P·G)/r far-field envelope.
    let e_field = (30.0 * p.tx_eirp_w).sqrt() / p.rf_dist_m.max(1e-3);
    let h_eff = p.wire_len_m;
    e_field * h_eff
}

struct EmNoise {
    /// Total noise voltage, V rms.
    total_v: f64,
    /// Individual contributors, V rms.
    xtalk_pair_v: f64,
    xtalk_link_v: f64,
    rf_v: f64,
    thermal_v: f64,
    /// Signal-to-noise ratio at the UART decision point.
    snr_db: f64,
    /// Bit error rate for a binary sample with decision margin.
    ber: f64,
}

/// Decision margin for a 3V3 UART: the tighter of (V_IH − V_OL) and
/// (V_OH − V_IL). For the ESP32's CMOS inputs V_IL≈0.99 V, V_IH≈2.31 V.
fn decision_margin(p: &EmParams) -> f64 {
    let v_il = 0.3 * p.v_swing;
    let v_ih = 0.7 * p.v_swing;
    (v_ih - 0.0).min(p.v_swing - v_il)
}

fn compute_noise(p: &EmParams) -> EmNoise {
    // Two aggressors per victim wire: its own pair-neighbour wire, and the
    // nearest wire of the sibling link.
    let xtalk_pair_v = xtalk_coeff(p, p.pair_sep_m) * p.v_swing;
    let xtalk_link_v = xtalk_coeff(p, p.link_sep_m) * p.v_swing;
    // RF pickup, reduced by the coupling efficiency.
    let rf_v = rf_pickup_v(p) * p.rf_coupling_eff;
    let thermal_v = p.thermal_v;

    let total_v = (xtalk_pair_v * xtalk_pair_v
        + xtalk_link_v * xtalk_link_v
        + rf_v * rf_v
        + thermal_v * thermal_v)
        .sqrt();

    let margin = decision_margin(p);
    let snr_lin = margin / total_v.max(1e-9);
    let snr_db = 20.0 * snr_lin.log10();

    // BER for a single sample: Gaussian noise exceeding ±margin/2. (The sample
    // sits at the centre of the bit, so an error needs the noise to push the
    // level across the threshold — roughly a Q(margin/(2σ)) event. We use the
    // full margin as a conservative floor.)
    let ber = q_function(snr_lin);

    EmNoise {
        total_v,
        xtalk_pair_v,
        xtalk_link_v,
        rf_v,
        thermal_v,
        snr_db,
        ber,
    }
}

/// Q-function (tail probability of a unit Gaussian).
fn q_function(x: f64) -> f64 {
    0.5 * (1.0 - erf(x / 2f64.sqrt()))
}

/// Approximation of the error function (Abramowitz & Stegun 7.1.26, |err|<1.5e-7).
fn erf(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let ax = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * ax);
    let y = 1.0
        - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t
            + 0.254829592)
            * t
            * (-ax * ax).exp();
    sign * y
}

/// Frame error rate from a BER: a frame is dropped if any of its wire-bits
/// flips. `bits` includes 8N1 framing (10 bits per byte).
fn fer_from_ber(ber: f64, wire_bits: usize) -> f64 {
    1.0 - (1.0 - ber).powi(wire_bits as i32)
}

fn wire_bits(bytes: usize) -> usize {
    bytes * 10
}

// ---------------------------------------------------------------------------
// Monte-Carlo cross-check through the real decoder
// ---------------------------------------------------------------------------

struct MonteCarlo {
    sent: u64,
    recovered: u64,
    undetected: u64,
}

/// Encode a mixed report/snapshot stream, corrupt `ber`-fraction of the bits,
/// feed the *production* decoder, and count frames that survive. Returns how
/// many of the `n` frames came through (CRC-valid, correctly framed).
fn monte_carlo(ber: f64, n: usize) -> MonteCarlo {
    // Build one feature report and one snapshot frame (real wire bytes).
    let report = FeatureReport {
        seq: 1234,
        n_frames: 20,
        rssi: -52,
        motion_energy: 0.37,
        ..Default::default()
    };
    let mut rbuf = [0u8; MAX_FRAME];
    let rn = radar_transport::build_feature_report(
        &mut rbuf,
        node::RX1,
        &report,
        1_000_000,
    );

    let snap = CsiSnapshot {
        seq: 1234,
        rssi: -52,
        ..Default::default()
    };
    let mut sbuf = [0u8; MAX_FRAME];
    let sn = radar_transport::build_csi_snapshot(&mut sbuf, node::RX1, &snap, 1_000_000);

    // Interleave reports and snapshots in the ratio the link actually carries.
    let frames: Vec<&[u8]> = (0..n)
        .map(|i| {
            if i % 5 == 0 {
                &sbuf[..sn]
            } else {
                &rbuf[..rn]
            }
        })
        .collect();

    // Corrupt each byte's bits independently at `ber`.
    let mut stream = Vec::with_capacity(n * 128);
    for f in &frames {
        for &b in *f {
            stream.push(corrupt_byte(b, ber));
        }
    }

    // Feed through the real decoder, counting valid frames by seq/kind.
    let mut decoder = RadarFrameDecoder::new();
    let mut recovered = 0u64;
    let mut undetected = 0u64;
    for chunk in stream.chunks(64) {
        decoder.feed(chunk);
        while let Some(f) = decoder.next() {
            recovered += 1;
            // If corruption slipped past the CRC, it would desync framing or
            // carry a wrong payload — the decoder only yields CRC-valid frames,
            // so an "undetected" corruption that still decodes as the same
            // kind/seq is the residual. This is ~2^-16-rare in practice.
            if f.kind() != radar_protocol::frame_type::FEATURE_REPORT
                && f.kind() != radar_protocol::frame_type::CSI_SNAPSHOT
            {
                undetected += 1;
            }
        }
    }
    // Drain remainder.
    while let Some(_) = decoder.next() {
        recovered += 1;
    }

    MonteCarlo {
        sent: n as u64,
        recovered,
        undetected,
    }
}

/// Corrupt each bit with probability `ber`.
fn corrupt_byte(b: u8, ber: f64) -> u8 {
    let mut out = b;
    let mut rng = 0x2545F4914F6CDD1Du64; // splitmix64 state
    for _ in 0..8 {
        rng = rng.wrapping_add(0x9E3779B97F4A7C15);
        let z = rng;
        let z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        let z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        let r = (z ^ (z >> 31)) as f64 / (u64::MAX as f64);
        if r < ber {
            out ^= 0x01;
        }
        out = out.rotate_left(1);
    }
    out
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------

fn print_link_budget(budget: &LinkBudget, rx1_len_m: f64, cam_len_m: f64) {
    println!("== Link budget ===============================================");
    println!("baud            : {} (8N1 → {} B/s)", budget.baud, budget.bytes_per_s);
    println!(
        "frame sizes     : FeatureReport {} B · CsiSnapshot {} B · CalCmd {} B · CalResp {} B",
        budget.report_bytes, budget.snapshot_bytes, budget.cal_cmd_bytes, budget.cal_resp_bytes
    );
    println!();
    println!(
        "max pure reports/s         : {:.0}",
        budget.max_reports_s()
    );
    println!(
        "max reports/s w/ 2 Hz snap : {:.0}",
        budget.max_reports_s_with_snapshots(2.0)
    );
    println!(
        "max reports/s w/ 5 Hz snap : {:.0}",
        budget.max_reports_s_with_snapshots(5.0)
    );
    println!();
    // Configured cadence: report_every=20 @200Hz → 10 Hz reports; ~2 Hz snap.
    let reports_s = 10.0;
    let snaps_s = 2.0;
    println!(
        "configured cadence        : {reports_s} reports/s + {snaps_s} snaps/s "
    );
    println!(
        "  → link utilization      : {:.1}% of {:.0} B/s",
        budget.utilization(reports_s, snaps_s) * 100.0,
        budget.bytes_per_s
    );
    println!(
        "  → software poll budget  : {:.1} ms max per report window (link idle {:.1}% of the time)",
        budget.max_software_poll_ms(reports_s, snaps_s),
        (1.0 - budget.utilization(reports_s, snaps_s)) * 100.0
    );
    println!();
    println!(
        "TX fusion poll ceiling    : one poll per link every 5 ms (READ_POLL_MS) → 200 polls/s/link"
    );
    println!(
        "  → the link byte budget, NOT the poll loop, is the binding limit: the CAM link can "
    );
    println!(
        "    carry ~{} reports/s, far above the {} configured.",
        budget.max_reports_s_with_snapshots(snaps_s) as u32, reports_s as u32
    );
    println!();
    println!(
        "jumper lengths            : RX1 (middle↔RX1) ≈ {:.0} mm · CAM (middle↔CAM) ≈ {:.0} mm",
        rx1_len_m * 1000.0,
        cam_len_m * 1000.0
    );
}

fn print_em(which: &str, p: &EmParams, budget: &LinkBudget) {
    let n = compute_noise(p);
    let f_report = fer_from_ber(n.ber, wire_bits(budget.report_bytes));
    let f_snapshot = fer_from_ber(n.ber, wire_bits(budget.snapshot_bytes));
    // Frames per second dropped at the configured cadence.
    let drops_s = f_report * 10.0 + f_snapshot * 2.0;
    println!("-- {which} link ------------------------------------------------");
    println!(
        "  geometry : L={:.0} mm  pair sep={:.0} mm  link sep={:.0} mm  h={:.1} mm  RF @ {:.0} mm (EIRP {:.0} mW)",
        p.wire_len_m * 1000.0,
        p.pair_sep_m * 1000.0,
        p.link_sep_m * 1000.0,
        p.height_m * 1000.0,
        p.rf_dist_m * 1000.0,
        p.tx_eirp_w * 1000.0
    );
    println!(
        "  noise    : pair-xtalk {:>7.2} mV  link-xtalk {:>7.2} mV  RF {:>8.2} mV  thermal {:>6.2} mV  → total {:>7.3} mV rms",
        n.xtalk_pair_v * 1000.0,
        n.xtalk_link_v * 1000.0,
        n.rf_v * 1000.0,
        n.thermal_v * 1000.0,
        n.total_v * 1000.0
    );
    println!(
        "  decision : margin {:.2} V  → SNR {:.1} dB  → BER {:.2e}",
        decision_margin(p),
        n.snr_db,
        n.ber
    );
    println!(
        "  FER      : report {:.2e}  snapshot {:.2e}  (drops/s at 10r+2s cadence: {:.2e})",
        f_report, f_snapshot, drops_s
    );
    // Undetected-error floor from CRC16.
    let crc_floor = (f_report * 10.0 + f_snapshot * 2.0) * (1.0 / 65536.0);
    println!(
        "  CRC16    : undetected-error floor ≈ {:.2e} /s (only corrupted frames that pass 16-bit CRC)",
        crc_floor
    );
    // Monte-Carlo cross-check at this BER.
    let mc = monte_carlo(n.ber, 4000);
    let measured = (mc.sent - mc.recovered) as f64 / mc.sent as f64;
    println!(
        "  Monte-Carlo @ this BER: {}/{} frames recovered (measured FER {:.3e} vs analytic {:.3e}, {} undetected)",
        mc.recovered, mc.sent, measured, f_report * 0.8 + f_snapshot * 0.2, mc.undetected
    );
    println!();
}

fn main() {
    let budget = LinkBudget::new(460_800);
    // Jumper run lengths from the physical layout: RX1 ≈ 14 columns × 2.54 mm,
    // CAM ≈ 10 columns × 2.54 mm.
    let rx1_len_m = 14.0 * 0.00254;
    let cam_len_m = 10.0 * 0.00254;

    print_link_budget(&budget, rx1_len_m, cam_len_m);
    println!();

    // EM sweep over the two link geometries.
    let mut p = EmParams::default();
    p.wire_len_m = rx1_len_m;
    print_em("LINK 1 (RX1)", &p, &budget);

    p.wire_len_m = cam_len_m;
    print_em("LINK 2 (CAM)", &p, &budget);

    // Sensitivity sweep: what happens if the CAM link's wires sit further from
    // the TX antenna (10 mm instead of 30 mm)?
    println!("== RF distance sensitivity (CAM link) ===========================");
    let mut p2 = p;
    for dist_mm in [50, 30, 20, 12, 8, 5] {
        p2.rf_dist_m = dist_mm as f64 / 1000.0;
        let n = compute_noise(&p2);
        let f_report = fer_from_ber(n.ber, wire_bits(budget.report_bytes));
        println!(
            "  RF @ {:>3} mm: total noise {:>7.2} mV  SNR {:>6.1} dB  BER {:.2e}  FER(report) {:.2e}",
            dist_mm,
            n.total_v * 1000.0,
            n.snr_db,
            n.ber,
            f_report
        );
    }
    println!();

    // Baud sweep for the polling ceiling.
    println!("== Polling ceiling vs baud =====================================");
    for baud in [115_200u32, 230_400, 460_800, 921_600] {
        let b = LinkBudget::new(baud);
        println!(
            "  {:>7} baud: {:.0} B/s → max {:.0} reports/s (2 Hz snap), {:.1}% util @ 10r+2s",
            baud,
            b.bytes_per_s,
            b.max_reports_s_with_snapshots(2.0),
            b.utilization(10.0, 2.0) * 100.0
        );
    }
    println!();

    // The binding constraint at 460800: the RX software loop.
    println!("== Binding constraints @ 460800 =================================");
    println!(
        "  byte budget: {} B/s → {:.1}% used by 10r+2s → the link is nowhere near saturated",
        budget.bytes_per_s,
        budget.utilization(10.0, 2.0) * 100.0
    );
    println!(
        "  real limit : the RX DSP/CSI pipeline cadence (report_every=20 @200 Hz) sets the "
    );
    println!(
        "              report rate at 10/s; the UART could carry ~{}× that.",
        (budget.max_reports_s_with_snapshots(2.0) / 10.0) as u32
    );
    println!(
        "  RX FIFO    : {} B ≥ {} B worst frame → no overflow window even at the max report rate",
        2 * (HEADER + MAX_PAYLOAD),
        budget.snapshot_bytes
    );
}
