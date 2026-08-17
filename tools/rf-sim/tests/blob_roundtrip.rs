//! End-to-end blob format validation: generate a scenario, write the simdata
//! blob exactly as the QEMU harness will flash it, then decode a frame back
//! through the REAL firmware DSP (`radar_dsp::transform::decode_channel`) and
//! check the recovered per-subcarrier phase matches the generator's ground
//! truth within the i8-quantization floor.
//!
//! This proves the byte layout the generator emits is the one the firmware
//! reads, without needing QEMU.

use radar_dsp::transform::decode_channel;
use rf_sim::scenario::{generate, PropPath, Scenario, Trajectory, N_SUBC, SIM_FRAME_LEN, SIM_HEADER_LEN, SIM_MAGIC, SIM_VERSION};

fn lo() -> Scenario {
    Scenario {
        version: 1,
        name: "roundtrip".into(),
        rate_hz: 200,
        n_frames: 300,
        fc_hz: 2.437e9,
        sub_spacing_hz: 312_500.0,
        // High SNR so the recovery is quantization-limited, not noise-limited.
        amp: 100.0,
        snr_db: 60.0,
        cfo_hz: 50.0,
        rssi_dbm: -52,
        noise_floor_dbm: -96,
        channel: 6,
        bssid: [36, 15, 40, 1, 2, 3],
        trajectory: Trajectory::ConstVel { v_m_s: -0.35 },
        paths: vec![PropPath {
            amp: 1.0,
            delay_m: 2.0,
            v_m_s: -0.35,
        }],
        target_path: 0,
        seed: 1234,
    }
}

fn wrap(x: f64) -> f64 {
    x - std::f64::consts::TAU * (x / std::f64::consts::TAU).round()
}

#[test]
fn header_matches_sim_format() {
    let sc = lo();
    let path = std::env::temp_dir().join("rf-sim-roundtrip.bin");
    generate(&sc, &path).unwrap();
    let bytes = std::fs::read(&path).unwrap();

    // Header layout as firmware/radar_rx/src/sim.rs reads it.
    let magic = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
    assert_eq!(magic, SIM_MAGIC);
    assert_eq!(bytes[4], SIM_VERSION);
    assert_eq!(bytes[5], 6, "channel");
    let rate = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
    assert_eq!(rate, 200);
    let n = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
    assert_eq!(n, 300);
    let rssi = i16::from_le_bytes(bytes[16..18].try_into().unwrap());
    assert_eq!(rssi, -52);
    assert_eq!(&bytes[20..26], &sc.bssid);
    let flen = u16::from_le_bytes(bytes[28..30].try_into().unwrap());
    assert_eq!(flen as usize, SIM_FRAME_LEN);
    // Total length = header + n_frames × frame_len.
    assert_eq!(bytes.len(), SIM_HEADER_LEN + 300 * SIM_FRAME_LEN);
}

#[test]
fn decoded_phase_recovers_ground_truth() {
    let sc = lo();
    let path = std::env::temp_dir().join("rf-sim-roundtrip.bin");
    generate(&sc, &path).unwrap();
    let bytes = std::fs::read(&path).unwrap();

    let frame_n = 150u64;
    let off = SIM_HEADER_LEN + frame_n as usize * SIM_FRAME_LEN;
    let raw = &bytes[off..off + SIM_FRAME_LEN];
    // The firmware reads the blob as i8.
    let buf: &[i8] = unsafe { std::slice::from_raw_parts(raw.as_ptr() as *const i8, raw.len()) };

    let ch = decode_channel(buf, false, sc.rssi_dbm, sc.noise_floor_dbm);
    assert!(ch.valid, "decode_channel rejected the frame");

    let mut max_err = 0.0f64;
    for k in 0..N_SUBC {
        let gt = sc.phase_clean(frame_n, k);
        let meas = ch.raw_phase[k] as f64;
        let e = wrap(meas - gt).abs();
        max_err = max_err.max(e);
        // At SNR 60 the only significant error is the i8 quantization
        // (σ ≈ 0.29/A ≈ 3 mrad at A=100); allow 10σ + mrad telemetry rounding.
        assert!(
            e < 0.05,
            "k={k} frame {frame_n}: measured {meas:.4} vs GT {gt:.4} (err {e:.4})"
        );
    }
    assert!(max_err < 0.05, "max per-subcarrier phase error {max_err:.4} rad");
}
