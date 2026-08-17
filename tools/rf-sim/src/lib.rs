//! rf-sim library targets: the RF scenario model, simdata blob generator, and
//! error analyzer. The `main.rs` binary wires these into the `gen` / `analyze`
//! CLI; integration tests under `tests/` exercise the generator against the
//! real firmware DSP (`radar_dsp::transform::decode_channel`) without QEMU.

pub mod analyze;
pub mod rng;
pub mod scenario;
