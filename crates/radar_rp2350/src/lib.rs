//! Optional wired RP2350 DSP/compute coprocessor link (spec §12).
//!
//! The RP2350 is a NON-Wi-Fi compute node with **no RF responsibility**, wired
//! to RADAR-TX over UART2 (ESP32 GPIO17 = TX → RP2350 RX, ESP32 GPIO16 = RX ←
//! RP2350 TX, 3V3 logic). The wire protocol itself lives in
//! [`radar_protocol::cp`]; this crate is the *transport* that carries it:
//!
//! * [`framer`] — turns the UART byte stream into validated
//!   [`radar_protocol::cp`] frames. Pure and host-testable: handles resync on
//!   garbage, partial frames, and CRC failures.
//! * [`session`] — pure link-session helpers (version negotiation, the
//!   coprocessor-local sequence counter).
//! * [`link`] (`feature = "device"`) — the UART2 driver wrapper: a
//!   HELLO/HELLO_ACK presence probe, best-effort `push` of feature/spectrogram
//!   payloads, and a STATUS poll for diagnostics.
//!
//! **RADAR-TX must operate with no coprocessor present** (spec §12). Every link
//! operation is therefore best-effort and self-healing: a missing or silent
//! coprocessor only costs a log line — it never blocks or fails the radar.

pub mod framer;
pub mod session;

#[cfg(feature = "device")]
pub mod link;

pub use framer::{Frame, FrameDecoder, MAX_FRAME};
pub use session::{Seq, compatible};
