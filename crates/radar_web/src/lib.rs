//! Live dashboard server for the radar head (spec §6, §10).
//!
//! RADAR-TX hosts the entire dashboard locally on the "ESP32-RADAR" AP: a phone
//! or tablet connects directly to the ESP32 with no laptop required. The web
//! surface is split into two transport channels:
//!
//! * **WebSocket** (`/ws`) — continuous, compact binary telemetry
//!   ([`telemetry`] frame kinds: status, CSI waterfalls, motion spectrograms).
//! * **HTTP** — the embedded dashboard files (`/`, `/app.js`) plus a plain JSON
//!   status snapshot (`/status`) for non-WS clients and quick diagnostics.
//!
//! This crate is pure for the frame encoders ([`telemetry`], host-testable) and
//! device-only for the server itself (`#[cfg(feature = "device")] pub mod
//! server`). The device module requires `CONFIG_HTTPD_WS_SUPPORT=y` in the
//! firmware's `sdkconfig.defaults` (enables esp-idf-svc's WebSocket server,
//! cfg-gated at esp-idf-sys build time).

pub mod telemetry;

#[cfg(feature = "device")]
pub mod server;

pub use telemetry::*;
