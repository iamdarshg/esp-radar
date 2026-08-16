//! Partition-aware firmware update via web upload (spec §13 component list).
//!
//! RADAR-TX can be reflashed without a serial cable: the dashboard uploads a
//! firmware image, this crate streams it into the *inactive* OTA partition,
//! validates it (`esp_ota_end`), and switches the boot partition. The running
//! image is never touched until the reboot that applies the update — if the
//! upload fails midway, the old firmware keeps running and the target slot is
//! left in place for the next attempt.
//!
//! Requires a two-OTA-slot partition table (the ESP-IDF default
//! factory/ota_0/ota_1 layout). Every function here is device-only; the crate
//! is an empty no-op when built on the host.

#[cfg(feature = "device")]
pub mod ota;
