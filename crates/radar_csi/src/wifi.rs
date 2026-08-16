//! Thin ESP-IDF binding around the WiFi CSI API (`esp_wifi.h`).
//!
//! Registered callback: [`csi_cb`] — kept deliberately short (memcpy only,
//! see [`crate::CsiRing`]).
//!
//! ## `rx_ctrl` decoding
//!
//! `wifi_csi_info_t` embeds a [`wifi_pkt_rx_ctrl_t`] bitfield struct as its
//! first member. bindgen models bitfields as raw `u32` words and may or may
//! not emit named accessors depending on version — so we decode the ESP32
//! bit layout directly from the bytes, verified against
//! `esp_wifi_types_native.h` (v5.4):
//!
//! ```text
//! word0 bits  0..7   rssi (signed)         | 8..12 rate | 13 rsv | 14..15 sig_mode | 16..31 rsv
//! word1 bits  0..6   mcs                   | 7 cwb      | 8..23 rsv | 24 smooth | 25 not_sound | 26 rsv | 27 agg | 28..29 stbc | 30 fec | 31 sgi
//! word2 bits  0..7   noise_floor (signed)  | 8..15 ampdu_cnt | 16..19 channel | 20..23 secondary | 24..31 rsv
//! word3 bits  0..31  timestamp
//! word5 bits  31     ant
//! word6 bits  0..11  sig_len               | 12..23 rsv | 24..31 rx_state
//! ```
//!
//! This targets ESP32 classic (all three radar boards are ESP32). On another
//! chip the `noise_floor` bit position differs — assert that here.

use crate::{CsiInfo, CsiRing, MAX_CSI_LEN};
use core::ffi::c_void;
use esp_idf_sys as sys;

/// CSI acquisition configuration (`wifi_csi_config_t`).
#[derive(Clone, Copy, Debug)]
pub struct CsiConfig {
    pub lltf_en: bool,
    pub htltf_en: bool,
    pub stbc_htltf2_en: bool,
    pub ltf_merge_en: bool,
    pub channel_filter_en: bool,
    pub manu_scale: bool,
    /// Manual left-shift of CSI data scale, 0..=15 (only used when
    /// `manu_scale` is set).
    pub shift: u8,
    pub dump_ack_en: bool,
}

impl Default for CsiConfig {
    fn default() -> Self {
        // Recommended for radar use: HT-LTF merged (full 56-subcarrier
        // estimate), channel filter on (smooth adjacent bins), automatic
        // scaling, no ACK dumping.
        Self {
            lltf_en: false,
            htltf_en: true,
            stbc_htltf2_en: true,
            ltf_merge_en: true,
            channel_filter_en: true,
            manu_scale: false,
            shift: 0,
            dump_ack_en: false,
        }
    }
}

/// Error from the WiFi CSI API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CsiError {
    /// A low-level `esp_err_t` was not `ESP_OK`.
    EspError(i32),
}

impl CsiError {
    pub fn code(&self) -> i32 {
        match self {
            CsiError::EspError(c) => *c,
        }
    }
}

impl core::fmt::Display for CsiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CSI error esp_err_t={}", self.code())
    }
}

fn check(err: sys::esp_err_t) -> Result<(), CsiError> {
    if err == sys::ESP_OK {
        Ok(())
    } else {
        Err(CsiError::EspError(err))
    }
}

/// The Wi-Fi task callback. Kept extremely short: decode the fixed metadata
/// from the info struct, `memcpy` the CSI bytes into the ring, done.
unsafe extern "C" fn csi_cb(ctx: *mut c_void, data: *mut sys::wifi_csi_info_t) {
    if data.is_null() {
        return;
    }
    let info = &*data;
    if info.buf.is_null() || info.len == 0 {
        return;
    }
    let ring = &*(ctx as *const CsiRing);
    let len = (info.len as usize).min(MAX_CSI_LEN);
    let bytes = core::slice::from_raw_parts(info.buf as *const i8, len);
    let meta = decode_info(info);
    ring.push(meta, bytes);
}

/// Decode the non-bitfield fields of `wifi_csi_info_t` plus the ESP32
/// `rx_ctrl` bitfields (see module docs).
unsafe fn decode_info(info: &sys::wifi_csi_info_t) -> CsiInfo {
    debug_assert_eq!(
        core::mem::size_of::<sys::wifi_pkt_rx_ctrl_t>(),
        28,
        "radar_csi rx_ctrl decoder assumes ESP32 (7 × u32). \
         Refactor for the target chip's bit layout before moving off ESP32."
    );

    // rx_ctrl is the first member; read its words directly.
    let base = info as *const sys::wifi_csi_info_t as *const u8;
    let w = |byte: usize| -> u32 { core::ptr::read_unaligned(base.add(byte) as *const u32) };
    let w0 = w(0);
    let w1 = w(4);
    let w2 = w(8);
    let w3 = w(12);
    let w6 = w(24);

    let mut mac = [0u8; 6];
    mac.copy_from_slice(&info.mac);

    CsiInfo {
        rssi: (w0 & 0xFF) as u8 as i8 as i16,
        noise_floor: (w2 & 0xFF) as u8 as i8 as i16,
        channel: ((w2 >> 16) & 0xF) as u8 as i8,
        sig_mode: ((w0 >> 14) & 0x3) as u8,
        mcs: (w1 & 0x7F) as u8,
        cwb: ((w1 >> 7) & 0x1) as u8,
        timestamp_us: w3,
        sig_len: (w6 & 0xFFF) as u16,
        first_word_invalid: info.first_word_invalid,
        mac,
        rx_seq: info.rx_seq,
    }
}

/// Configure, register the callback and enable CSI.
///
/// `ring` must outlive the CSI session (leak it or make it `'static`); the
/// callback dereferences this pointer on every received packet.
pub fn start_csi(ring: &'static CsiRing, config: &CsiConfig) -> Result<(), CsiError> {
    let idf = sys::wifi_csi_config_t {
        lltf_en: config.lltf_en,
        htltf_en: config.htltf_en,
        stbc_htltf2_en: config.stbc_htltf2_en,
        ltf_merge_en: config.ltf_merge_en,
        channel_filter_en: config.channel_filter_en,
        manu_scale: config.manu_scale,
        shift: config.shift,
        dump_ack_en: config.dump_ack_en,
    };
    // The esp-idf-sys 0.37 FFI marks these WiFi API entry points `unsafe fn`:
    // they poke shared WiFi state, so wrap each call in an explicit block.
    unsafe {
        check(sys::esp_wifi_set_csi_config(&idf))?;
        check(sys::esp_wifi_set_csi_rx_cb(
            Some(csi_cb),
            ring as *const CsiRing as *mut c_void,
        ))?;
        check(sys::esp_wifi_set_csi(true))
    }
}

/// Disable CSI (callback stays registered).
pub fn stop_csi() -> Result<(), CsiError> {
    // SAFETY: same reasoning as `start_csi`.
    unsafe { check(sys::esp_wifi_set_csi(false)) }
}

/// Restore the default CSI config (bindgen maps C `bool` to Rust `bool` and
/// `uint8_t shift` to `u8` — same as [`CsiConfig`]).
#[allow(dead_code)]
pub fn get_csi_config() -> Result<CsiConfig, CsiError> {
    let mut idf = core::mem::MaybeUninit::<sys::wifi_csi_config_t>::uninit();
    // SAFETY: the buffer is a live `MaybeUninit` of the right type; the
    // `assume_init` below runs only after the ESP-IDF call has filled it.
    unsafe {
        check(sys::esp_wifi_get_csi_config(idf.as_mut_ptr()))?;
    }
    let idf = unsafe { idf.assume_init() };
    Ok(CsiConfig {
        lltf_en: idf.lltf_en,
        htltf_en: idf.htltf_en,
        stbc_htltf2_en: idf.stbc_htltf2_en,
        ltf_merge_en: idf.ltf_merge_en,
        channel_filter_en: idf.channel_filter_en,
        manu_scale: idf.manu_scale,
        shift: idf.shift,
        dump_ack_en: idf.dump_ack_en,
    })
}
