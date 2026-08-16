//! ESP32 NVS binding for the radar (`feature = "device"`).
//!
//! ESP-IDF's modern Rust stack (esp-idf-svc 0.52.1) exposes a *partition*
//! abstraction but no raw handle store we can reuse for our own namespace, so
//! — exactly like `radar_transport::udp` — we call the C NVS API directly
//! through the generated `esp-idf-sys` bindings (`nvs_flash_init`, `nvs_open`,
//! `nvs_get_blob`, `nvs_set_blob`, ...).
//!
//! One namespace `radar` holds every artifact. NVS keys are limited to
//! 15 characters; ours are all shorter.

use core::ffi::{CStr, c_void};
use esp_idf_sys as sys;
use radar_calibration::{BaselineStats, ClassThresholds, TxPowerModel};

use crate::{RadarConfig, RxLink};

/// NVS namespace holding all radar settings.
const NAMESPACE: &CStr = c"radar";
/// Whole-system config blob.
const KEY_CONFIG: &CStr = c"config";
/// Per-link empty-room baselines (CAL 1 output).
const KEY_BASELINE1: &CStr = c"baseline1";
const KEY_BASELINE2: &CStr = c"baseline2";
/// Occupancy classifier thresholds (CAL 4 output or defaults).
const KEY_THRESHOLDS: &CStr = c"thresh";
/// TX power↔RSSI model (CAL 2 output, used for auto-commissioning, spec §5).
const KEY_POWER_MODEL: &CStr = c"powmodel";

/// Errors from the raw NVS API, mapped to a small typed set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NvsError {
    /// `nvs_flash_init` failed (partition absent, or corrupt and not reinit).
    Init(i32),
    /// `nvs_open` failed.
    Open(i32),
    /// The key does not exist (or the namespace is empty).
    NotFound,
    /// A blob was stored with a different length than this crate expects
    /// (e.g. a baseline from an older schema version).
    SizeMismatch { expected: usize, actual: usize },
    /// Any other `esp_err_t` from read/write/commit.
    Io(i32),
}

impl core::fmt::Display for NvsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            NvsError::Init(e) => write!(f, "nvs_flash_init failed ({e:#x})"),
            NvsError::Open(e) => write!(f, "nvs_open failed ({e:#x})"),
            NvsError::NotFound => write!(f, "NVS key not found"),
            NvsError::SizeMismatch { expected, actual } => {
                write!(f, "NVS blob size {actual} != expected {expected}")
            }
            NvsError::Io(e) => write!(f, "NVS io failed ({e:#x})"),
        }
    }
}

/// An open handle to the `radar` NVS namespace.
pub struct Nvs {
    handle: sys::nvs_handle_t,
}

impl Nvs {
    /// Initialize the default NVS partition and open the `radar` namespace.
    ///
    /// `reinit = true` recovers from a full/corrupt partition by erasing it
    /// and starting fresh (mirrors esp-idf-svc's `EspNvsPartition::take`).
    pub fn take(reinit: bool) -> Result<Self, NvsError> {
        let rc = unsafe { sys::nvs_flash_init() };
        if rc != sys::ESP_OK {
            let recoverable =
                rc == sys::ESP_ERR_NVS_NO_FREE_PAGES || rc == sys::ESP_ERR_NVS_NEW_VERSION_FOUND;
            if reinit && recoverable {
                let e = unsafe { sys::nvs_flash_erase() };
                if e != sys::ESP_OK {
                    return Err(NvsError::Init(e));
                }
                let rc = unsafe { sys::nvs_flash_init() };
                if rc != sys::ESP_OK {
                    return Err(NvsError::Init(rc));
                }
            } else {
                return Err(NvsError::Init(rc));
            }
        }

        let mut handle: sys::nvs_handle_t = 0;
        let rc = unsafe {
            sys::nvs_open(
                NAMESPACE.as_ptr(),
                sys::nvs_open_mode_t_NVS_READWRITE,
                &mut handle as *mut _,
            )
        };
        if rc != sys::ESP_OK {
            return Err(NvsError::Open(rc));
        }
        Ok(Self { handle })
    }

    // ---- whole-config ----

    /// Load the system config. `Err(NvsError::NotFound)` if unset (fresh flash).
    pub fn load_config(&self) -> Result<RadarConfig, NvsError> {
        let mut buf = [0u8; RadarConfig::SERIALIZED_LEN];
        self.get_blob_const(KEY_CONFIG, &mut buf)?;
        Ok(RadarConfig::from_bytes(&buf))
    }

    pub fn store_config(&self, cfg: &RadarConfig) -> Result<(), NvsError> {
        self.set_blob(KEY_CONFIG, &cfg.to_bytes())
    }

    // ---- calibration artifacts ----

    pub fn load_baseline(&self, link: RxLink) -> Result<BaselineStats, NvsError> {
        let mut buf = [0u8; BaselineStats::SERIALIZED_LEN];
        self.get_blob_const(link_key(link), &mut buf)?;
        Ok(BaselineStats::from_bytes(&buf))
    }

    pub fn store_baseline(&self, link: RxLink, b: &BaselineStats) -> Result<(), NvsError> {
        self.set_blob(link_key(link), &b.to_bytes())
    }

    pub fn load_thresholds(&self) -> Result<ClassThresholds, NvsError> {
        let mut buf = [0u8; ClassThresholds::SERIALIZED_LEN];
        self.get_blob_const(KEY_THRESHOLDS, &mut buf)?;
        Ok(ClassThresholds::from_bytes(&buf))
    }

    pub fn store_thresholds(&self, t: &ClassThresholds) -> Result<(), NvsError> {
        self.set_blob(KEY_THRESHOLDS, &t.to_bytes())
    }

    pub fn load_power_model(&self) -> Result<TxPowerModel, NvsError> {
        let mut buf = [0u8; TxPowerModel::SERIALIZED_LEN];
        self.get_blob_const(KEY_POWER_MODEL, &mut buf)?;
        Ok(TxPowerModel::from_bytes(&buf))
    }

    pub fn store_power_model(&self, m: &TxPowerModel) -> Result<(), NvsError> {
        self.set_blob(KEY_POWER_MODEL, &m.to_bytes())
    }

    // ---- lifecycle ----

    /// Erase every radar key (used by calibration reset). Returns `Ok` even if
    /// some keys were already absent.
    pub fn clear_all(&self) -> Result<(), NvsError> {
        for key in [
            KEY_CONFIG,
            KEY_BASELINE1,
            KEY_BASELINE2,
            KEY_THRESHOLDS,
            KEY_POWER_MODEL,
        ] {
            let rc = unsafe { sys::nvs_erase_key(self.handle, key.as_ptr()) };
            if rc != sys::ESP_OK && rc != sys::ESP_ERR_NVS_NOT_FOUND {
                return Err(NvsError::Io(rc));
            }
        }
        let rc = unsafe { sys::nvs_commit(self.handle) };
        if rc != sys::ESP_OK {
            return Err(NvsError::Io(rc));
        }
        Ok(())
    }

    // ---- primitives ----

    /// Read a fixed-size blob. Two-phase: query the stored length with a null
    /// out-pointer first (NVS requires this to learn the true size), then read.
    fn get_blob_const<const N: usize>(&self, key: &CStr, out: &mut [u8; N]) -> Result<(), NvsError> {
        let mut len: usize = 0;
        let rc = unsafe {
            sys::nvs_get_blob(self.handle, key.as_ptr(), core::ptr::null_mut(), &mut len as *mut _)
        };
        if rc == sys::ESP_ERR_NVS_NOT_FOUND {
            return Err(NvsError::NotFound);
        }
        if rc != sys::ESP_OK {
            return Err(NvsError::Io(rc));
        }
        if len != N {
            return Err(NvsError::SizeMismatch { expected: N, actual: len });
        }
        let rc = unsafe {
            sys::nvs_get_blob(
                self.handle,
                key.as_ptr(),
                out.as_mut_ptr() as *mut c_void,
                &mut len as *mut _,
            )
        };
        if rc != sys::ESP_OK {
            return Err(NvsError::Io(rc));
        }
        Ok(())
    }

    /// Write a blob. NVS requires an erased entry before a rewrite (a set on
    /// top of a differently-sized value fails), so we erase first — a missing
    /// key is fine (`NOT_FOUND` from erase is not an error here).
    fn set_blob(&self, key: &CStr, buf: &[u8]) -> Result<(), NvsError> {
        let rc = unsafe { sys::nvs_erase_key(self.handle, key.as_ptr()) };
        if rc != sys::ESP_OK && rc != sys::ESP_ERR_NVS_NOT_FOUND {
            return Err(NvsError::Io(rc));
        }
        let rc = unsafe {
            sys::nvs_set_blob(self.handle, key.as_ptr(), buf.as_ptr() as *const c_void, buf.len())
        };
        if rc != sys::ESP_OK {
            return Err(NvsError::Io(rc));
        }
        let rc = unsafe { sys::nvs_commit(self.handle) };
        if rc != sys::ESP_OK {
            return Err(NvsError::Io(rc));
        }
        Ok(())
    }
}

impl Drop for Nvs {
    fn drop(&mut self) {
        unsafe {
            sys::nvs_close(self.handle);
        }
    }
}

/// NVS key for a per-link baseline (15-char limit respected).
fn link_key(link: RxLink) -> &'static CStr {
    match link {
        RxLink::Rx1 => KEY_BASELINE1,
        RxLink::Rx2 => KEY_BASELINE2,
    }
}
