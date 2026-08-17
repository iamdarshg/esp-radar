//! ESP-IDF OTA FFI wrapper (`feature = "device"`).
//!
//! Thin, safe-ish wrapper around the `app_update` component. The dashboard
//! upload flow (task 11/13) is: [`OtaWriter::begin`] with the image length,
//! one [`write`](OtaWriter::write) per received chunk, then [`finish`](OtaWriter::finish)
//! and reboot. [`OtaWriter`] is `Drop`-safe — a dropped, unfinished writer
//! aborts the partial update so the slot is never left half-flashed.

use core::ffi::{c_char, c_void, CStr};

use esp_idf_sys as sys;

/// Sentinel passed to `esp_ota_begin` when the image length is unknown.
///
/// This is the IDF macro `OTA_WITH_SEQUENTIAL_WRITES` (`0xfffffffe`): instead
/// of erasing the whole slot up front it erases incrementally as each write
/// lands, so a web upload starts instantly with no long blocking erase. (Note
/// that `0xffffffff` is a *different* macro, `OTA_SIZE_UNKNOWN`, which erases
/// the entire slot first.) On the 32-bit target `usize` is 32 bits, so
/// `usize::MAX - 1 == 0xfffffffe`.
const SEQUENTIAL_WRITES: usize = usize::MAX - 1;

/// Errors from the OTA subsystem, wrapping the raw `esp_err_t` where relevant.
#[derive(Debug)]
pub enum OtaError {
    /// The partition table has no second (inactive) OTA slot, or the running
    /// image is not an OTA-capable layout. Check the partition table.
    NoOtaPartition,
    /// `esp_ota_begin` failed (raw ESP error code).
    Begin(i32),
    /// `esp_ota_write` failed (raw ESP error code).
    Write(i32),
    /// `esp_ota_end` failed — the written image failed validation, or the slot
    /// is full (raw ESP error code).
    End(i32),
    /// `esp_ota_set_boot_partition` failed (raw ESP error code).
    SetBoot(i32),
    /// `esp_ota_abort` failed (raw ESP error code).
    Abort(i32),
}

impl core::fmt::Display for OtaError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            OtaError::NoOtaPartition => write!(f, "no inactive OTA partition available"),
            OtaError::Begin(rc) => write!(f, "esp_ota_begin failed (0x{rc:08x})"),
            OtaError::Write(rc) => write!(f, "esp_ota_write failed (0x{rc:08x})"),
            OtaError::End(rc) => write!(f, "esp_ota_end failed (0x{rc:08x})"),
            OtaError::SetBoot(rc) => write!(f, "esp_ota_set_boot_partition failed (0x{rc:08x})"),
            OtaError::Abort(rc) => write!(f, "esp_ota_abort failed (0x{rc:08x})"),
        }
    }
}

impl std::error::Error for OtaError {}

/// Streams a firmware image into the inactive OTA slot.
pub struct OtaWriter {
    handle: sys::esp_ota_handle_t,
    partition: *const sys::esp_partition_t,
    written: usize,
}

// SAFETY: `partition` points into the (static, never-freed) partition table;
// the IDF keeps the struct alive for the lifetime of the writer.
unsafe impl Send for OtaWriter {}

impl OtaWriter {
    /// Begin an update into the slot that is NOT currently running.
    ///
    /// `expected_size`: the image length in bytes when known (e.g. the
    /// `Content-Length` of the dashboard upload). Pass `0` for unknown — the
    /// slot is validated incrementally and at [`finish`](Self::finish) instead
    /// of being pre-erased.
    pub fn begin(expected_size: usize) -> Result<Self, OtaError> {
        // SAFETY: a null `from` asks IDF to pick the next inactive OTA slot.
        let next = unsafe { sys::esp_ota_get_next_update_partition(core::ptr::null()) };
        if next.is_null() {
            return Err(OtaError::NoOtaPartition);
        }
        let size = if expected_size == 0 {
            SEQUENTIAL_WRITES
        } else {
            expected_size
        };
        let mut handle: sys::esp_ota_handle_t = 0;
        // SAFETY: `handle` is written by IDF before return; `next` is a valid
        // static partition pointer.
        let rc =
            unsafe { sys::esp_ota_begin(next, size, &mut handle as *mut sys::esp_ota_handle_t) };
        if rc != sys::ESP_OK {
            return Err(OtaError::Begin(rc));
        }
        Ok(Self {
            handle,
            partition: next,
            written: 0,
        })
    }

    /// Stream the next chunk of the image. Call repeatedly as the upload
    /// arrives; chunks are written in order to the flash.
    pub fn write(&mut self, chunk: &[u8]) -> Result<(), OtaError> {
        // SAFETY: `handle` is live; `chunk` points into valid memory for
        // `chunk.len()` bytes, which is what `esp_ota_write` consumes.
        let rc = unsafe {
            sys::esp_ota_write(self.handle, chunk.as_ptr() as *const c_void, chunk.len())
        };
        if rc != sys::ESP_OK {
            return Err(OtaError::Write(rc));
        }
        self.written += chunk.len();
        Ok(())
    }

    /// Bytes written so far (for upload progress on the dashboard).
    pub fn written(&self) -> usize {
        self.written
    }

    /// Target partition label, e.g. `"ota_0"` (for logs/confirmation).
    pub fn target_label(&self) -> Option<&'static str> {
        // SAFETY: `partition` is a valid static pointer; `label` is a
        // null-terminated inline C string that lives for the program's lifetime.
        let label = unsafe { (*self.partition).label.as_ptr() };
        unsafe { CStr::from_ptr(label as *const c_char) }
            .to_str()
            .ok()
    }

    /// Validate the written image and switch the boot partition to it.
    ///
    /// The caller must reboot immediately after `Ok` to run the new image;
    /// until then the old firmware keeps executing from the old slot.
    pub fn finish(self) -> Result<(), OtaError> {
        // Copy the values out first: after `mem::forget(self)` the moved-out
        // `self` is dead, so nothing may read from it afterwards.
        let handle = self.handle;
        let partition = self.partition;
        // `esp_ota_end` invalidates the handle; forget `self` so Drop never
        // tries to abort an already-finished OTA.
        core::mem::forget(self);

        // SAFETY: `handle` is live (we own it); `esp_ota_end` frees it.
        let rc = unsafe { sys::esp_ota_end(handle) };
        if rc != sys::ESP_OK {
            return Err(OtaError::End(rc));
        }
        // SAFETY: `partition` is the slot we just validated.
        let rc = unsafe { sys::esp_ota_set_boot_partition(partition) };
        if rc != sys::ESP_OK {
            return Err(OtaError::SetBoot(rc));
        }
        Ok(())
    }

    /// Abort the update explicitly, freeing the handle without touching the
    /// running image or the boot partition.
    pub fn abort(self) -> Result<(), OtaError> {
        let handle = self.handle;
        core::mem::forget(self);
        // SAFETY: `handle` is live and owned by us.
        let rc = unsafe { sys::esp_ota_abort(handle) };
        if rc != sys::ESP_OK {
            Err(OtaError::Abort(rc))
        } else {
            Ok(())
        }
    }
}

impl Drop for OtaWriter {
    fn drop(&mut self) {
        // Safety net for the error path: a dropped-but-unfinished writer aborts
        // the partial update so the inactive slot is free for the next attempt.
        // (`finish`/`abort` mem::forget self first, so this never double-aborts.)
        // SAFETY: `handle` is live unless we were forgotten.
        unsafe { sys::esp_ota_abort(self.handle) };
    }
}

/// Label of the currently running application partition (for confirmation
/// before switching boot).
pub fn running_partition_label() -> Option<&'static str> {
    partition_label_of(unsafe { sys::esp_ota_get_running_partition() })
}

/// Label of the last partition that failed to boot (rollback diagnostic), or
/// `None` if every image so far has been healthy.
pub fn last_invalid_partition_label() -> Option<&'static str> {
    partition_label_of(unsafe { sys::esp_ota_get_last_invalid_partition() })
}

/// Mark the currently running image as valid, cancelling any pending rollback.
///
/// Call once early at boot (before starting the network/dashboard). If the
/// previous boot OTA-updated and then crashed, the bootloader would otherwise
/// roll back to the old image the next time; this commits the new one.
pub fn mark_app_valid() -> Result<(), i32> {
    let rc = unsafe { sys::esp_ota_mark_app_valid_cancel_rollback() };
    if rc == sys::ESP_OK {
        Ok(())
    } else {
        Err(rc)
    }
}

/// Read a partition's inline `label[17]` C string, or `None` for a null pointer.
fn partition_label_of(p: *const sys::esp_partition_t) -> Option<&'static str> {
    if p.is_null() {
        return None;
    }
    // SAFETY: `p` points into the static partition table; `label` is a
    // null-terminated inline C string with static lifetime.
    unsafe { CStr::from_ptr((*p).label.as_ptr() as *const c_char) }
        .to_str()
        .ok()
}
