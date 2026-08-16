//! HTTP endpoints beyond the dashboard itself: `/cal` (calibration control,
//! §17) and `/ota` (firmware upload via the web UI). Registered on the
//! dashboard's own server through `radar_web::server::Dashboard::register`.

use std::sync::mpsc;
use std::time::Duration;

use esp_idf_svc::http::server::{EspHttpConnection, Request};
use esp_idf_svc::http::Method;
use esp_idf_svc::io::{EspIOError, Write};
use esp_idf_svc::sys::EspError;

use radar_ota::ota::OtaWriter;
use radar_web::server::Dashboard;

use crate::calibrate::CalCommand;

/// Sanity gate for the uploaded image size (the OTA slot is 1 MiB; a build
/// rarely exceeds a few hundred KiB). Guards against a corrupt header turning
/// into a giant write.
const MAX_IMAGE_BYTES: usize = 2 * 1024 * 1024;

/// Register `/cal` and `/ota` on the dashboard server.
pub fn register(
    dashboard: &mut Dashboard,
    cal_tx: mpsc::Sender<CalCommand>,
) -> Result<(), EspError> {
    dashboard.register(move |server| {
        let ct = cal_tx.clone();
        server.fn_handler("/cal", Method::Get, move |request| handle_cal(request, &ct))?;
        server.fn_handler("/ota", Method::Post, |request| handle_ota(request))?;
        Ok(())
    })
}

// -- /cal ---------------------------------------------------------------------

fn handle_cal(
    request: Request<&mut EspHttpConnection<'_>>,
    cal_tx: &mpsc::Sender<CalCommand>,
) -> Result<(), EspIOError> {
    let uri = request.uri().to_string();
    let mut resp = match parse_cal_uri(&uri) {
        Some(cmd) => match cal_tx.send(cmd) {
            Ok(()) => {
                log::info!("cal endpoint: queued {cmd:?}");
                let body = format!("queued: {cmd:?}\n");
                let mut r = request.into_status_response(200)?;
                r.write_all(body.as_bytes())?;
                r
            }
            Err(e) => {
                log::error!("cal command channel error: {e}");
                let mut r = request.into_status_response(503)?;
                r.write_all(b"controller unavailable\n")?;
                r
            }
        },
        None => {
            let mut r = request.into_status_response(400)?;
            r.write_all(b"usage: /cal?stage=1..5 | /cal?auto=1 | /cal?abort=1\n")?;
            r
        }
    };
    let _ = resp.flush();
    Ok(())
}

/// Parse the query string of a `/cal` URI into a [`CalCommand`].
fn parse_cal_uri(uri: &str) -> Option<CalCommand> {
    let query = uri.split('?').nth(1)?;
    for pair in query.split('&') {
        let mut kv = pair.splitn(2, '=');
        let key = kv.next()?;
        let val = kv.next().unwrap_or("1");
        match key {
            "stage" => return val.parse::<u8>().ok().map(CalCommand::StartStage),
            "auto" if val != "0" => return Some(CalCommand::AutoCommission),
            "abort" if val != "0" => return Some(CalCommand::Abort),
            _ => {}
        }
    }
    None
}

// -- /ota ---------------------------------------------------------------------

fn handle_ota(mut request: Request<&mut EspHttpConnection<'_>>) -> Result<(), EspIOError> {
    let Some(len_str) = request.header("Content-Length") else {
        return ota_error(request, "missing Content-Length header");
    };
    let Ok(expected) = len_str.trim().parse::<usize>() else {
        return ota_error(request, "unparseable Content-Length");
    };
    if !(16..=MAX_IMAGE_BYTES).contains(&expected) {
        return ota_error(request, &format!("implausible image size {expected}"));
    }

    let mut writer = match OtaWriter::begin(expected) {
        Ok(w) => w,
        Err(e) => return ota_error(request, &format!("ota begin: {e}")),
    };

    // Stream the body straight into the inactive OTA slot.
    let mut buf = [0u8; 4096];
    let mut total = 0usize;
    loop {
        match request.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                if let Err(e) = writer.write(&buf[..n]) {
                    let _ = writer.abort();
                    return ota_error(request, &format!("ota write at {total}: {e}"));
                }
                total += n;
            }
            Err(e) => {
                let _ = writer.abort();
                return ota_error(request, &format!("ota read: {e:?}"));
            }
        }
    }
    if total != expected {
        let _ = writer.abort();
        return ota_error(request, &format!("short upload: {total} of {expected} bytes"));
    }

    let label = writer.target_label().unwrap_or("unknown");
    match writer.finish() {
        Ok(()) => {
            log::info!("OTA complete: {total} bytes into {label}; rebooting");
            let mut resp = request.into_status_response(200)?;
            let _ = resp.write_all(format!("OK {label} {total} bytes; rebooting\n").as_bytes());
            drop(resp);
            // Give the httpd a moment to flush, then apply the new image.
            std::thread::sleep(Duration::from_millis(300));
            // `esp_restart` never returns; the httpd drops when this task ends.
            unsafe { esp_idf_sys::esp_restart() };
        }
        Err(e) => ota_error(request, &format!("ota finish: {e}")),
    }
}

fn ota_error(
    request: Request<&mut EspHttpConnection<'_>>,
    msg: &str,
) -> Result<(), EspIOError> {
    log::error!("OTA error: {msg}");
    let mut resp = request.into_status_response(500)?;
    resp.write_all(msg.as_bytes())?;
    Ok(())
}
