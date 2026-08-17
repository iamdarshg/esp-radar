//! RADAR-TX HTTP + WebSocket dashboard server (`feature = "device"`).
//!
//! Wraps esp-idf-svc's `EspHttpServer`, which for our purposes is the ESP-IDF
//! httpd with the WebSocket upgrade path enabled. Design:
//!
//! * `/` and `/app.js` serve the embedded dashboard files (written separately
//!   in the dashboard task; placeholders here).
//! * `/status` returns the current [`StatusSnapshot`] as compact JSON.
//! * `/ws` is a WebSocket endpoint. On connect the handler grabs an
//!   [`EspHttpWsDetachedSender`] and registers it in the
//!   [`TelemetryBroadcaster`]; the fusion loop then pushes telemetry frames
//!   from its own task. Closed sockets are pruned lazily on each broadcast.
//!
//! Requires `CONFIG_HTTPD_WS_SUPPORT=y` in the firmware's `sdkconfig.defaults`.

use std::sync::{Arc, Mutex};

use esp_idf_svc::http::server::ws::EspHttpWsDetachedSender;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::http::Method;
use esp_idf_svc::io::{EspIOError, Write};
// `esp_idf_svc::ws` re-exports `embedded_svc::ws::FrameType` unconditionally; the
// `http::server::ws` module above is gated on `esp_idf_httpd_ws_support` (i.e.
// `CONFIG_HTTPD_WS_SUPPORT=y` in the firmware sdkconfig).
use esp_idf_svc::sys::EspError;
use esp_idf_svc::ws::FrameType;

use crate::telemetry::{EncodeError, StatusFrame, StatusSnapshot};

/// Embedded dashboard entry point. Replaced by the real dashboard build in the
/// dashboard task.
const INDEX_HTML: &str = include_str!("../static/index.html");
/// Embedded dashboard script.
const APP_JS: &str = include_str!("../static/app.js");

const CT_HTML: &str = "text/html; charset=utf-8";
const CT_JS: &str = "application/javascript; charset=utf-8";
const CT_JSON: &str = "application/json; charset=utf-8";

/// Shared set of live WebSocket dashboard connections.
///
/// `EspHttpWsDetachedSender` is `Send` but not `Sync`; wrapping the vec in a
/// `Mutex` makes the whole thing `Sync` (a `Mutex<T>` is `Sync` iff `T: Send`),
/// so it can be `Arc`-shared between the httpd thread and the fusion task.
#[derive(Clone, Default)]
pub struct TelemetryBroadcaster {
    clients: Arc<Mutex<Vec<EspHttpWsDetachedSender>>>,
}

impl TelemetryBroadcaster {
    fn connect(&self, sender: EspHttpWsDetachedSender) {
        self.clients.lock().unwrap().push(sender);
    }

    /// Push `data` to every live dashboard. Prunes closed sockets and any whose
    /// last send failed. Returns the number of clients after pruning.
    pub fn broadcast(&self, frame_type: FrameType, data: &[u8]) -> usize {
        let mut clients = self.clients.lock().unwrap();
        let mut i = 0;
        while i < clients.len() {
            if clients[i].is_closed() {
                clients.swap_remove(i);
                continue;
            }
            if let Err(e) = clients[i].send(frame_type, data) {
                log::warn!("websocket send failed: {e:?}; dropping client");
                clients.swap_remove(i);
                continue;
            }
            i += 1;
        }
        clients.len()
    }

    /// Push a raw telemetry payload to all dashboards as one binary WS frame.
    /// Convenience for firmware tasks that don't want to name `FrameType`.
    pub fn broadcast_raw(&self, data: &[u8]) -> usize {
        self.broadcast(FrameType::Binary(false), data)
    }

    /// Number of currently-registered dashboard sockets.
    pub fn client_count(&self) -> usize {
        self.clients.lock().unwrap().len()
    }
}

/// The running dashboard server plus its shared state.
pub struct Dashboard {
    _server: EspHttpServer<'static>,
    /// Live status shared with the fusion loop (writer) and `/status` (reader).
    status: Arc<Mutex<StatusSnapshot>>,
    /// WebSocket fan-out to connected dashboards.
    broadcast: TelemetryBroadcaster,
}

impl Dashboard {
    /// Start the HTTP + WebSocket server on port 80.
    pub fn start(status: Arc<Mutex<StatusSnapshot>>) -> Result<Self, EspError> {
        let config = Configuration {
            http_port: 80,
            ctrl_port: 32768,
            max_sessions: 8,
            stack_size: 8192,
            ..Default::default()
        };
        // `EspHttpServer::new` reports `EspIOError` (embedded-io's newtype over
        // `EspError`); unwrap it so the public API stays `EspError`.
        let mut server = EspHttpServer::new(&config).map_err(|e| e.0)?;

        // Embedded dashboard files.
        server.fn_handler("/", Method::Get, |request| -> Result<(), EspIOError> {
            let mut resp = request.into_response(200, Some("OK"), &[("Content-Type", CT_HTML)])?;
            resp.write_all(INDEX_HTML.as_bytes())?;
            Ok(())
        })?;
        server.fn_handler(
            "/app.js",
            Method::Get,
            |request| -> Result<(), EspIOError> {
                let mut resp =
                    request.into_response(200, Some("OK"), &[("Content-Type", CT_JS)])?;
                resp.write_all(APP_JS.as_bytes())?;
                Ok(())
            },
        )?;

        // JSON status snapshot (no WebSocket required).
        let status_http = status.clone();
        server.fn_handler(
            "/status",
            Method::Get,
            move |request| -> Result<(), EspIOError> {
                let snap = status_http.lock().unwrap();
                let body = snap.to_json();
                drop(snap);
                let mut resp =
                    request.into_response(200, Some("OK"), &[("Content-Type", CT_JSON)])?;
                resp.write_all(body.as_bytes())?;
                Ok(())
            },
        )?;

        // WebSocket telemetry: register a detached sender per connection.
        let broadcast = TelemetryBroadcaster::default();
        let ws_broadcast = broadcast.clone();
        server.ws_handler("/ws", None, move |conn| -> Result<(), EspIOError> {
            if conn.is_new() {
                match conn.create_detached_sender() {
                    Ok(sender) => ws_broadcast.connect(sender),
                    Err(e) => log::warn!("ws detached sender failed: {e:?}"),
                }
            }
            // `Closed` connections are pruned lazily by the next broadcast.
            Ok(())
        })?;

        log::info!("dashboard server up on http://192.168.4.1:80 (ws /ws, json /status)");

        Ok(Self {
            _server: server,
            status,
            broadcast,
        })
    }

    /// Push a raw telemetry frame to all dashboards. Returns live client count.
    pub fn broadcast(&self, data: &[u8]) -> usize {
        self.broadcast.broadcast(FrameType::Binary(false), data)
    }

    /// Encode the current status snapshot as a [`StatusFrame`] and push it.
    pub fn broadcast_status(&self) -> Result<usize, EncodeError> {
        let snap = self.status.lock().unwrap();
        let mut buf = [0u8; StatusFrame::LEN];
        let n = snap.frame.encode(&mut buf)?;
        drop(snap);
        Ok(self.broadcast(&buf[..n]))
    }

    /// Number of connected dashboards.
    pub fn client_count(&self) -> usize {
        self.broadcast.client_count()
    }

    /// Clone of the WebSocket fan-out, for pushing telemetry from other tasks
    /// (the fusion loop owns the snapshot; it broadcasts from its own task).
    pub fn broadcaster(&self) -> TelemetryBroadcaster {
        self.broadcast.clone()
    }

    /// Register extra HTTP handlers on the dashboard's server (the firmware's
    /// `/cal` and `/ota` endpoints). Must be called after `start`; the closures
    /// run on the httpd task's own thread.
    pub fn register<F>(&mut self, f: F) -> Result<(), EspError>
    where
        F: FnOnce(&mut EspHttpServer<'static>) -> Result<(), EspError>,
    {
        f(&mut self._server)
    }
}
