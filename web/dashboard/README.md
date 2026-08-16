# Dashboard (host-side copy)

This directory is the **host-side copy** of the radar dashboard. It is *not*
the source of truth for what runs on the device.

## Canonical source

The dashboard that RADAR-TX actually serves lives in:

```
crates/radar_web/static/
  index.html     single-page dashboard layout + styles
  app.js         WebSocket binary telemetry decoder + render loop
```

Those two files are compiled into the firmware via `include_str!` in
`crates/radar_web/src/server.rs`, so what you see at `http://192.168.4.1`
is exactly the content of `crates/radar_web/static/`. **Any change made here
in `web/dashboard/` does not affect the device until it is copied back there
and the firmware rebuilt.**

## Why this copy exists

The spec calls for a host-side copy of the dashboard so you can open the UI in
a browser on the host machine while developing (spec §13). It is documentation
and a development convenience, not the deployed artifact.

## How to iterate locally

Serve this directory (or `crates/radar_web/static/`) with any static file
server:

```bash
cd web/dashboard
python -m http.server 8000     # then open http://localhost:8000
```

or

```bash
cd web/dashboard
npx http-server . -p 8000
```

What to expect when there is no radar connected:

* The page renders and the layout is inspectable.
* **WS —** and the status pills stay disconnected/offline: `/ws` and `/status`
  only exist on RADAR-TX (`192.168.4.1`), so they fail against a static server.
  That is expected, not a bug.
* To see real data, open `http://192.168.4.1` from a device on the `ESP32-RADAR`
  AP instead.

## How to deploy a change

1. Edit `crates/radar_web/static/index.html` / `app.js` (keep the byte layouts
   in `app.js` in sync with `crates/radar_web/src/telemetry.rs`).
2. Rebuild the firmware: from `firmware/radar_tx`, source
   `../esp-env.sh` and run `esp_cargo build.log build --release`.
3. Flash RADAR-TX (serial) or upload the image over the air (`/ota`).
4. Optionally mirror the change here in `web/dashboard/` for the host-side copy.

## Keeping the two in sync

The two copies drift easily. The rule is: **`crates/radar_web/static/` is the
source of truth**; `web/dashboard/` is a mirror for convenience. When you
update one, update the other in the same change.
