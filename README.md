# ESP-RADAR — Compact 2.4 GHz CSI Motion Radar

A single rigid three-ESP32 Wi-Fi sensing head that uses **Channel State
Information (CSI)** to detect presence and motion in the surrounding
environment. RADAR-TX broadcasts 2.4 GHz measurement frames; RADAR-RX1 and
RADAR-RX2 each capture per-packet CSI on the *same* packets, run the DSP
pipeline, report features back to TX, and TX fuses the two links into a live
occupancy/motion estimate served over its own Wi-Fi AP.

The three boards are mounted **once** on breadboards as one assembly and never
move. See the supplied photograph (`IMG_9F899C38-89DF-446E-BFB1-46FE287B91C3.jpeg`)
— it defines the hardware topology. Canonical layout (fixed):

```text
[ LEFT DevKit, rotated 180° ]  [ MIDDLE DevKit ]  [ RIGHT ESP32-CAM, rotated 180° ]
         = RADAR-RX1               = RADAR-TX               = RADAR-RX2
```

The middle board powers the whole head (its 3V3 feeds both neighbours on a
shared rail, common GND, CAM D5→GND return sink) and is the hub for both the
WiFi measurement plane and the wired UART data plane.

## The physical constraint — DO NOT MOVE THESE BOARDS

> The three ESP32 boards sit centimetres apart on breadboards and must remain in
> essentially this exact relative position and orientation for the completed
> radar. The system is a **single compact 2.4 GHz RF sensing head**, *not* three
> sensor stations distributed around a room. At 2.4 GHz the wavelength is
> ≈ 12.5 cm, so this is never treated as a metre-scale distributed array.
>
> Never:
> * put the receivers 1 m apart;
> * put the transmitter across the room;
> * form a triangle with the ESP32s;
> * move a receiver for calibration;
> * rotate a board between experiments;
> * reposition antennas for different sensing modes.
>
> The boards are mounted once and then remain fixed. Every algorithm,
> calibration, UI, filter and TX-power decision in this project is designed
> around that fact.

## System overview

### Node roles

| Node | Hardware | Role | Responsibilities |
|------|----------|------|-----------------|
| **RADAR-TX** | Middle ESP32 DevKit | AP + fusion + dashboard host + calibration host + OTA host | Generates the 2.4 GHz measurement traffic, owns the global packet sequence number, coordinates the radar session, receives processed features from RX1 and RX2 over the wired UART data plane, fuses them, and serves the standalone web dashboard, `/cal` and `/ota` endpoints. (The optional RP2350 coprocessor task is parked — its pins are the RX2 link.) |
| **RADAR-RX1** | Left ESP32 DevKit (rotated 180°) | CSI capture + DSP | Receives TX measurement frames, captures CSI, runs the DSP/PCA/STFT pipeline, and reports compact high-rate features to TX over a wired UART link. |
| **RADAR-RX2** | ESP32-CAM (right, rotated 180°) | CSI capture + DSP (same firmware as RX1) | Captures a second, spatially/diversely-placed CSI observation of the *same* TX packets and reports the same features over its own wired UART link. Runs the identical `radar_rx` firmware; the node role is resolved at boot (NVS, else PSRAM presence — only the ESP32-CAM has PSRAM). Carries a microSD slot; SD recording is currently deferred in the code. |

RX1 and RX2 are **independent, non-coherent** observations of the same
transmitted packets. They are paired by TX packet sequence number, never by RF
phase — the two ESP32s are not a coherent phased array.

### How the boards talk

```
RADAR-TX  (AP "ESP32-RADAR", 192.168.4.1, channel 6)
   |
   | MEASUREMENT PLANE (WiFi): UDP broadcast DataFrame, one per seq,
   | 192.168.4.255:4444, 200/s  ← the CSI stimulus
   |----------> RADAR-RX1  (STA, CSI capture + DSP)
   |----------> RADAR-RX2  (STA, CSI capture + DSP)
   |
   | DATA PLANE (wired UART, crossed 2-wire links, common GND):
   |<----------- FeatureReport (RX -> TX, every 20 frames)
   |<----------- CsiSnapshot  (RX -> TX, ~2 Hz, for the waterfall/spectrogram)
   |<----------- CAL_RESP     (RX -> TX)
   |-----------> CAL_CMD      (TX -> RX, both links)
   |
   |   (TX pairs RX1/RX2 reports by seq, fuses, runs occupancy classifier)
   |
   |--- HTTP  /        embedded dashboard
   |--- HTTP  /status  JSON status snapshot
   |--- WS    /ws      binary telemetry frames
   |--- HTTP  /cal     calibration control
   |--- HTTP  /ota     firmware upload
```

The RX boards **never transmit on 2.4 GHz** — they only receive the
measurement broadcast. Their reports, snapshots and calibration responses go
to TX over the wired UART data plane (`docs/flashing.md` → Normal-operation
wiring); TX's own radio keeps the measurement broadcast and the dashboard AP.

A phone/tablet connects directly to the `ESP32-RADAR` AP and opens
`http://192.168.4.1`. No laptop, router or internet is required after
programming.

## Repository layout

```
crates/                  shared Rust libraries (pure crates are host-testable)
  radar_protocol/        wire format: versioned headers, CRC-16/XMODEM, global seq
  radar_csi/             Wi-Fi CSI capture: short RX callback -> lock-free ring
  radar_dsp/             amplitude, normalization, baseline subtraction, filters,
                         PCA, STFT, motion spectra
  radar_features/        per-link features, RX1/RX2 fusion, occupancy state machine
  radar_calibration/     CAL 1-5 artifacts: empty-room baseline, TX power model,
                         classifier thresholds
  radar_transport/       wire frames: measurement broadcast (TX->RX WiFi) +
                         report/snapshot/cal builders+parsers + a byte-stream
                         `framer` for the wired UART data plane
                         (RX->TX), gap tracking, cross-link pairing
  radar_storage/         NVS persistence of config + calibration artifacts
  radar_web/             dashboard server: HTTP + WebSocket binary telemetry,
                         embedded dashboard files (static/)
  radar_ota/             partition-aware OTA via web upload
  radar_rp2350/          optional wired RP2350 coprocessor link (UART2), best-effort
firmware/
  esp-env.sh             reusable build environment + workarounds (see Building)
  radar_tx/              TX app: AP, traffic generator, fusion, dashboard,
                         calibration, OTA  (target: xtensa-esp32-espidf)
  radar_rx/              shared RX app (RX1 and RX2): CSI capture, DSP, features
web/
  dashboard/             host-side copy of the dashboard (see web/dashboard/README.md)
tools/                   host-side tools for working with captured radar data
                         (see docs/tools.md)
  replay/                re-send a capture over UDP, preserving timing
  decoder/               decode a capture to text/JSON, validate CRCs
  analysis/              aggregate stats over a capture (rates, RSSI, packet loss)
  test-generator/        synthesize deterministic radar streams (stdout or UDP)
docs/                    flashing, architecture, calibration, dashboard, tools
```

## Building

The firmware is Rust on ESP-IDF (`esp-idf-sys`), built with the Espressif
`esp` nightly Rust toolchain. There are **no prebuilt std binaries** for the
xtensa-esp32 target — std is compiled from source via `-Zbuild-std`. Each
firmware directory carries its own `.cargo/config.toml` (target
`xtensa-esp32-espidf`, target dir `C:/rt`, `build-std`).

### Firmware

Everything needed is encoded in `firmware/esp-env.sh`. **Source it from inside
a firmware directory** so `pwd` is that firmware's root (the script uses the
directory's `sdkconfig.defaults`), then use the `esp_cargo` wrapper:

```bash
cd firmware/radar_tx          # or firmware/radar_rx
source ../esp-env.sh          # Windows / Git Bash
esp_cargo build.log build --release
#                    ^--- cargo args (no `--` separator — esp_cargo runs `cargo <args>` verbatim)
# default logfile is check.log; output is appended to the logfile,
# which ends with "EXIT=<code>"
```

The resulting app ELF lands in the short side target dir `C:/rt/`
(`C:/rt/xtensa-esp32-espidf/release/<app>`).

`esp-env.sh` exists because the standard esp-idf-sys/embuild build needs seven
workarounds on this machine; it sets all of them and `esp_cargo` runs `cargo`
with that environment:

1. `RUSTUP_TOOLCHAIN=esp` — the esp nightly toolchain (has `-Zbuild-std`).
2. `IDF_PYTHON_ENV_PATH=C:/Espressif/python_env` — the IDF venv lives there
   directly, not under a `idf5.4_py3.12_env` subdir.
3. The venv `Scripts` dir is first on `PATH` — embuild resolves `python` via
   `which("python")` and passes it as `-DPYTHON` to CMake; the venv Python
   must win over the system Python.
4. `LIBCLANG_PATH` is set to the **directory** holding `libclang.dll`
   (clang-sys appends the platform libclang filename to this value).
5. The esp-clang `bin` dir is first on `PATH` so `libclang.dll`'s own runtime
   dependencies (libclang-cpp.dll, libLLVM-20.dll, ...) resolve from the
   build cwd on Windows.
6. `ESP_IDF_SDKCONFIG_DEFAULTS` is pointed at the current firmware dir's
   `sdkconfig.defaults` — without it the generated sdkconfig loses
   `CONFIG_HTTPD_WS_SUPPORT=y` (the WebSocket dashboard) and the OTA partition
   table for `radar_tx`.
7. **Single-core builds**: `NUM_JOBS=1` and `CARGO_BUILD_JOBS=1` cap the native
   ESP-IDF CMake/ninja build and rustc to one job each to keep RAM flat.
   (Raise them for speed on a machine with more memory.)

### Host-side pure crates

The four pure crates are the workspace `default-members`, so plain `cargo`
from the repo root touches only them:

```bash
cargo check      # or: cargo test   (radar_protocol, radar_dsp, radar_features, radar_calibration)
```

`cargo test` from the root runs their unit tests on the host. The `device`
feature crates (radar_csi, radar_transport, radar_storage, radar_web,
radar_ota, radar_rp2350) keep their ESP-only code behind a `device` feature so
their pure parts (the ring buffer, seq pairing, config serialization, telemetry
encoders) are host-testable too — but they are not in `default-members`.

## Flashing

All three boards are flashed over serial with the standard ESP-IDF-Rust tooling
(`espflash`); the RX2 ESP32-CAM has no USB port, so it is programmed through the
middle DevKit used as a USB-UART adapter.

* **RADAR-TX / RADAR-RX1** — normal serial flash from the firmware dir.
* **RADAR-RX2 (ESP32-CAM)** — wire the middle DevKit as a USB-UART adapter,
  hold its `EN` low, and flash through UART0.

Full wiring diagram (including the verbatim ESP32-CAM-via-middle-DevKit
procedure) and the OTA path are in **[docs/flashing.md](docs/flashing.md)**.

## Using the radar

1. **Power on** in this order: RADAR-TX first, then RADAR-RX1 and RADAR-RX2.
   RX retries association forever, so a cold-start RX that boots before the AP
   simply waits.
2. **Connect** a phone/tablet/laptop to the Wi-Fi AP **`ESP32-RADAR`**
   (no password).
3. **Open `http://192.168.4.1`**. The dashboard shows:
   * **Live status** — RADAR ACTIVE, channel, TX rate, paired frames/s, TX
     power, RX1/RX2 RSSI, CSI quality, saturation/dynamic range, packet
     delivery %, sequence.
   * **Live CSI waterfall** for RX1 and RX2 (time × subcarrier × normalized
     amplitude).
   * **Motion spectrogram** for RX1, RX2 and fused.
   * **Per-subcarrier live plot** — select RX1/RX2 and one of AMPLITUDE,
     NORMALIZED AMPLITUDE, TEMPORAL DERIVATIVE, RAW I, RAW Q, SANITIZED PHASE.
     (RAW I / RAW Q / SANITIZED PHASE are reported honestly as *unavailable* —
     they are not carried on the WS telemetry, rather than being synthesized.)
   * **Motion energy** — RX1, RX2 and fused time-series.
   * **Occupancy state** with confidence — EMPTY, POSSIBLE PRESENCE, STATIC
     PRESENCE, MOVEMENT, STRONG MOVEMENT, COMPLEX/MULTIPLE MOVEMENT, UNKNOWN.
   * **Differential channel display** — PCA1, PCA2, cross-link correlation,
     differential RMS, spectral entropy, dominant frequency.
4. **Calibrate** (see [docs/calibration.md](docs/calibration.md)):
   * On boot, TX auto-runs CAL 2 (RF power sweep) when `tx_power_db` is 0
     (not yet commissioned), so a fresh head self-commissions its RF power.
   * Run the stages manually from the `/cal` endpoint:
     `http://192.168.4.1/cal?stage=1..5`, `http://192.168.4.1/cal?auto=1`,
     `http://192.168.4.1/cal?abort=1`.
   * All calibration results persist in NVS; power cycling does not require
     re-calibration.
5. **Update firmware over the air** — `POST` a firmware image to
   `http://192.168.4.1/ota` (Content-Length required). The image is written to
   the inactive OTA slot, validated, and TX reboots into it. No serial cable
   needed.

## Host tools

Four host-side binaries under `tools/` work with captured radar data off the
device (see [docs/tools.md](docs/tools.md) for full usage). They read a raw
captured byte stream — a file, or stdin — and run with plain `cargo`:

```bash
cargo run -p decoder -- capture.bin          # decode frames to text (--json for JSON)
cargo run -p analysis -- --csv out.csv capture.bin   # aggregate stats + packet loss
cargo run -p replay -- capture.bin           # re-send capture over UDP to TX's report port
cargo run -p test-generator -- --frames 500  # synthesize a deterministic radar stream
```

## License

MIT OR Apache-2.0.
