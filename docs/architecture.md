# Architecture

How the system is put together: the shared crates, the two firmware
applications, and the data flow between them.

## Component map

The repo is a Cargo workspace. Shared, reusable logic lives in `crates/`; the
two firmware applications live in `firmware/`. Pure crates are host-testable
(`cargo test` from the repo root); ESP-only code sits behind a `device`
feature.

| Crate | Responsibility |
|-------|----------------|
| `radar_protocol` | The wire format every board shares: `"RDR1"` magic, versioned header (src/dst node, seq, t_us, payload_len), CRC-16/XMODEM. Frame kinds: DATA, FEATURE_REPORT, CAL_CMD, CAL_RESP, STATUS, CSI_SNAPSHOT, CP_MESSAGE. Also the RP2350 coprocessor protocol (`"RCOP"`). |
| `radar_csi` | Wi-Fi CSI capture: `start_csi` installs the ESP32 CSI callback (a short memcpy) that pushes into a preallocated lock-free SPSC ring (`CsiRing`, 128 slots × 256 B). Never does DSP in the callback. |
| `radar_dsp` | The receiver-side pipeline: `decode_channel` (CSI bytes → 56-subcarrier complex channel), `Normalizer` (z-score vs. baseline), per-subcarrier Biquad band-pass (HP 0.2 Hz / LP 5 Hz), streaming PCA (power iteration + deflation), STFT/spectra, and metrics (energy, spectral entropy, dominant frequency, phase dispersion). |
| `radar_features` | Per-link feature structs, RX1/RX2 fusion (`Fuser`, correlation-weighted), and the occupancy state machine (`OccupancyEstimator` with hysteresis). Defines the occupancy states 0-6. |
| `radar_calibration` | Calibration artifacts (spec §17): empty-room baseline stats + collector, TX-power model (linear RSSI fit, CAL 2), classifier thresholds (CAL 4). All byte-serializable for NVS. |
| `radar_transport` | UDP framing and session logic: measurement port (TX→RX broadcast), report port (RX→TX unicast), `SequenceTracker`, `WindowCounter`, `Pairer` (pairs RX1/RX2 reports by seq), builders/parsers for the wire frames. |
| `radar_storage` | NVS persistence: `RadarConfig` (channel, rate, report period, pair tolerance, TX power, node role, antenna offsets) and the calibration artifacts. Namespace `"radar"`. |
| `radar_web` | The dashboard server: HTTP (`/`, `/app.js`, `/status`) + WebSocket (`/ws`) binary telemetry, and the telemetry wire format (`"RTM1"`, kinds STATUS/WATERFALL/SPECTROGRAM). Serves the embedded dashboard files from `static/`. |
| `radar_ota` | Partition-aware firmware update: `OtaWriter` (begin/write/finish/abort), `mark_app_valid`, running/last-invalid partition labels. |
| `radar_rp2350` | Optional wired RP2350 coprocessor link over UART2 (TX GPIO17, RX GPIO16), best-effort: frame decoder, versioned session (`compatible()`), probe/HELLO, status poll. |

## RADAR-TX boot flow (`firmware/radar_tx/src/main.rs`)

```
link_patches + logger
  └ mark_app_valid()              commit any OTA-updated image (no rollback later)
  └ load RadarConfig from NVS      (fresh flash → defaults: ch6, 200 Hz, tx_power=0)
  └ bring_up_ap()                  AP "ESP32-RADAR", channel from config, open auth
  └ set_tx_power()                 commissioned power, else DEFAULT_TX_POWER_DBM
  └ Dashboard::start()             HTTP :80 + WS /ws, /status snapshot shared state
  └ httpd::register()              add /cal and /ota endpoints
  └ spawn threads:
      traffic   → broadcast one DataFrame per seq at config.tx_rate_hz
      fusion    → receive reports, pair RX1/RX2, fuse, classify, broadcast telemetry,
                  run the calibration controller (incl. boot auto-CAL2)
      copro     → optional RP2350 link (probe, heartbeat, best-effort)
  └ sleep forever (leaked EspWifi + Dashboard keep AP/HTTP alive)
```

## RADAR-RX boot flow (`firmware/radar_rx/src/main.rs`)

The **same binary** runs on both receivers; the node role is resolved at boot:

```
link_patches + logger
  └ load RadarConfig from NVS       (defaults on fresh flash)
  └ resolve_role()                  NVS node_role → else PSRAM presence (ESP32-CAM
                                    has PSRAM → RX2, DevKit → RX1) → else RX1
  └ connect_sta()                   associate with AP "ESP32-RADAR" on the radar
                                    channel; returns AP BSSID for CSI MAC filtering
  └ start_csi()                     CSI callback → leaked CsiRing (128 slots)
  └ spawn radar task                drain ring, DSP pipeline, feature reports,
                                    CSI snapshots, calibration collectors
  └ sleep forever (leaked EspWifi + ring keep radio/collection alive)
```

The CSI callback fires for *every* received 802.11 frame (beacons, foreign
stations, ...), so the radar loop keeps only frames whose source MAC is the
RADAR-TX AP's BSSID — i.e. the broadcast measurement frames.

## Data flow: TX → RX → TX

```
 RATE-1 (per measurement frame, ~200 Hz)
   RADAR-TX ── UDP broadcast ──> RADAR-RX1 ┐
   (DataFrame,                RADAR-RX2 ┘  both capture CSI from the same packet
    seq, t_us, tx_power)
        │
        │  the two RX run the DSP pipeline independently and
        │  emit reports keyed by the shared sequence number
        │
 RATE-2 (per report window)
   RADAR-RX1 ── UDP unicast ──> RADAR-TX
   RADAR-RX2 ── UDP unicast ──> RADAR-TX     (FeatureReport, every report_every
                                              frames, default 20 → ~10 Hz)

 RATE-3 (low rate)
   RADAR-RX1/2 ── CsiSnapshot ──> RADAR-TX   (~2 Hz, raw-ish amplitude matrix
                                              for the waterfall/spectrogram)

   RADAR-TX fusion:
     Pairer     pairs RX1/RX2 reports by seq (tolerance)
     Fuser      correlation-weighted fusion of the two links
     Estimator  occupancy state + confidence
     StatusSnapshot → /status JSON (2 Hz) and StatusFrame over WS (1 Hz)
     Waterfall/Spectrogram frames over WS (1 Hz)
```

TX owns the global sequence counter: every broadcast `DataFrame` carries the
next `seq`, and both RX echo that `seq` back in their reports. RX1/RX2 are
**independent, non-coherent** observations — they are paired by sequence
number, never by RF phase. The system is not a phased array.

## Calibration control flow

The calibration controller lives in RADAR-TX (`firmware/radar_tx/src/calibrate.rs`),
but the actual data collection happens on the receivers. TX sends `CAL_CMD`
frames; each RX arms the matching collector and returns `CAL_RESP` frames with
the gathered statistics. See `docs/calibration.md` for the stages.
