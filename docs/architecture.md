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
| `radar_transport` | Wire-frame building/parsing + framing logic: measurement broadcast (TX→RX WiFi), `SequenceTracker`, `WindowCounter`, `Pairer` (pairs RX1/RX2 reports by seq), builders/parsers for all `"RDR1"` frames, and `framer` — a host-pure byte-stream decoder (magic hunt + length + CRC resync) that both firmware apps use to pull frames off the wired UART links. |
| `radar_storage` | NVS persistence: `RadarConfig` (channel, rate, report period, pair tolerance, TX power, node role, antenna offsets) and the calibration artifacts. Namespace `"radar"`. |
| `radar_web` | The dashboard server: HTTP (`/`, `/app.js`, `/status`) + WebSocket (`/ws`) binary telemetry, and the telemetry wire format (`"RTM1"`, kinds STATUS/WATERFALL/SPECTROGRAM). Serves the embedded dashboard files from `static/`. |
| `radar_ota` | Partition-aware firmware update: `OtaWriter` (begin/write/finish/abort), `mark_app_valid`, running/last-invalid partition labels. |
| `radar_rp2350` | Optional wired RP2350 coprocessor link. **Suspended**: its pins (UART2/GPIO16-17) are now the wired data-plane link to RADAR-RX2 (ESP32-CAM). Module stays compiled (`#[allow(dead_code)]`) for a future build that gives the coprocessor its own pins. |

## RADAR-TX boot flow (`firmware/radar_tx/src/main.rs`)

```
link_patches + logger
  └ mark_app_valid()              commit any OTA-updated image (no rollback later)
  └ load RadarConfig from NVS      (fresh flash → defaults: ch6, 200 Hz, tx_power=0)
  └ bring_up_ap()                  AP "ESP32-RADAR", channel from config, open auth
  └ set_tx_power()                 commissioned power, else DEFAULT_TX_POWER_DBM
  └ Dashboard::start()             HTTP :80 + WS /ws, /status snapshot shared state
  └ httpd::register()              add /cal and /ota endpoints
  └ open wired links               UART1 GPIO18/19 → RADAR-RX1, UART2 GPIO17/16 → RADAR-RX2
  └ spawn threads:
      traffic   → broadcast one DataFrame per seq at config.tx_rate_hz (WiFi measurement plane)
      fusion    → poll both wired links, pair RX1/RX2, fuse, classify, broadcast telemetry,
                  run the calibration controller (incl. boot auto-CAL2)
                  (copro task suspended — its pins are the RX2 link)
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
  └ open wired link                 role match: RX1 UART1 GPIO18/19, RX2 (CAM) UART1 GPIO14/13
  └ spawn radar task                drain ring, DSP pipeline, feature reports + CSI
                                    snapshots + CAL_RESP up the wired link, CAL_CMD
                                    down it
  └ sleep forever (leaked EspWifi + ring keep radio/collection alive)
```

The CSI callback fires for *every* received 802.11 frame (beacons, foreign
stations, ...), so the radar loop keeps only frames whose source MAC is the
RADAR-TX AP's BSSID — i.e. the broadcast measurement frames.

## Data flow: TX → RX → TX

The inter-board data plane is wired; only the measurement plane is WiFi:

```
 RATE-1 (per measurement frame, ~200 Hz)   — WiFi (the CSI stimulus)
   RADAR-TX ── UDP broadcast ──> RADAR-RX1 ┐
   (DataFrame,                RADAR-RX2 ┘  both capture CSI from the same packet
    seq, t_us, tx_power)                    (CSI requires real 2.4 GHz frames)
        │
        │  the two RX run the DSP pipeline independently and
        │  emit reports keyed by the shared sequence number
        │
 RATE-2 (per report window)               — wired UART
   RADAR-RX1 ── UART1 GPIO18 ──> RADAR-TX GPIO19
   RADAR-RX2 ── UART1 GPIO14 ──> RADAR-TX GPIO16   (FeatureReport, every
                                                    report_every frames, default
                                                    20 → ~10 Hz)

 RATE-3 (low rate)                        — wired UART
   RADAR-RX1/2 ── CsiSnapshot ──> RADAR-TX           (~2 Hz, raw-ish amplitude
                                                    matrix for the waterfall /
                                                    spectrogram)

 Calibration                              — wired UART (both directions)
   RADAR-TX ── GPIO18/17 ──> RX1/RX2      CAL_CMD (down)
   RADAR-RX1/2 ── ... ──> RADAR-TX        CAL_RESP (up)

   RADAR-TX fusion:
     Pairer     pairs RX1/RX2 reports by seq (tolerance)
     Fuser      correlation-weighted fusion of the two links
     Estimator  occupancy state + confidence
     StatusSnapshot → /status JSON (2 Hz) and StatusFrame over WS (1 Hz)
     Waterfall/Spectrogram frames over WS (1 Hz)
```

**Why wired:** the three boards form one rigid cm-scale sensing head, so any
WiFi transmission an RX makes is near-field self-interference on the sensing
band. Moving RATE-2/3 and CAL off WiFi means the RX boards **never transmit**
on 2.4 GHz — they only receive the measurement broadcast. TX's own radio keeps
its RATE-1 broadcast (the measurement plane) plus the AP the dashboard rides on.

TX owns the global sequence counter: every broadcast `DataFrame` carries the
next `seq`, and both RX echo that `seq` back in their reports. RX1/RX2 are
**independent, non-coherent** observations — they are paired by sequence
number, never by RF phase. The system is not a phased array.

## Calibration control flow

The calibration controller lives in RADAR-TX (`firmware/radar_tx/src/calibrate.rs`),
but the actual data collection happens on the receivers. TX broadcasts `CAL_CMD`
frames down **both** wired links; each RX arms the matching collector and
returns `CAL_RESP` frames up its own link with the gathered statistics. See
`docs/calibration.md` for the stages.
