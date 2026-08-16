# Dashboard

The radar's live view, served by RADAR-TX. Connect a phone/tablet/laptop to the
`ESP32-RADAR` AP and open **`http://192.168.4.1`**. No internet, laptop or
router required — the AP and the dashboard are the radar itself.

The dashboard is a real, embedded web app: the HTML and JavaScript live in
`crates/radar_web/static/` and are baked into the firmware with `include_str!`.
A host-side copy of the same files lives in `web/dashboard/` (see
`web/dashboard/README.md`).

## Blocks (spec §6)

| Block | Content |
|-------|---------|
| **Live status** | RADAR ACTIVE, channel, TX rate (frames/s), paired frames/s, TX power, RX1/RX2 RSSI, CSI quality, saturation score, dynamic range, packet delivery %, sequence. |
| **Occupancy** | Current state + confidence bar (EMPTY / POSSIBLE PRESENCE / STATIC PRESENCE / MOVEMENT / STRONG MOVEMENT / COMPLEX/MULTIPLE MOVEMENT / UNKNOWN). |
| **Live CSI waterfall — RX1/RX2** | Time × subcarrier × normalized amplitude (56 subcarriers), one per link. |
| **Motion spectrogram — RX1/RX2/FUSED** | STFT/PCA motion spectrum, time × frequency (64 bins), including the fused link. |
| **Per-subcarrier live plot** | Select link (RX1/RX2) and metric: AMPLITUDE, NORMALIZED AMPLITUDE, TEMPORAL DERIVATIVE, RAW I, RAW Q, SANITIZED PHASE, plus a subcarrier slider. |
| **Motion energy** | RX1, RX2 and fused motion-energy time-series. |
| **Differential channel display** | PCA1, PCA2, cross-link correlation, differential RMS, spectral entropy, dominant frequency. |

The per-subcarrier plot's RAW I, RAW Q and SANITIZED PHASE are shown honestly
as *unavailable*: they are not carried on the WebSocket telemetry, and the
dashboard does not synthesize them (spec §7 — no fake data).

## How the dashboard gets data

Two sources, both from RADAR-TX:

1. **HTTP `GET /status`** — a compact JSON snapshot, polled by the page every
   ~2 s. Good for status text, no websocket needed.
2. **WebSocket `ws://192.168.4.1/ws`** — binary telemetry frames for the
   canvases. The JS opens this with `binaryType = "arraybuffer"` and reconnects
   on drop.

## WebSocket binary telemetry format

All integers little-endian. Every frame starts with the same 6-byte header:

```
magic u32 = 0x52544D31 ("RTM1") | version u8 = 1 | kind u8
```

Frame kinds: `0x01` STATUS, `0x02` WATERFALL, `0x03` SPECTROGRAM.

### StatusFrame (kind 0x01) — 66 bytes total

```
offset  size  field
0       4     magic u32
4       1     version u8
5       1     kind u8 = 0x01
6       1     occupancy u8             (0..6, see below)
7       1     confidence u8            (0..=100)
8       1     tx_power_db i8           (0 = not commissioned)
9       1     rssi_rx1 i8
10      1     rssi_rx2 i8
11      1     csi_quality_rx1 u8
12      1     csi_quality_rx2 u8
13      1     sat_score_rx1 u8
14      1     sat_score_rx2 u8
15      1     dyn_range_rx1 u8
16      1     dyn_range_rx2 u8
17      1     packet_delivery_pct u8   (0..=100)
18      2     paired_frames_s u16
20      4     seq u32
24      8     t_us u64
32      4     motion_energy_rx1 f32
36      4     motion_energy_rx2 f32
40      4     motion_energy_fused f32
44      4     spectral_entropy f32
48      2     dominant_freq_hz u16
50      4     pca1 f32
54      4     pca2 f32
58      4     correlation f32
62      4     differential f32
```

Occupancy wire codes: `0` UNKNOWN, `1` EMPTY, `2` POSSIBLE PRESENCE,
`3` STATIC PRESENCE, `4` MOVEMENT, `5` STRONG MOVEMENT, `6` COMPLEX/MULTIPLE
MOVEMENT.

### WaterfallFrame (kind 0x02) — 11-byte header + data

```
magic u32 | version u8 | kind u8=0x02 | link u8 | n_sub u8 | bins u16 | scale u8 |
data[n_sub*bins]
```

* `link`: `1` RX1, `2` RX2 (never FUSED for the waterfall).
* `n_sub`: number of subcarriers per time bin (fixed head: 56).
* `bins`: number of time bins.
* `scale`: right-shift applied to the 16-bit amplitudes before packing to u8
  (`amp ≈ raw << scale`) so the dashboard can reconstruct approximate values.
* `data`: `n_sub*bins` 8-bit normalized amplitudes, **time-major** (row = time
  bin).

### SpectrogramFrame (kind 0x03) — 11-byte header + data

```
magic u32 | version u8 | kind u8=0x03 | link u8 | n_freq u8 | bins u16 | scale u8 |
data[n_freq*bins]
```

* `link`: `1` RX1, `2` RX2, `3` FUSED.
* `n_freq`: number of frequency bins (64).
* `bins`: number of time bins.
* `data`: `n_freq*bins` 8-bit STFT/PCA magnitudes, **time-major**.

## `GET /status` JSON keys

```json
{
  "radar_active": 1,
  "channel": 6,
  "tx_rate_hz": 200,
  "cal_stage": 0,
  "cal_active": 0,
  "occupancy": "MOVEMENT",
  "occupancy_code": 4,
  "confidence": 87,
  "tx_power_db": 12,
  "rssi_rx1": -45, "rssi_rx2": -48,
  "csi_quality_rx1": 220, "csi_quality_rx2": 210,
  "sat_score_rx1": 0, "sat_score_rx2": 0,
  "dyn_range_rx1": 40, "dyn_range_rx2": 38,
  "packet_delivery_pct": 98,
  "paired_frames_s": 196,
  "seq": 123456, "t_us": 1720000000,
  "motion_energy_rx1": 0.42, "motion_energy_rx2": 0.38,
  "motion_energy_fused": 0.40,
  "spectral_entropy": 0.721,
  "dominant_freq_hz": 1,
  "pca1": 0.11, "pca2": 0.04,
  "correlation": 0.78, "differential": 0.19
}
```

## How to add a metric

The format is defined in `crates/radar_web/src/telemetry.rs` and decoded by
`crates/radar_web/static/app.js` — the two must stay in sync.

1. **Add the field to the struct.** In `telemetry.rs`, add the field to
   `StatusFrame` (or a matrix frame) and encode it in `encode()` (order
   matters — it is the wire layout). Bump `LEN`.
2. **Fill it on the TX.** In `firmware/radar_tx/src/fusion.rs`, set the new
   field in the `StatusFrame` the fusion task broadcasts, or in the matrix
   frame it builds.
3. **Decode it in the JS.** In `static/app.js`, read the new byte offsets in
   `decodeStatus` / `decodeMatrix` and draw or display them.
4. **Optionally surface it in `/status`.** Add the key to
   `StatusSnapshot::to_json()` in `telemetry.rs`.
5. **Rebuild the firmware** (`crates/radar_web/static/` is baked in via
   `include_str!`, so the JS changes are part of the firmware image).

Additions must not change the magic or version for existing fields; add new
fields at the end and, if it becomes necessary, bump `TELEMETRY_VERSION`.
