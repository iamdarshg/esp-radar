# Calibration

The radar head is a **fixed installation**. Calibration tunes the system to
the exact room it sits in, but it never moves the boards — the empty-room
baseline, TX power and thresholds are all learned while the boards stay exactly
where they were mounted.

There are five stages (spec §17):

| Stage | Name | Purpose |
|-------|------|---------|
| 1 | IDENTITY | Verify node roles and links are up (RX1/RX2 identified, links alive). |
| 2 | RF POWER | Sweep TX power, fit the RSSI-vs-power model, pick the power that hits the target RSSI. |
| 3 | EMPTY ROOM | Record the empty-room baseline (per-subcarrier amplitude mean/std) that the DSP normalizes against. |
| 4 | MOVING TEST | Learn the classifier thresholds from a known motion session. |
| 5 | FINGERPRINT | Record a per-subject/per-room fingerprint for the occupancy classifier. |

## How it runs

The controller is RADAR-TX (`firmware/radar_tx/src/calibrate.rs`). It sends
`CAL_CMD` frames to the receivers; each RX arms the matching collector and
streams `CAL_RESP` frames back with statistics. All results persist in NVS
(namespace `"radar"`), so **power cycling does not require re-calibration**.

You control it from the dashboard host:

* `http://192.168.4.1/cal?stage=1` .. `?stage=5` — run one stage.
* `http://192.168.4.1/cal?auto=1` — auto-commission: run CAL 2 unless a power
  model is already stored (same path as the boot-time auto-CAL2).
* `http://192.168.4.1/cal?abort=1` — abort the current stage.

### Boot auto-commission

On boot, if the config has `tx_power_db = 0` (not yet commissioned), RADAR-TX
runs a conservative default power and schedules an automatic **CAL 2** after a
short delay, once the links are up. This makes a fresh head self-configure its
RF power without any user action; stages 3-5 are still run explicitly.

## Stage 2 — RF power

The compact geometry means the antennas are only centimetres apart, so direct
coupling is strong. TX power is chosen to land at a fixed target RSSI rather
than "as loud as possible" (spec §5):

* Sweep TX powers: **4, 8, 12, 16, 20 dBm**, collecting **~800 ms** at each.
* Target received RSSI: **−45 dBm**.
* Fit a linear RSSI-vs-power model (`TxPowerModel`); `power_for_rssi` returns
  the power that hits the target, clamped to `[4, 20]` dBm.
* If a receiver reports a high saturation score during the sweep, it stops the
  sweep early (saturation stop threshold: 70).

The fitted model is stored in NVS (`powmodel` key).

## Stage 3 — empty-room baseline

The most important stage. With the room **empty**, the receiver accumulates the
per-subcarrier amplitude statistics (mean and std over all 56 subcarriers) that
the DSP uses to normalize live CSI into a z-score. The baseline is collected
over a **10 s** window, and requires at least **100 samples** to be accepted
(`MIN_BASELINE_SAMPLES`).

* Run it with the room empty and still.
* The boards must not have moved since it was recorded.
* Baseline is stored per link (`baseline1` / `baseline2` in NVS).

The `Normalizer` in the DSP (`crates/radar_dsp`) subtracts this baseline so the
rest of the pipeline sees only *changes* from the empty room — which is exactly
what presence and motion produce.

## Stage 4 — moving test

A **15 s** window during which a person moves through the sensing region. TX
collects motion-energy statistics and derives the classifier thresholds
(`ClassThresholds`) from the histogram — separating "empty", "static presence",
"movement" and "strong movement" for *this* room. Stored in NVS (`thresh` key).

## Stage 5 — fingerprint

A **5 s** recording used to capture a per-subject/per-room fingerprint for the
occupancy classifier. Stored alongside the other artifacts.

## What is NOT calibrated

The boards themselves never move. There is no step that asks you to reposition
a receiver, rotate a board, or spread the nodes around the room. If the
physical layout changes, the sensible path is to re-run calibration (at minimum
stages 3 and 4), not to "tune" the hardware.

## Where results live

| Artifact | NVS key | Produced by |
|----------|---------|-------------|
| Radar config (channel, rate, TX power, role, offsets) | `config` | boot / CAL 2 |
| Empty-room baseline, link 1 | `baseline1` | CAL 3 |
| Empty-room baseline, link 2 | `baseline2` | CAL 3 |
| Classifier thresholds | `thresh` | CAL 4 |
| TX power model | `powmodel` | CAL 2 |
