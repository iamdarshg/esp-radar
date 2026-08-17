# Verification

What is verified about the compact ESP-RADAR head, how it was verified, and what
can only be confirmed on the real hardware. The QEMU validation plan
(`docs/superpowers/plans/qemu-integrated-verification.md`) produced the evidence
below; this document is the summary you take to the flash bench.

Two kinds of proof are in play:

* **Deterministic checks** (builds, sizes, boot milestones, host-loopback comms) —
  fully verified on this machine, with log/command evidence.
* **On-hardware smoke items** — the things QEMU cannot exercise (real WiFi RF,
  real PSRAM on the ESP32-CAM, real radio power, real flash/OTA). These are
  listed as a checklist, not claimed as done.

---

## 1. Build & link (verified)

Both firmware release links are green through the Rust → ESP-IDF toolchain:

| Firmware | Release ELF | Verdict |
|----------|-------------|---------|
| `radar_tx` | `C:/rt/xtensa-esp32-espidf/release/radar_tx` | link `EXIT=0` |
| `radar_rx` | `C:/rt/xtensa-esp32-espidf/release/radar_rx` | link `EXIT=0` |

The link mechanism (ldproxy + app `build.rs` re-emitting esp-idf-sys's link
args) is the fix for both; `radar_rx` additionally links the `esp_psram`
component so its boot-time role inference (`src/link.rs::resolve_role`) has the
`esp_psram_is_initialized()` symbol. See `docs/flashing.md` → Building.

## 2. Image sizes & flash artifacts (verified)

The release profile at the workspace root (`opt-level="z"`, `lto="fat"`,
`codegen-units=1`, `panic="immediate-abort"`) plus newlib nano-format in both
`sdkconfig.defaults` keep both app images under their 1 MB slots:

| App image | Size | Gate (1,048,576 B) | Margin |
|-----------|------|--------------------|--------|
| `radar_tx` (`.scratch/flash/radar_tx.bin`) | 1,038,528 B | PASS | **10,048 B** |
| `radar_rx` (`.scratch/flash/radar_rx.bin`) | 929,344 B | PASS | 119,232 B |

Merged flash images (bootloader + explicitly generated partition table + app in
one file) are the flash-session deliverable:

| Image | Partition table | Flash to |
|-------|-----------------|----------|
| `.scratch/flash/radar_tx_merged.bin` | OTA (nvs, otadata, phy_init, factory, ota_0, ota_1) | RADAR-TX DevKit |
| `.scratch/flash/radar_rx_merged.bin` | default single-app (nvs, phy_init, factory) | RADAR-RX1 DevKit **and** RADAR-RX2 (via middle DevKit as USB-UART) |

Flash commands and the ESP32-CAM wiring are in `docs/flashing.md`. The partition
tables were generated explicitly from CSVs (never the native build's) and their
bytes were decoded and matched to the intended layout during review.

## 3. Boot milestones — QEMU, 3 machines (verified)

The QEMU harness (`scripts/qemu-harness.sh`, commit `da6f334`) boots three
concurrent `qemu-system-xtensa -machine esp32` instances mapped to the physical
roles, captures each board's UART log, and asserts the milestone lines.
**14/14 milestones PASS.** Evidence: `.scratch/qemu/qemu_tx_uart0.log`,
`qemu_rx_uart0.log`, `qemu_rx_rx2_uart0.log`.

Every machine reaches the **documented QEMU WiFi ceiling** — `assert failed:
esp_phy_enable phy_init.c:328` (QEMU has no WiFi PHY/modem-clock emulation).
That assert + reboot loop is the *pass marker* for "booted through the software
stack to WiFi bring-up"; it is not a failure.

| Board | Role resolved | Key milestone lines |
|-------|---------------|---------------------|
| TX | (AP) | `mark_app_valid failed (0x00000105); continuing` (benign — no otadata in QEMU table), `no config in NVS; writing defaults`, `config: channel=6 rate=200Hz report_every=20 pair_tol=10 tx_power_db=0`, WiFi ceiling |
| RX1 | `RADAR-RX1` | boot 1: `node role inferred from hardware: RADAR-RX1`; boot 2+: `node role from NVS: RADAR-RX1` (inference persisted to NVS); `config: ...`; WiFi ceiling |
| RX2 | `RADAR-RX2` | `node role from NVS: RADAR-RX2` (NVS-provisioned `node_role=0x03` at byte 12 of the 21-byte `RadarConfig` blob); `config: ...`; WiFi ceiling |

The RX2 role path is proven **without** real PSRAM by provisioning NVS with
`node_role=0x03`; on the real ESP32-CAM the same `radar_rx` binary reaches
`RADAR-RX2` by hardware inference (PSRAM present).

## 4. Inter-chip communications — host loopback (verified)

The three roles run as **three separate OS processes** on this machine,
communicating over real loopback UDP, exercising the genuine
`radar_transport` serializers/parsers and the real `radar_dsp` /
`radar_features` / `radar_calibration` logic:

```
tools/integration --orchestrate --duration 8 --rate 200
```

**14/14 assertions PASS** (reproducible). Measured in an 8 s window at 200 Hz:

| Flow | Assertion | Measured |
|------|-----------|----------|
| RATE-1 TX→RX1/RX2 (DataFrame) | frames_sent ≥ 1400; each RX ≥ 99% with 0 CRC / 0 loss / 0 resync / 0 seq gaps | 1600/1600 per RX (100%) |
| RATE-2 RX→TX (FeatureReport, every 20) | reports ≥ 90% of expected; pairs ≥ 60% of smaller link | 160/160 (100%), 80/80 (100%) |
| RATE-3 RX→TX (CsiSnapshot) | snapshots ≥ 8 | 32 |
| Fusion (TX Pairer + occupancy) | ≥ 1 fused output from a real pair | 80, final state STRONG MOVEMENT |

Transport note: on Windows loopback, multicast delivered the stream to only one
of the two joined receivers, so the tool uses the documented **two-unicast**
fallback — the same DataFrame bytes (same seq, same CRC) sent to each RX's own
loopback address, then both links paired on the same seq stream. Framing, CRC,
and sequence semantics are identical to what the boards will exercise over the
air; the only thing not exercised literally is the single-broadcast-datagram
reception by both RX stations. Real over-the-air reception of a broadcast is an
on-hardware smoke item below.

On the physical head the RATE-2/RATE-3/CAL flows (the ones this loopback
validates) move to the **wired UART data plane** (`docs/flashing.md` →
Normal-operation wiring) while the measurement plane stays WiFi. The loopback
harness is transport-agnostic: it exercises the same `radar_transport`
serializers, CRC and `Pairer`/`SequenceTracker` logic the wired links now carry.
The `framer` byte-stream decoder that pulls frames off the UART is covered
separately by host unit tests in `radar_transport` (see §5).

## 5. Host unit tests & pipeline emulation (prior, committed)

The pure crates carry unit tests (round-trips, CRC rejection, seq tracking) and
a host end-to-end pipeline emulation was run before this plan; those remain
green under the current tree (host `cargo test` from the repo root works; the
firmware-only release profile does not affect host builds). The wired data
plane adds `radar_transport::framer` — a byte-stream frame extractor (magic
hunt + length + CRC resync) with host tests for single/split/garbage-prefixed/
concatenated/corrupt/bounded/empty-payload frames — all green alongside the
existing transport tests.

---

## On-hardware smoke checklist (NOT yet verified — do these at the flash bench)

QEMU has no WiFi MAC/PHY, this machine has no ESP32 hardware, and the
ESP32-CAM's PSRAM is only present on the real board. The following are the
deliberate gaps — confirm each on the real head before trusting a deployment:

1. **AP + over-the-air RATE-1.** RADAR-TX's `ESP32-RADAR` AP comes up on
   channel 6; both RX stations associate and receive the 200 Hz broadcast
   stream (the measurement plane). This is the one thing QEMU/loopback cannot
   prove (single broadcast to both RX, real RF propagation). Use
   `tools/integration`'s three roles on the host as a comparison baseline, but
   the over-the-air path is hardware-only.
2. **Wired data plane (both links).** After wiring per `docs/flashing.md` →
   Normal-operation wiring, the dashboard at `http://192.168.4.1` should show
   both links reporting: each RX's RSSI/quality in `/status` and the waterfall
   updating (RATE-3 snapshots arrive over UART). A `/cal` round-trip completes
   — CAL 1 identity acks from both RX over the wire. If a link shows continuous
   CRC failures, check the RX pull-up / wiring first, then drop both sides to
   baud 230400.
3. **RADAR-RX2 role inference.** On the real ESP32-CAM, first boot should log
   `node role inferred from hardware: RADAR-RX2` (PSRAM present), then persist
   `RADAR-RX2` to NVS. If it logs `RADAR-RX1`, the PSRAM connection or
   `CONFIG_SPIRAM_BOOT_INIT` path needs attention.
4. **SPIRAM-configured bootloader on the no-PSRAM TX board.** The merged TX
   image carries the bootloader built with radar_rx's SPIRAM-enabled config
   (the brief's single-bootloader choice). `CONFIG_SPIRAM_IGNORE_NOTFOUND=y`
   should make the bootloader's PSRAM-init failure non-fatal — confirm TX boots
   to its AP.
5. **Panic behavior + logging.** `panic = "immediate-abort"` aborts without a
   Rust backtrace (the C panic handler still prints the message). The firmware
   panic path was never observed on the boot path; confirm nothing panics during
   normal operation, and that `log::info!` output (which goes through Rust's fmt,
   not newlib printf) is intact under newlib nano-format.
6. **DSP throughput at 200 Hz on radar_rx.** The workspace profile's
   `opt-level="z"` (size-optimized) applies to the DSP crates too. Confirm the
   RX boards keep up with the incoming 200 Hz stream (feature report cadence and
   CSI snapshots don't starve). If frames back up, a per-crate
   `[profile.release.package.radar_dsp]` opt-level override is the lever.
7. **OTA on RADAR-TX.** The real flash uses the otadata-bearing OTA table, so
   `mark_app_valid` should succeed (it fails benignly only because the QEMU
   table has no otadata). Exercise one dashboard OTA upload
   (`http://192.168.4.1/ota`) and the rollback path.
8. **Headroom.** `radar_tx` has only **10,048 B** of slot margin. Any firmware
   feature growth must be size-checked before it fits; the fallback is enlarging
   the OTA slots in `firmware/radar_tx/partitions_ota.csv` (do not shrink the
   factory slot below what the bootloader needs).

## Artifacts at a glance

| Artifact | Path |
|----------|------|
| Merged flash images | `.scratch/flash/radar_tx_merged.bin`, `.scratch/flash/radar_rx_merged.bin` |
| App images | `.scratch/flash/radar_tx.bin`, `.scratch/flash/radar_rx.bin` |
| QEMU boot images | `.scratch/qemu/qemu_tx.bin`, `qemu_rx.bin`, `qemu_rx_rx2.bin` (NVS-provisioned RX2) |
| QEMU harness | `scripts/qemu-harness.sh` |
| Inter-chip comms tool | `tools/integration/` (build: `cargo build` there; run `.\target\debug\integration.exe --orchestrate --duration 8 --rate 200`) |
| Plan + ledger | `docs/superpowers/plans/qemu-integrated-verification.md`, `.superpowers/sdd/qemu-integrated-verification/progress.md` |
