# Plan: QEMU full-system validation + flash-ready binaries

**Deadline:** 2026-08-17 13:00 local — binaries flash-ready AND full-system QEMU validation complete.

## Goal

Produce flash-ready release firmware (`radar_tx`, `radar_rx`) and a QEMU-based
full-system validation that runs the three-board ESP-RADAR as an integrated
system and exercises inter-chip communications — not just standalone tests.

## Physical context (pinout photo, transcribed via Ollama `gemma4:12b` — existing model, no pulls)

Three boards laid in a horizontal row across two breadboards, **no wires
connected yet** (photo is of the bare mounted head):

| Position | Board | Labels | Role in firmware |
|----------|-------|--------|------------------|
| LEFT | ESP32-CAM (silver shield, camera connector) | IO0–IO10, 3V3, GND, VIN | RADAR-RX2 (PSRAM → RX2) |
| MIDDLE | ESP32 DevKit (micro-USB, "ESP32") | 3V3, GND, VIN, GPIO14/15 | RADAR-TX |
| RIGHT | ESP32 DevKit | 3V3, GND | RADAR-RX1 (no PSRAM → RX1) |

Inter-chip comms are WiFi only (RATE-1/2/3 per `docs/architecture.md`). The
QEMU harness must map the physical layout onto machines: RX2 on the PSRAM
machine, RX1/TX on non-PSRAM machines — role resolution is provable at boot.

## Global constraints (binding — copy into every brief)

- Boards are a FIXED compact RF head; never instruct repositioning receivers/antennas.
- Rust only; NVS-only storage (no SD).
- **Use existing models only — no new Ollama pulls** (`gemma4:12b` is the local vision model).
- Builds share target-dir `C:/rt` → run **sequentially**; `NUM_JOBS=3 CARGO_BUILD_JOBS=3` after sourcing `firmware/esp-env.sh`.
- `esp_cargo` takes **no `--`**; returns 0 always; real result is the `EXIT=` line at the end of the log.
- `MSYSTEM` env var breaks `idf_tools.py` under Git Bash → use the **PowerShell tool** for idf_tools.py, or extract the QEMU archive directly.
- QEMU (Espressif fork, `-machine esp32`) has **no WiFi MAC** → firmware boot ends at WiFi bring-up; harness documents what each binary reaches. Never promise RF emulation.
- `-Zbuild-std` std objects reference libc/pthread → final link must include newlib, `libpthread`, `libgcc` and the esp-idf static libs. **Current blocker: `radar_tx` release link fails with undefined libc symbols (EXIT=101).**

## Task 1 — Fix radar_tx release link

(BLOCKER) Make `cargo build --release` (via `esp_cargo`) link. Deliverable:
`C:/rt/xtensa-esp32-espidf/release/radar_tx` ELF + clean link log (`EXIT=0`).

Diagnosis: `.cargo/config.toml` is missing `[target.xtensa-esp32-espidf]`
`linker = "ldproxy"`. ldproxy.exe is installed at
`C:/Users/Darsh Gupta/.cargo/bin/ldproxy.exe`. See the SDD ledger for the full
root-cause notes and fallback hypotheses.

## Task 2 — Build radar_rx release

Sequential after Task 1 (shared `C:/rt`). Deliverable:
`C:/rt/xtensa-esp32-espidf/release/radar_rx` ELF. Apply the same config fix to
`firmware/radar_rx/.cargo/config.toml` if not already done.

## Task 3 — Install ESP32-capable QEMU on Windows

`idf_tools.py install qemu-xtensa` failed with `0xC0000135`
(STATUS_DLL_NOT_FOUND) on the tool check and rolled back the tools dir.
`C:\Espressif\dist\qemu-xtensa-softmmu-esp_develop_9.2.2_20250817-x86_64-w64-mingw32.tar.xz`
is already downloaded (34 MB). Extract it manually to
`C:\Espressif\tools\qemu-xtensa\esp_develop_9.2.2_20250817`, resolve the
missing mingw runtime DLL, verify `qemu-system-xtensa -machine esp32`.

## Task 4 — Build merged flash images

Build merged flashable images (bootloader + partition table + app) for both
firmware from the release ELFs, per `docs/flashing.md`. Deliverable: merged
`.bin` images + verified flash commands.

## Task 5 — QEMU integrated harness

Launch 3 concurrent `qemu-system-xtensa -machine esp32` instances (TX, RX1,
RX2 with the PSRAM/layout mapping above), capture per-board UART logs, assert
boot milestones (logger, NVS config load, role resolution RX1/RX2, AP/STA
bring-up attempt), write a pass/fail integration report. Document the
WiFi-ceiling per binary.

## Task 6 — Integrated multi-node inter-chip comms test

Host-side test/tool running TX-broadcast / two-RX DSP+report / TX-fusion as
separate UDP processes using `radar_transport`/`radar_dsp`/`radar_features`,
asserting RATE-1/2/3 flows end-to-end. This is the "communications between the
chips" verification.

## Task 7 — Final whole-branch review + verification report

`docs/verification.md` for the flash session: what's verified, what QEMU can't
prove, on-hardware smoke checklist.

## Notes

- Pinout transcription already complete (this file's Physical context section).
- #18 host unit tests and #19 host pipeline emulation are already DONE (committed work).
