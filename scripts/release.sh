#!/usr/bin/env bash
#
# Create (or update) a GitHub release with the flash-ready binaries.
#
#   scripts/release.sh [TAG]     # default TAG = v1
#
# The binaries live in .scratch/flash/ and were cross-built on the dev machine
# (which has the Espressif `esp` nightly-fork toolchain + ESP-IDF; see
# docs/verification.md). CI does NOT build firmware — it tests + lints the host
# crates and runs the inter-chip comms integration gate.
#
# The tag is created at HEAD, so run this with the source tree the binaries
# were built from checked out (see docs/verification.md for which commit).
#
# Idempotent: if the tag already exists it is not re-created; if the release
# already exists it is updated with the (same) assets.
set -euo pipefail

TAG="${1:-v1}"
TITLE="ESP-RADAR $TAG"
FLASH_DIR="$(cd "$(dirname "$0")/.." && pwd)/.scratch/flash"

ASSETS=(
  "radar_tx_merged.bin"
  "radar_rx_merged.bin"
  "radar_tx.bin"
  "radar_rx.bin"
  "tx_ota-partition-table.bin"
  "rx_default-partition-table.bin"
)

echo "==> Preflight: checking $FLASH_DIR for release assets"
for a in "${ASSETS[@]}"; do
  if [[ ! -f "$FLASH_DIR/$a" ]]; then
    echo "ERROR: missing $FLASH_DIR/$a — build the firmware first (docs/verification.md)" >&2
    exit 1
  fi
done

echo "==> Creating annotated tag $TAG at HEAD"
if git rev-parse "$TAG" >/dev/null 2>&1; then
  echo "    tag $TAG already exists — reusing"
else
  git tag -a "$TAG" -m "$TITLE"
  git push origin "$TAG"
fi

NOTES="$(mktemp)"
trap 'rm -f "$NOTES"' EXIT

cat > "$NOTES" <<'EOF'
## v1 — flash-ready binaries

The three ESP32 boards (LEFT ESP32-CAM → RADAR-RX2, MIDDLE DevKit → RADAR-TX,
RIGHT DevKit → RADAR-RX1) run the apps below. The measurement plane is WiFi
(RATE-1 broadcast — the CSI stimulus); the data plane is **wired UART**
(RATE-2 FeatureReports, CSI snapshots, CAL_CMD/CAL_RESP), so the RX boards
never transmit on the 2.4 GHz sensing band. The boards form one rigid, fixed
sensing head — do not move the antennas/receivers apart.

### Assets

| File | Purpose | Size |
|---|---|---|
| `radar_tx_merged.bin` | **TX full flash image** (OTA partition table, bootloader + app) | 1,169,600 B |
| `radar_rx_merged.bin` | **RX1/RX2 full flash image** (single-app table, bootloader + app) | 994,880 B |
| `radar_tx.bin` | TX app-only — the web dashboard OTA upload target | 1,038,528 B |
| `radar_rx.bin` | RX app-only | 929,344 B |
| `tx_ota-partition-table.bin` | TX OTA partition table (extract from merged image) | 3,072 B |
| `rx_default-partition-table.bin` | RX default partition table | 3,072 B |

Both apps fit their ~1 MB slots (TX app 1,038,528 B, RX app 929,344 B).
Use `esptool write_flash` with the merged images (see `docs/flashing.md`).

### Verification

- **Build/link:** `EXIT=0` for both apps (espidf std, `-Zbuild-std`).
- **QEMU full-system boot:** 14/14 milestones PASS for TX and both RX apps.
- **Inter-chip comms:** 14/14 assertions PASS (`tools/integration --orchestrate`)
  — RATE-1/2/3 framing, CRC, sequence integrity (0 gaps), RX1/RX2 pairing,
  fusion/controller on TX. The loopback is transport-agnostic: the same
  `radar_transport` serializers/CRC/Pairer now carry the wired UART data plane
  on hardware (the byte-stream `framer` for the links is host-tested).
- **Host tests:** 70/70 unit tests across the 10 host crates (incl. the
  `radar_transport::framer` byte-stream cases); lint clean (rustfmt + clippy
  `-D warnings`).
- On-hardware smoke checklist: `docs/verification.md`.

Binaries built from the source at this tag. `cargo` job `NUM_JOBS=3`.

### License

All rights reserved. No license is granted to use, copy, modify, or distribute
this project or its binaries.
EOF

echo "==> Creating/updating release $TAG"
# `gh release create` takes assets as positional args (no --attach flag).
FILES=()
for a in "${ASSETS[@]}"; do
  FILES+=("$FLASH_DIR/$a")
done

if gh release view "$TAG" >/dev/null 2>&1; then
  echo "    release $TAG already exists — updating"
  gh release edit "$TAG" --title "$TITLE" --notes-file "$NOTES" \
    "${FILES[@]}" >/dev/null
else
  gh release create "$TAG" --title "$TITLE" --notes-file "$NOTES" \
    "${FILES[@]}"
fi

echo "==> Done: $(gh release view "$TAG" --json url -q .url)"
