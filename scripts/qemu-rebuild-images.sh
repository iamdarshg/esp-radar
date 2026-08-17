#!/usr/bin/env bash
# Rebuild the QEMU boot images from the freshly-built release ELFs.
#
# The QEMU harness (scripts/qemu-harness.sh) boots qemu_tx.bin / qemu_rx.bin /
# qemu_rx_rx2.bin — merged images carrying bootloader + partition table + app.
# After any firmware change, regenerate the app bins (esptool elf2image) and
# re-merge, then delete the RX2 image so the harness re-provisions it from the
# fresh radar_rx.bin. Run from the repo root:
#
#   bash scripts/qemu-rebuild-images.sh
#
# Env overrides (all optional, same names as the harness):
#   ESPTOOL      esptool(.exe) path
#   BOOTLOADER   bootloader.bin path
#   SCRATCH      scratch dir holding the images + logs (default .scratch/qemu)
#   RX_ELF / TX_ELF   the release ELF paths (default C:/rt/...)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ESPTOOL="${ESPTOOL:-C:/Espressif/python_env/Scripts/esptool.exe}"
BOOTLOADER="${BOOTLOADER:-C:/rt/xtensa-esp32-espidf/release/build/esp-idf-sys-8625b8f33a4b386c/out/build/bootloader/bootloader.bin}"
SCRATCH="${SCRATCH:-$REPO_ROOT/.scratch/qemu}"
TX_ELF="${TX_ELF:-C:/rt/xtensa-esp32-espidf/release/radar_tx}"
RX_ELF="${RX_ELF:-C:/rt/xtensa-esp32-espidf/release/radar_rx}"

# Normalize for native Windows executables.
if command -v cygpath >/dev/null 2>&1; then
  SCRATCH="$(cygpath -w "$SCRATCH")"
fi

# Preflight
[ -f "$ESPTOOL" ]     || { echo "ERROR: esptool not found: $ESPTOOL" >&2; exit 1; }
[ -f "$BOOTLOADER" ]  || { echo "ERROR: bootloader not found: $BOOTLOADER" >&2; exit 1; }
[ -f "$SCRATCH/qemu_partition-table.bin" ] || { echo "ERROR: qemu partition table missing: $SCRATCH/qemu_partition-table.bin" >&2; exit 1; }
[ -f "$TX_ELF" ]      || { echo "ERROR: TX ELF missing: $TX_ELF (build firmware first)" >&2; exit 1; }
[ -f "$RX_ELF" ]      || { echo "ERROR: RX ELF missing: $RX_ELF (build firmware first)" >&2; exit 1; }

echo "==> App images (esptool elf2image)"
"$ESPTOOL" --chip esp32 elf2image --flash_mode dio --flash_freq 40m --flash_size 4MB \
  -o "$SCRATCH/radar_tx.bin" "$TX_ELF"
"$ESPTOOL" --chip esp32 elf2image --flash_mode dio --flash_freq 40m --flash_size 4MB \
  -o "$SCRATCH/radar_rx.bin" "$RX_ELF"
echo "    TX app: $(stat -c %s "$SCRATCH/radar_tx.bin") B · RX app: $(stat -c %s "$SCRATCH/radar_rx.bin") B"

echo "==> QEMU merged images (bootloader + qemu partition table + app)"
"$ESPTOOL" --chip esp32 merge_bin -o "$SCRATCH/qemu_tx.bin" --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 "$BOOTLOADER" 0x8000 "$SCRATCH/qemu_partition-table.bin" 0x10000 "$SCRATCH/radar_tx.bin"
truncate -s 4M "$SCRATCH/qemu_tx.bin"
"$ESPTOOL" --chip esp32 merge_bin -o "$SCRATCH/qemu_rx.bin" --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 "$BOOTLOADER" 0x8000 "$SCRATCH/qemu_partition-table.bin" 0x10000 "$SCRATCH/radar_rx.bin"
truncate -s 4M "$SCRATCH/qemu_rx.bin"

echo "==> Remove RX2 image so the harness re-provisions it from the new radar_rx.bin"
rm -f "$SCRATCH/qemu_rx_rx2.bin"

echo "==> Done. Run: bash scripts/qemu-harness.sh"
