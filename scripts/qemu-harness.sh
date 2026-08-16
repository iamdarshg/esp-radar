#!/usr/bin/env bash
# QEMU integrated harness — 3-board ESP32 WiFi CSI radar (Task 5).
#
# Maps the three physical board roles (fixed compact RF head) onto three
# concurrent QEMU esp32 machines and asserts each board's boot milestones:
#
#   TX  (radar_tx app)   -> qemu_tx.bin        (it is the AP; no role line)
#   RX1 (radar_rx app)   -> qemu_rx.bin        (role inferred from hardware: RX1)
#   RX2 (radar_rx app)   -> qemu_rx_rx2.bin    (role provisioned via NVS: RX2)
#
# Every board boots to QEMU's documented WiFi ceiling: the firmware runs through
# the software stack to esp_phy_enable, where QEMU (no WiFi MAC/PHY emulation)
# asserts `esp_phy_enable phy_init.c:328`. That assert + reboot loop is the PASS
# marker for "booted to the WiFi ceiling", never a failure. The harness greps
# each board's UART log for the milestone lines and prints a per-board matrix.
#
# Inter-chip RF/WiFi traffic is NOT verifiable under QEMU (no WiFi MAC/PHY);
# that is Task 6 (host-side UDP). This harness only proves each role boots to
# the ceiling with the right role/config resolution.
#
# Usage:
#   bash scripts/qemu-harness.sh
#
# Env overrides (all optional):
#   QEMU_BIN     qemu-system-xtensa.exe path
#   IDF_PY       python that has esp_idf_nvs_partition_gen + esptool
#   ESPTOOL      esptool(.exe) path
#   NVS_GEN      nvs_partition_gen.py path
#   BOOTLOADER   bootloader.bin path (for the RX2 merge)
#   SCRATCH      scratch dir holding the images + logs (default .scratch/qemu)
#   TX_WAIT      timeout sec for the TX machine           (default 45)
#   RX1_WAIT     timeout sec for the RX1 machine           (default 70: a
#                second boot is needed for the NVS-persistence line)
#   RX2_WAIT     timeout sec for the RX2 machine           (default 45)
#
# Rerunnable: every run cleans the per-board logs first and regenerates the
# RX2-provisioned image only if it is missing.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

QEMU_BIN="${QEMU_BIN:-C:/Espressif/tools/qemu-xtensa/esp_develop_9.2.2_20250817/qemu/bin/qemu-system-xtensa.exe}"
IDF_PY="${IDF_PY:-C:/Espressif/python_env/Scripts/python.exe}"
ESPTOOL="${ESPTOOL:-C:/Espressif/python_env/Scripts/esptool.exe}"
NVS_GEN="${NVS_GEN:-C:/Espressif/frameworks/esp-idf-v5.4/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py}"
BOOTLOADER="${BOOTLOADER:-C:/rt/xtensa-esp32-espidf/release/build/esp-idf-sys-8625b8f33a4b386c/out/build/bootloader/bootloader.bin}"
SCRATCH="${SCRATCH:-$REPO_ROOT/.scratch/qemu}"

# QEMU / python / esptool are native Windows executables: they cannot open
# MSYS-style /d/... paths, so normalize SCRATCH to a Windows-style path.
if command -v cygpath >/dev/null 2>&1; then
  SCRATCH="$(cygpath -w "$SCRATCH")"
fi

TX_WAIT="${TX_WAIT:-45}"
RX1_WAIT="${RX1_WAIT:-70}"
RX2_WAIT="${RX2_WAIT:-45}"

# Exact 21-byte RadarConfig blob, little-endian per crates/radar_storage
# to_bytes(): version u32=1 | channel u8=6 | tx_rate_hz u16=200 |
# report_every u16=20 | pair_tolerance u16=10 | tx_power_db u8=0 |
# node_role u8=0x03 (RX2) | ant_txrx1 u16=0 | ant_txrx2 u16=0 |
# csi_shift u8=0 | reserved[0;3].
#   = 01 00 00 00 06 c8 00 14 00 0a 00 00 03 00 00 00 00 00 00 00 00
RX2_BLOB_HEX="0100000006c80014000a0000030000000000000000"

TX_IMG_SRC="$SCRATCH/qemu_tx.bin"
RX1_IMG_SRC="$SCRATCH/qemu_rx.bin"
RX2_IMG="$SCRATCH/qemu_rx_rx2.bin"

# Per-run pristine images: Task 4 booted qemu_tx.bin / qemu_rx.bin many times,
# so their NVS partitions already hold the persisted config. A rerunnable
# harness needs the boot-1 fresh-NVS milestones every run, so each run starts
# from a byte-identical copy with the NVS (+phy_init) region reset to erased
# 0xFF. The Task 4 artifacts are never modified.
TX_IMG="$SCRATCH/qemu_tx_fresh.bin"
RX1_IMG="$SCRATCH/qemu_rx_fresh.bin"

TX_LOG="$SCRATCH/qemu_tx_uart0.log"
RX1_LOG="$SCRATCH/qemu_rx_uart0.log"
RX2_LOG="$SCRATCH/qemu_rx_rx2_uart0.log"

die() { echo "ERROR: $*" >&2; exit 1; }

# Fill [offset, offset+size) of file with 0xFF (erased flash).
blank_region() {
  "$IDF_PY" -c "
import sys
path, off, size = sys.argv[1], int(sys.argv[2], 0), int(sys.argv[3], 0)
with open(path, 'r+b') as f:
    f.seek(off)
    f.write(b'\xff' * size)
" "$1" "$2" "$3" || die "blank_region failed on $1 @0x$2"
}

# Copy a Task 4 image to a per-run pristine copy (NVS + phy_init erased).
prepare_image() {
  local src="$1" dst="$2"
  if [ ! -f "$src" ]; then die "image missing: $src (run Task 4 first)"; fi
  cp -f "$src" "$dst" || die "cp failed for $dst"
  blank_region "$dst" 0x9000 0x7000   # nvs 0x9000/0x6000 + phy_init 0xf000/0x1000
}

# ---- preflight --------------------------------------------------------------
[ -f "$QEMU_BIN" ] || die "qemu not found at $QEMU_BIN"
[ -f "$TX_IMG_SRC" ]  || die "TX image missing: $TX_IMG_SRC (run Task 4 first)"
[ -f "$RX1_IMG_SRC" ] || die "RX1 image missing: $RX1_IMG_SRC (run Task 4 first)"

# ---- RX2 image provisioning (regenerate only if missing) --------------------
if [ ! -f "$RX2_IMG" ]; then
  echo "RX2 image missing -> provisioning NVS + merging $RX2_IMG"
  NVS_CSV="$SCRATCH/nvs_rx2.csv"
  NVS_BIN="$SCRATCH/nvs_rx2.bin"
  if [ ! -f "$NVS_CSV" ]; then
    # Namespace must be `radar` (crates/radar_storage/src/nvs.rs), key `config`.
    # v5.4 nvs_partition_gen accepts data/hex2bin (the older blob/hex names are
    # rejected by this module).
    printf 'key,type,encoding,value\nradar,namespace,,\nconfig,data,hex2bin,%s\n' "$RX2_BLOB_HEX" > "$NVS_CSV"
  fi
  if [ ! -f "$NVS_BIN" ]; then
    "$IDF_PY" "$NVS_GEN" generate "$NVS_CSV" "$NVS_BIN" 0x6000 || die "nvs_partition_gen failed"
  fi
  "$ESPTOOL" --chip esp32 merge_bin -o "$RX2_IMG" --flash_mode dio --flash_freq 40m --flash_size 4MB \
    0x1000 "$BOOTLOADER" 0x8000 "$SCRATCH/qemu_partition-table.bin" 0x9000 "$NVS_BIN" \
    0x10000 "$SCRATCH/radar_rx.bin" || die "esptool merge_bin failed for RX2"
  truncate -s 4M "$RX2_IMG" || die "truncate RX2 image failed"
fi
[ -f "$RX2_IMG" ] || die "RX2 image missing after provisioning: $RX2_IMG"

# ---- per-run pristine copies (deterministic boot-1 milestones) --------------
prepare_image "$TX_IMG_SRC"  "$TX_IMG"
prepare_image "$RX1_IMG_SRC" "$RX1_IMG"

# ---- clean logs (rerunnable) -------------------------------------------------
rm -f "$TX_LOG" "$RX1_LOG" "$RX2_LOG" \
      "$SCRATCH/qemu_tx_stderr.log" "$SCRATCH/qemu_rx_stderr.log" \
      "$SCRATCH/qemu_rx_rx2_stderr.log"

# ---- launch 3 QEMU machines concurrently -------------------------------------
run_board() {  # name img log wait
  local name="$1" img="$2" log="$3" wait_s="$4"
  local stderr="$SCRATCH/${name}_stderr.log"
  timeout "$wait_s" "$QEMU_BIN" -display none -machine esp32 \
    -drive file="$img",if=mtd,format=raw \
    -global driver=timer.esp32.timg,property=wdt_disable,value=true \
    -serial file:"$log" 2>"$stderr" &
  echo "launched $name (pid $!, wait ${wait_s}s) -> $log"
}

run_board "qemu_tx"    "$TX_IMG"  "$TX_LOG"  "$TX_WAIT"
run_board "qemu_rx"    "$RX1_IMG" "$RX1_LOG" "$RX1_WAIT"
run_board "qemu_rx_rx2" "$RX2_IMG" "$RX2_LOG" "$RX2_WAIT"

echo "waiting for all boards to hit the WiFi ceiling (or timeout)..."
wait
echo "all boards done; grepping milestone lines"
echo

# ---- assertion helpers -------------------------------------------------------
PASS=0
FAIL=0
PASSED=""
FAILED=""

check() {  # board milestone pattern
  local board="$1" milestone="$2" pattern="$3"
  if grep -q -- "$pattern" "$LOG_FILE"; then
    local line
    line="$(grep -m1 -- "$pattern" "$LOG_FILE")"
    PASS=$((PASS + 1))
    PASSED="$PASSED|$board|$milestone"
    printf 'PASS  %-4s  %-34s %s\n' "$board" "$milestone" "$line"
  else
    FAIL=$((FAIL + 1))
    FAILED="$FAILED|$board|$milestone"
    printf 'FAIL  %-4s  %-34s (pattern not found: %s)\n' "$board" "$milestone" "$pattern"
  fi
}

# A "must NOT contain" assertion (used to prove the NVS path was taken, not the
# hardware-inference path, on the RX2 machine).
check_absent() {
  local board="$1" milestone="$2" pattern="$3"
  if grep -q -- "$pattern" "$LOG_FILE"; then
    local line
    line="$(grep -m1 -- "$pattern" "$LOG_FILE")"
    FAIL=$((FAIL + 1))
    FAILED="$FAILED|$board|$milestone"
    printf 'FAIL  %-4s  %-34s (must be ABSENT but found: %s)\n' "$board" "$milestone" "$line"
  else
    PASS=$((PASS + 1))
    PASSED="$PASSED|$board|$milestone"
    printf 'PASS  %-4s  %-34s (correctly absent)\n' "$board" "$milestone"
  fi
}

echo "============================= BOARD MATRIX ============================="
printf '%-6s %-36s %s\n' "RESULT" "MILESTONE" "EVIDENCE"

# ---- TX (radar_tx / the AP) ---------------------------------------------------
LOG_FILE="$TX_LOG"
check "TX" "no-otadata benign marker" "mark_app_valid failed (0x00000105); continuing"
check "TX" "fresh NVS -> defaults"     "no config in NVS; writing defaults"
check "TX" "config printed"            "config: channel=6 rate=200Hz report_every=20 pair_tol=10 tx_power_db=0"
check "TX" "wifi-ceiling: phy clock"   "phy module clock bits 0x0, required 0x8f8f"
check "TX" "wifi-ceiling: phy assert"  "assert failed: esp_phy_enable phy_init.c:328"

# ---- RX1 (radar_rx / hardware-inferred RX1) -----------------------------------
LOG_FILE="$RX1_LOG"
check "RX1" "fresh NVS -> defaults"      "no config in NVS; writing defaults"
check "RX1" "boot1 role inferred"        "node role inferred from hardware: RADAR-RX1"
check "RX1" "boot2+ role persisted"      "node role from NVS: RADAR-RX1"
check "RX1" "config printed"             "config: channel=6 rate=200Hz report_every=20"
check "RX1" "wifi-ceiling: phy assert"   "assert failed: esp_phy_enable phy_init.c:328"

# ---- RX2 (radar_rx / NVS-provisioned RX2) -------------------------------------
LOG_FILE="$RX2_LOG"
check "RX2" "provisioned role from NVS" "node role from NVS: RADAR-RX2"
check_absent "RX2" "no hardware-inference line" "node role inferred from hardware"
check "RX2" "config printed"             "config: channel=6 rate=200Hz report_every=20"
check "RX2" "wifi-ceiling: phy assert"   "assert failed: esp_phy_enable phy_init.c:328"

echo "========================================================================="
echo
printf 'SUMMARY: %d passed, %d failed\n' "$PASS" "$FAIL"
if [ -n "$FAILED" ]; then
  echo "failed milestones:"
  echo "$FAILED" | tr '|' '\n' | awk 'NF' | while read -r b m; do echo "  - $b: $m"; done
fi

[ "$FAIL" -eq 0 ] && exit 0 || exit 1
