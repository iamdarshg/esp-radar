# Flashing

How to get firmware onto the three boards, and how to update it afterwards.

There are two distinct paths:

* **Serial flashing** — for the initial programming of all three boards and for
  any recovery. Programs a **merged flash image** (bootloader + partition table
  + app in one file) with `esptool write_flash`; `espflash` may also work if
  installed.
* **Over-the-air (OTA)** — for `RADAR-TX` after it is running. Upload a new
  firmware image through the web dashboard at `http://192.168.4.1/ota`. No
  serial cable needed.

> Serial programming a *DevKit* is a normal USB-UART flash. The **ESP32-CAM**
> has no USB port, so it is programmed through the middle DevKit used as a
> temporary USB-UART adapter — see the dedicated section below. The wiring shown
> there is **temporary programming wiring only**: it must not be required during
> normal radar operation.

## Building first

Firmware is built with cargo inside `firmware/radar_tx` or `firmware/radar_rx`
(see `README.md` → Building). The flash tool programs the ELF that build
produces:

```
C:/rt/xtensa-esp32-espidf/release/radar_tx    <- RADAR-TX app
C:/rt/xtensa-esp32-espidf/release/radar_rx    <- RADAR-RX1 / RADAR-RX2 app
```

For serial flashing, the release pipeline also produces **merged flash images**
(`.scratch/flash/radar_tx_merged.bin`, `.scratch/flash/radar_rx_merged.bin` —
bootloader + partition table + app in one file) that `esptool` writes in a
single shot; see "Serial flashing" below.

## Serial flashing (RADAR-TX, RADAR-RX1)

Both DevKit boards expose a USB-UART bridge, so they flash the standard way.
The deterministic deliverable is a **merged flash image** (bootloader +
partition table + app concatenated at their fixed offsets in one file),
programmed with `esptool`:

```bash
# RADAR-TX (left DevKit) — OTA-capable table (factory + ota_0 + ota_1)
esptool --chip esp32 --port <COM> write_flash 0x0 .scratch/flash/radar_tx_merged.bin

# RADAR-RX1 (middle DevKit) — default single-app table
esptool --chip esp32 --port <COM> write_flash 0x0 .scratch/flash/radar_rx_merged.bin
```

* The merged images are built by the release pipeline
  (`docs/architecture.md` / Task 4 of the QEMU-validation plan): the workspace
  root `[profile.release]` (`opt-level = "z"`, `lto = "fat"`,
  `codegen-units = 1`, `panic = "immediate-abort"`) plus newlib nano-format in
  both `sdkconfig.defaults` keep radar_tx's app image at 1,038,528 B — under
  its 1 MB factory/OTA slots (radar_rx: 929,344 B). `esptool merge_bin` packs
  bootloader@0x1000, the explicitly generated partition table@0x8000 and the
  app (0x10000 or 0x20000) into a single image written at offset 0x0 — a blank
  chip boots straight off one write.
* `esptool` (esptool.py v4.12.0) is on the Python 3.12 Scripts `PATH` once
  `firmware/esp-env.sh` is sourced. `espflash` is **not installed** in this
  build environment; if it is installed later it can flash the ELF directly
  instead (`espflash flash --monitor C:/rt/xtensa-esp32-espidf/release/radar_tx`),
  but the merged image + `write_flash` above is the verified path.
* **RADAR-TX** uses a custom two-slot OTA partition table
  (`firmware/radar_tx/partitions_ota.csv`) — the `radar_tx_merged.bin` table is
  generated from it explicitly. RADAR-RX uses the default single-app table
  (`.scratch/flash/rx_default-partition-table.bin`).
* Select the board's serial port for `--port` (on Windows this is a COM port).
* For a recovery/erase, `esptool --chip esp32 --port <COM> erase_flash` followed
  by a fresh `write_flash` works.

## ESP32-CAM via the middle DevKit (RADAR-RX2)

The ESP32-CAM has **no USB port**. The middle ESP32 DevKit is used temporarily
as a serial (USB-UART) programmer. Its own ESP32 must be **disabled** so it does
not drive UART0 — this is done by holding the middle DevKit's **EN pin LOW**
(bound to GND).

### Temporary programming connections

```text
MIDDLE DEVKIT                 ESP32-CAM

GND ------------------------ GND

5V / VIN ------------------- 5V
                             ^
                             |
                   DO NOT CONNECT 5 V TO 3V3

TX0 / GPIO1 ---------------- U0R / GPIO3

RX0 / GPIO3 ---------------- U0T / GPIO1

GND ------------------------ GPIO0
                        [FLASH MODE ONLY]

EN ---------------- GND
(on middle DevKit)
```

UART lines are crossed logically:

```text
programmer TX → target RX
programmer RX ← target TX
```

### Flash sequence

1. Disconnect unnecessary peripherals.
2. Connect common GND.
3. Connect middle DevKit `EN` to GND so its ESP32 does not drive UART0.
4. Connect middle DevKit USB to computer.
5. Connect:
   * DevKit TX0 → ESP32-CAM U0R;
   * DevKit RX0 ← ESP32-CAM U0T.
6. Power the ESP32-CAM through its 5 V input.
7. Connect ESP32-CAM GPIO0 to GND.
8. Reset/power-cycle ESP32-CAM.
9. Flash `RADAR-RX2`.
10. Remove GPIO0 from GND.
11. Reset/power-cycle ESP32-CAM.
12. Remove the temporary programming UART wiring when complete.
13. Release middle DevKit EN from GND.

Do not require these UART wires during normal radar operation.

Flashing `RADAR-RX2` is otherwise identical to the DevKit flash: the same
`radar_rx` firmware (it is the *same binary* as RX1 — the node role is resolved
at boot from PSRAM presence). Use the serial port of the middle DevKit in place
of a native port:

```bash
esptool --chip esp32 --port <COM> write_flash 0x0 .scratch/flash/radar_rx_merged.bin
```

After boot, the ESP32-CAM detects its PSRAM, infers the `RADAR-RX2` role, and
persists it in NVS.

## Over-the-air updates (RADAR-TX)

Once RADAR-TX is running its AP, re-flash it without a cable:

1. Connect to the `ESP32-RADAR` AP.
2. Open `http://192.168.4.1/ota`.
3. `POST` the new firmware image with a valid `Content-Length` header
   (16 bytes to 2 MiB).
4. The image is streamed into the inactive OTA slot; when the upload finishes
   the slot is validated and set as the boot partition, and TX reboots into it.

The two-slot OTA partition table (`factory` + `ota_0` + `ota_1`) means a failed
or interrupted upload does not brick the head — TX keeps running the last good
image and can roll back.

## What to flash on each board

| Board | Firmware source | App | Method |
|-------|-----------------|-----|--------|
| RADAR-TX (left DevKit) | `firmware/radar_tx` | `radar_tx` | USB-UART serial, then OTA |
| RADAR-RX1 (middle DevKit) | `firmware/radar_rx` | `radar_rx` | USB-UART serial |
| RADAR-RX2 (ESP32-CAM) | `firmware/radar_rx` | `radar_rx` | middle DevKit as UART adapter |

> The merged images (`.scratch/flash/*.bin`) are built in TX-then-RX order, so
> the embedded bootloader reflects the **last** `esp-idf-sys` build (currently
> radar_rx's SPIRAM-enabled one). If the build order ever changes, re-run the
> merge so the images carry a consistent bootloader, and confirm each board
> boots to its expected milestone.
