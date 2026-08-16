# Flashing

How to get firmware onto the three boards, and how to update it afterwards.

There are two distinct paths:

* **Serial flashing** — for the initial programming of all three boards and for
  any recovery. Uses the standard ESP-IDF-Rust flash tool (`espflash`) against
  the ELF produced by the cargo build.
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

## Serial flashing (RADAR-TX, RADAR-RX1)

Both DevKit boards expose a USB-UART bridge, so they flash the standard way.
From the firmware directory (with `firmware/esp-env.sh` sourced), flash the
built app with the ESP-IDF-Rust flash tool:

```bash
cd firmware/radar_tx        # or firmware/radar_rx
espflash flash --monitor C:/rt/xtensa-esp32-espidf/release/radar_tx
```

* `espflash` is on `PATH` once `esp-env.sh` is sourced; point it at the built
  ELF explicitly (the target dir is the short `C:/rt/`, not the firmware's own
  `target/`). Do NOT route it through `esp_cargo` — that wrapper runs
  `cargo <args>` verbatim and only takes cargo invocations.
* **RADAR-TX** uses a custom two-slot OTA partition table
  (`firmware/radar_tx/partitions_ota.csv`). The build encodes it via
  `ESP_IDF_SDKCONFIG_DEFAULTS`, so `espflash` uses the right layout; if a
  future tool needs it explicitly, pass `--partition-table partitions_ota.csv`
  from `firmware/radar_tx`. RADAR-RX uses the default partition table.
* Select the board's serial port when prompted (on Windows this is a COM port).
* For a recovery/erase, `espflash erase-flash` followed by a fresh flash works.

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
cd firmware/radar_rx
espflash flash --monitor C:/rt/xtensa-esp32-espidf/release/radar_rx
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
