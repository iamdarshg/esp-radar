\# CRITICAL PHYSICAL CONSTRAINT — DO NOT MOVE THESE BOARDS



The supplied photograph is the canonical physical construction of this radar.



\*\*Do not redesign the geometry.\*\*



The three ESP32 boards are mounted immediately next to one another on breadboards and must remain in essentially this exact relative position and orientation for the completed radar.



The completed system is therefore a \*\*single compact 2.4 GHz RF sensing head\*\*, not three sensor stations distributed around a room.



All algorithms, calibration, UI, filtering, TX-power control and interpretation must be designed around this fact.



Never instruct the user to:



\* put the receivers 1 m apart;

\* put the transmitter across the room;

\* form a triangle with the ESP32s;

\* move a receiver for calibration;

\* rotate a board between experiments;

\* reposition antennas for different sensing modes.



The boards are mounted once and then remain fixed.



\---



\# 1. EXACT RELATIVE LAYOUT FROM THE PHOTOGRAPH



Viewed from the component side exactly as in the supplied photograph:



```text

&#x20;         TOP OF PHOTOGRAPH



&#x20;      USB                         PCB ANTENNA

&#x20;       │                               │

&#x20;       ▼                               ▼



┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐

│                  │   │   PCB ANTENNA    │   │    ESP32-CAM     │

│  LEFT ESP32      │   │                  │   │                  │

│  DEVKIT          │   │  MIDDLE ESP32    │   │  FPC CONNECTOR   │

│                  │   │  DEVKIT          │   │                  │

│                  │   │                  │   │                  │

│                  │   │                  │   │                  │

│ PCB ANTENNA      │   │                  │   │   microSD SLOT   │

│                  │   │                  │   │                  │

└──────────────────┘   └──────────────────┘   └──────────────────┘

&#x20;                            ▲

&#x20;                            │

&#x20;                           USB



&#x20;         BOTTOM OF PHOTOGRAPH

```



Important details visible in the photograph:



1\. The \*\*left DevKit is rotated 180° relative to the middle DevKit\*\*.

2\. The left DevKit USB connector is toward the top of the assembly.

3\. The left DevKit ESP32 PCB antenna is consequently toward the bottom.

4\. The middle DevKit ESP32 PCB antenna is toward the top.

5\. The middle DevKit USB connector is toward the bottom.

6\. The ESP32-CAM sits immediately to the right of the middle DevKit.

7\. All three boards are separated by only the small physical gaps visible in the photograph.

8\. The breadboards and board arrangement shown should be regarded as one rigid mechanical assembly.



Do not assign fictitious metre-scale coordinates to these devices.



\---



\# 2. NODE ASSIGNMENT



Use the boards in the following roles.



\## LEFT DEVKIT — RADAR-TX



The left ESP32 DevKit becomes:



```text

RADAR-TX

```



Responsibilities:



\* generate controlled 2.4 GHz measurement traffic;

\* maintain global packet sequence numbering;

\* coordinate the radar session;

\* perform system control;

\* receive processed features from RX1 and RX2;

\* perform feature fusion;

\* host the standalone web dashboard;

\* host configuration/calibration pages;

\* expose diagnostic information;

\* optionally communicate with an RP2350 in future.



The reason for selecting this physical board as TX is that its ESP32 antenna is at the opposite end of the assembly from the antennas on the other boards, producing the largest useful TX/RX separation obtainable \*\*without altering the photographed construction\*\*.



Do not change that assignment unless hardware inspection proves the assumed antenna location wrong.



\---



\## MIDDLE DEVKIT — RADAR-RX1



The middle ESP32 DevKit becomes:



```text

RADAR-RX1

```



Responsibilities:



\* receive RADAR-TX measurement frames;

\* capture CSI;

\* maintain full-rate local CSI buffers;

\* calculate amplitude and usable relative-phase information;

\* perform filtering;

\* perform PCA/feature extraction;

\* calculate motion spectra;

\* report compact high-rate features to RADAR-TX.



Its onboard PCB antenna remains exactly where it is in the photograph.



\---



\## RIGHT ESP32-CAM — RADAR-RX2



The ESP32-CAM becomes:



```text

RADAR-RX2

```



Responsibilities:



\* capture a second CSI observation of the same TX packets;

\* calculate the same basic DSP features as RX1;

\* provide spatial/channel diversity;

\* use the onboard microSD slot for radar data recording;

\* maintain rolling logs;

\* optionally store selected raw CSI bursts when triggered.



No camera is required.



Do not require the FPC camera connector.



\---



\# 3. TREAT THE THREE BOARDS AS A RIGID SENSOR HEAD



Define a sensor coordinate system attached to the assembly.



Use:



```text

+X = left → right across the photograph

+Y = bottom → top of the photograph

+Z = outward normal from the PCB/component faces

```



Define the \*\*actual RF antenna centres\*\*, rather than MCU package centres, as:



```text

A\_TX

A\_RX1

A\_RX2

```



Set:



```text

A\_TX = origin = (0,0,0)

```



and represent the others as:



```text

A\_RX1 = (dx1, dy1, dz1)

A\_RX2 = (dx2, dy2, dz2)

```



Do NOT invent these millimetre values from software assumptions.



The supplied photograph determines the topology and orientation, but an ordinary photograph is not an accurate dimensional calibration standard.



Therefore the implementation must include a one-time configuration field:



```text

TX → RX1 antenna-centre offset

TX → RX2 antenna-centre offset

```



in millimetres.



The user may measure those distances once with a ruler/caliper after fixing the boards permanently.



Once entered:



```text

THE VALUES NEVER CHANGE DURING NORMAL USE.

```



Store them in NVS.



The sensing algorithms must not require those values to be adjusted repeatedly.



Even if the user never enters millimetre measurements, the radar must still function using an empirical fixed-head calibration.



\---



\# 4. IMPORTANT CONSEQUENCE OF THE COMPACT GEOMETRY



These ESP32s are only centimetres apart.



At approximately 2.4 GHz:



```text

wavelength ≈ 12.5 cm

```



Therefore this system should NOT pretend to be a metre-scale distributed antenna array.



Instead exploit:



\* independent RF receiver chains;

\* different antenna locations;

\* different antenna orientations;

\* different board-level multipath;

\* different CSI responses;

\* common transmitted packets;

\* temporal CSI variation;

\* environmental multipath.



Treat RX1 and RX2 as two independent observations:



```text

TX packet n

&#x20;   │

&#x20;   ├──── environment/multipath ────> RX1 → H1\[k,n]

&#x20;   │

&#x20;   └──── environment/multipath ────> RX2 → H2\[k,n]

```



The useful information comes largely from how those channels \*\*change\*\* when something in the surrounding environment moves.



\---



\# 5. DIRECT-COUPLING MANAGEMENT IS A FIRST-CLASS FEATURE



Because TX and RX are physically close, the direct TX→RX component may be extremely strong.



Do not ignore this.



Implement automatic commissioning specifically for this compact geometry.



At startup/calibration:



1\. Start RADAR-TX at low Wi-Fi TX power.

2\. Measure RSSI and CSI quality at RX1.

3\. Measure RSSI and CSI quality at RX2.

4\. Increase/decrease TX power until both receivers have:



&#x20;  \* reliable packet reception;

&#x20;  \* good CSI dynamic range;

&#x20;  \* minimal clipping/saturation;

&#x20;  \* useful sensitivity to environmental multipath.

5\. Store the chosen power.

6\. Periodically monitor whether conditions have materially changed.



Use:



```c

esp\_wifi\_set\_max\_tx\_power(...)

```



for software TX-power control.



Do not simply operate at maximum RF output.



Create dashboard telemetry:



```text

TX power

RX1 RSSI

RX2 RSSI

RX1 CSI saturation score

RX2 CSI saturation score

RX1 dynamic range

RX2 dynamic range

packet delivery %

```



Provide an:



```text

AUTO RF GAIN / TX POWER CALIBRATION

```



button.



The target is \*\*not maximum RSSI\*\*.



The target is a useful CSI measurement channel.



\---



\# 6. WHAT THE DISPLAY SHOULD SHOW



Because this is a compact fixed radar head, the primary live dashboard should prioritize quantities this hardware can genuinely measure.



Create a polished live dashboard with the following.



\## LIVE STATUS



```text

RADAR ACTIVE

channel: 6

TX rate: 200 packets/s

paired frames: 196/s

TX power: ...

RX1 RSSI: ...

RX2 RSSI: ...

RX1 quality: ...

RX2 quality: ...

```



\## LIVE CSI WATERFALL — RX1



```text

time

vs.

OFDM subcarrier

vs.

normalized amplitude

```



\## LIVE CSI WATERFALL — RX2



Same representation.



\## RX1 MOTION SPECTROGRAM



STFT/PCA-derived temporal spectrum.



\## RX2 MOTION SPECTROGRAM



Same.



\## FUSED MOTION SPECTROGRAM



Combine useful signal components from both links.



\## PER-SUBCARRIER LIVE PLOT



Allow selection of:



```text

RAW I

RAW Q

AMPLITUDE

NORMALIZED AMPLITUDE

SANITIZED PHASE

TEMPORAL DERIVATIVE

```



for RX1 and RX2.



\## MOTION ENERGY



Time-series:



```text

RX1

RX2

fused

```



\## OCCUPANCY STATE



At minimum:



```text

EMPTY

POSSIBLE PRESENCE

STATIC PRESENCE

MOVEMENT

STRONG MOVEMENT

COMPLEX/MULTIPLE MOVEMENT

UNKNOWN

```



with confidence.



\## DIFFERENTIAL CHANNEL DISPLAY



Show quantities such as:



```text

RX1 activity / RX2 activity

RX1-RX2 correlation

RX1-RX2 differential response

PCA1

PCA2

spectral entropy

```



This is especially important because the two nearby receivers will respond differently to environmental multipath.



\---



\# 7. DO NOT FAKE A NORMAL RADAR PPI



Do not show an impressive circular radar screen with fake dots at arbitrary distances.



This hardware cannot directly provide FMCW-like range bins.



If a spatial display is included, call it:



```text

ENVIRONMENTAL RESPONSE MAP

```



or:



```text

MOTION LIKELIHOOD DISPLAY

```



not:



```text

RADAR RANGE MAP

```



unless actual ranging has been experimentally demonstrated.



Use confidence and uncertainty.



\---



\# 8. FIXED-INSTALLATION SPATIAL LEARNING



Because the hardware itself never moves, exploit that heavily.



The radar should become better after installation.



Implement:



```text

EMPTY-ROOM CALIBRATION

```



which records the stable CSI environment of the final installed sensor head.



Build per-link baselines:



```text

B1\[k]

B2\[k]

```



plus covariance/statistical models.



After calibration:



```text

ΔH1\[k,t] = H1\[k,t] - baseline1\[k]

ΔH2\[k,t] = H2\[k,t] - baseline2\[k]

```



or a robust normalized equivalent.



This fixed baseline is valuable precisely because the hardware geometry never changes.



Allow slow long-term environmental adaptation, but freeze rapid adaptation whenever motion is detected.



\---



\# 9. OPTIONAL FINGERPRINT MAPPING WITHOUT MOVING THE BOARDS



If coarse location/direction estimation is attempted:



\*\*the person moves; the ESP32 boards do not.\*\*



Calibration can ask the user to stand or move at several known positions around the fixed sensor head.



For example:



```text

&#x20;      P7   P8   P9



&#x20;      P4 SENSOR P6



&#x20;      P1   P2   P3

```



Collect fingerprints such as:



```text

RX1 normalized CSI

RX2 normalized CSI

RX1 PCA components

RX2 PCA components

spectral energy

RX1/RX2 ratio

cross-link correlation

```



Then train a lightweight classifier/regressor.



This can provide experimental labels such as:



```text

LEFT

RIGHT

FRONT

FARTHER FRONT

NEAR

MOVING LEFT→RIGHT

MOVING RIGHT→LEFT

```



if the data supports them.



Do not claim centimetre-level localization.



\---



\# 10. STANDALONE OPERATION



No laptop may be required after programming.



RADAR-TX must create:



```text

SSID: ESP32-RADAR

```



and host the entire dashboard locally.



A phone/tablet connects directly to the ESP32.



The dashboard should use:



```text

HTTP

WebSocket

compact binary telemetry where useful

```



and should update continuously.



If nobody is connected:



\* radar keeps sensing;

\* RX1 keeps processing;

\* RX2 keeps processing;

\* event detection continues;

\* SD logging continues;

\* calibration remains active.



The laptop is an optional development/debugging instrument only.



\---



\# 11. ESP32-CAM FLASHING — USE THE MIDDLE DEVKIT AS USB-UART ADAPTER



There is no separate USB-to-UART adapter for RADAR-RX2.



Use the middle ESP32 DevKit temporarily as the serial programmer.



During ESP32-CAM flashing, \*\*disable the middle DevKit's own ESP32\*\* by holding its EN pin LOW.



Temporary programming connections:



```text

MIDDLE DEVKIT                 ESP32-CAM



GND ------------------------ GND



5V / VIN ------------------- 5V

&#x20;                             ^

&#x20;                             |

&#x20;                   DO NOT CONNECT 5 V TO 3V3



TX0 / GPIO1 ---------------- U0R / GPIO3



RX0 / GPIO3 ---------------- U0T / GPIO1



GND ------------------------ GPIO0

&#x20;                        \[FLASH MODE ONLY]



EN ---------------- GND

(on middle DevKit)

```



UART lines are crossed logically:



```text

programmer TX → target RX

programmer RX ← target TX

```



\### Flash sequence



1\. Disconnect unnecessary peripherals.

2\. Connect common GND.

3\. Connect middle DevKit `EN` to GND so its ESP32 does not drive UART0.

4\. Connect middle DevKit USB to computer.

5\. Connect:



&#x20;  \* DevKit TX0 → ESP32-CAM U0R;

&#x20;  \* DevKit RX0 ← ESP32-CAM U0T.

6\. Power the ESP32-CAM through its 5 V input.

7\. Connect ESP32-CAM GPIO0 to GND.

8\. Reset/power-cycle ESP32-CAM.

9\. Flash `RADAR-RX2`.

10\. Remove GPIO0 from GND.

11\. Reset/power-cycle ESP32-CAM.

12\. Remove the temporary programming UART wiring when complete.

13\. Release middle DevKit EN from GND.



Do not require these UART wires during normal radar operation.



Ensure the implementation documentation includes a diagram of this.



\---



\# 12. OPTIONAL RP2350



A non-Wi-Fi RP2350 may be added later.



It is NOT necessary for version 1.



If used, it must not alter the physical ESP32 geometry.



Treat it as a wired compute/DSP coprocessor.



Preferred connection to RADAR-TX:



```text

ESP32                  RP2350



GND ------------------ GND

3V3 logic only



TX2 / GPIO17 --------- UART RX

RX2 / GPIO16 <--------- UART TX

```



Alternatively implement SPI if throughput requires it.



Potential RP2350 jobs:



\* FFT/STFT;

\* longer spectrogram buffers;

\* PCA;

\* filtering;

\* classifier inference;

\* data compression;

\* display generation.



Do not give the RP2350 any RF responsibility.



RADAR-TX must remain capable of operating without it.



Implement a versioned coprocessor protocol so the RP2350 can be plugged in later without rewriting the radar network.



\---



\# 13. SOFTWARE ARCHITECTURE



Use ESP-IDF.



Espressif's ESP32 CSI implementation provides CSI configuration/callback support, and the official `esp-csi` repository already contains ESP-to-ESP sender and receiver examples. Use those as validated starting references rather than inventing the low-level CSI capture mechanism from scratch.



Required components:



```text

/components

&#x20;   /radar\_protocol

&#x20;   /radar\_csi

&#x20;   /radar\_dsp

&#x20;   /radar\_features

&#x20;   /radar\_transport

&#x20;   /radar\_calibration

&#x20;   /radar\_storage

&#x20;   /radar\_web

&#x20;   /radar\_ota

&#x20;   /radar\_rp2350



/firmware

&#x20;   /radar\_tx

&#x20;   /radar\_rx



/web

&#x20;   dashboard



/tools

&#x20;   replay

&#x20;   decoder

&#x20;   analysis

&#x20;   test-generator

```



RX1 and RX2 should use the same receiver codebase with compile-time/runtime node configuration.



\---



\# 14. CSI ACQUISITION



Use the documented ESP-IDF CSI flow:



```c

esp\_wifi\_set\_csi\_rx\_cb(...)

esp\_wifi\_set\_csi\_config(...)

esp\_wifi\_set\_csi(...)

```



Keep the CSI callback extremely short.



Do not:



\* FFT;

\* write SD;

\* allocate large buffers;

\* render JSON;

\* run WebSockets;



inside the Wi-Fi callback.



Instead:



```text

CSI CALLBACK

&#x20;     ↓

preallocated frame/ring buffer

&#x20;     ↓

DSP task

&#x20;     ↓

feature extraction

&#x20;     ↓

network telemetry / SD logger

```



Track ring-buffer overflow explicitly.



\---



\# 15. COMMON MEASUREMENT SEQUENCE



Every TX radar packet needs a sequence counter:



```text

0

1

2

3

...

```



Both RX nodes extract it.



Fusion is based on:



```text

sequence n:



TX

&#x20;├── RX1 CSI\[n]

&#x20;└── RX2 CSI\[n]

```



Do not assume RX1 and RX2 have synchronized RF oscillators.



Independent ESP32s are not a coherent phased array.



Use sequence alignment for temporal pairing.



Absolute cross-receiver RF phase must not be treated as coherent unless experimentally proven.



\---



\# 16. SIGNAL PROCESSING



Implement progressively:



```text

raw complex CSI

&#x20;     ↓

valid-subcarrier selection

&#x20;     ↓

amplitude

&#x20;     ↓

normalization

&#x20;     ↓

baseline subtraction

&#x20;     ↓

outlier removal

&#x20;     ↓

temporal filtering

&#x20;     ↓

PCA

&#x20;     ↓

STFT

&#x20;     ↓

per-link features

&#x20;     ↓

RX1/RX2 fusion

&#x20;     ↓

motion/activity output

```



Retain raw CSI recording capability so better algorithms can be developed later without reflashing the sensor.



\---



\# 17. CALIBRATION MUST ASSUME THE COMPACT HEAD FOREVER



Commissioning stages:



\### CAL 1 — hardware identity



Learn:



```text

TX MAC

RX1 MAC

RX2 MAC

```



\### CAL 2 — RF power



Automatically select TX power appropriate for centimetre-scale board separation.



\### CAL 3 — empty environment



Record long baseline CSI.



\### CAL 4 — moving-person test



Ask user to walk around the fixed sensor.



Determine high-information subcarriers and PCA modes.



\### CAL 5 — optional fingerprints



User moves to calibration positions.



\*\*ESP32s never move.\*\*



Save everything in NVS/SD.



Power cycling must not require recalibration unless:



\* boards were physically moved;

\* operating channel changed;

\* calibration is manually reset;

\* environment has changed drastically.



\---



\# 18. FINAL REQUIREMENT



The photograph is not merely an illustration.



It defines the hardware topology.



The final product must therefore look conceptually like:



```text

┌────────────────────────────────────────────┐

│       FIXED COMPACT ESP32 RADAR HEAD       │

│                                            │

│ \[TX DEVKIT]\[RX1 DEVKIT]\[ESP32-CAM RX2]    │

│                                            │

│      exactly one rigid assembly            │

└────────────────────────────────────────────┘



&#x20;                  ↓ 2.4 GHz CSI sensing



&#x20;         surrounding environment



&#x20;                  ↓



&#x20;      local browser / optional SD



&#x20;                  ↓



&#x20;     live CSI + motion visualization

```



Do not optimize the design around moving the nodes farther apart.



Optimize the RF power, DSP, calibration and inference around \*\*the exact compact assembly supplied by the user\*\*.



The objective is to squeeze the maximum physically defensible environmental sensing capability from this permanent three-board arrangement.

