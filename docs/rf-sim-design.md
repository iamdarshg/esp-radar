# RF sensing-chain simulator — design (QEMU-integrated, phase-coherent)

Goal: quantify **phase error, position/displacement error, velocity error** of the
ESP-RADAR sensing head from **what the firmware actually outputs**, not from a
re-implementation of the DSP. The synthetic RF channel is injected into the real
RX firmware running under QEMU; error analysis runs on the real CSI → DSP →
telemetry output.

## Why phase (and why this is an actual radar)

A CW radar senses motion through **coherent phase**: a target displacement Δr
changes the received phase by Δφ = 4π·Δr/λ (round-trip reflection; λ ≈ 12.5 cm at
2.4 GHz → 1 mm ≈ 0.10 rad). Velocity appears as the phase rate dφ/dt (Doppler).
The current firmware is amplitude-only presence detection: it can say "something
moved" but cannot measure displacement or velocity, and amplitude fading is
non-linear. This simulator models and measures the **phase-based** radar the
hardware can already produce.

Key existing capability (verified by reading the code):

- `radar_dsp::transform::decode_channel` already computes per-subcarrier
  `phase[i] = atan2(im, re)` over 56 HT20 subcarriers from the raw interleaved
  i8 I/Q, then `sanitize_phase` removes the linear-across-subcarrier slope.
- The `CsiSnapshot` already carries reconstructed I/Q from the sanitized phase.
- `phase_dispersion` (circular variance) is the only phase aggregate in the
  FeatureReport; phase is NOT in the motion path (band-pass + PCA run on
  amplitude).

**Critical caveat (drives the design):** `sanitize_phase` removes the linear
phase slope across subcarriers, but that slope IS the dominant-path range signal
(`φ_k = -2π·(f_c + k·Δf)·τ`, slope = -2π·Δf·τ). So a single dominant path has
zero sanitized residual, and range/Doppler of the dominant path live in the
*raw* phase. The simulator therefore carries the **raw** phase (new `CSI_PHASE`
telemetry) as the radar observable, and quantifies what the sanitize step
discards.

## Hardware / geometry

Rigid 3-board head (fixed, photo-final): TX antenna (middle), RX1 (left,
rotated), RX2/CAM (right, rotated). Board spacing ~cm. TX broadcasts HT20
measurement frames at 200 Hz on channel 6 (2.442 GHz center). A human target is
at range r ≈ 0.5–5 m. Direct TX→RX path (static), one dominant moving path
TX→target→RX (the body reflection), plus static room multipath clutter.

## Channel model (host, deterministic, ground-truth-known)

Per RX board b ∈ {RX1, RX2}, per packet n (t_n = n/200 s), per subcarrier k
(center f_c, offset k·Δf, Δf = 312.5 kHz):

```
H_{b,k}(n) = A_direct·exp(-j·2π·f_k·τ_direct)                     [static direct]
           + A_clutter·Σ_c exp(-j·2π·f_k·τ_c)                      [static multipath]
           + A_t·sqrt(G_b(θ))·exp(-j·2π·f_k·τ_t(n))                [moving target path]
           + n_I + j·n_Q                                            [thermal/SNR]
```

- τ_t(n) = (d_1(n) + d_2(n))/c — TX→target→RX path delay; d_1, d_2 from the
  target position; target moves at velocity v(t) → dτ/dt = 2·v_rad/c →
  Doppler f_d = -2·f_c·v_rad/c (the factor 2 from the reflected path).
- n_I, n_Q ~ N(0, σ²), σ from per-subcarrier SNR (target SNR sweepable).
- **Per-board CFO** ε_b: constant oscillator offset → phase ramp across packets
  of 2π·ε_b·t_n. `sanitize_phase` (per-packet linear fit in k) removes the
  across-subcarrier part; the residual CFO that leaks into the phase-rate is a
  measured velocity bias.
- **PLL phase noise**: Wiener process, random walk per packet, σ_φ_pn^2 growing
  linearly in time (parameter, sweepable).
- **Quantization**: ESP32 CSI delivers i8 I/Q → round(Re,Im to i8) → the
  firmware's i8 → f32 decode. Phase quantization error ≈ q/(√12·A) per
  component (q = 1 LSB).

Ground truth recorded per (b, k, n): raw phase φ_true, sanitized residual,
range r(n), velocity v(n), plus the true CFO and phase-noise realization
(seeded PRNG; host re-generates the identical noise realization to compare).

## Firmware changes (radar_rx)

1. **Sim CSI source** (`sim.rs`): in sim mode, a 200 Hz timer (esp_timer) reads
   the next packet's I/Q blob from a `simdata` flash partition and pushes a
   `CsiFrame { info, buf }` into the real `CsiRing` — identical metadata path to
   the WiFi callback (mac = configured AP BSSID, rssi, noise_floor,
   first_word_invalid). WiFi bring-up is skipped in sim mode (QEMU has no radio).
2. **Raw-phase telemetry**: per-packet, compute raw phase from the ring I/Q
   (real `Complex::phase`), emit a new `CSI_PHASE` frame over the wired UART at
   200 Hz (i16 rad fixed-point ×1000 per subcarrier; payload 112 B + 24 header =
   136 B × 200 = 27.2 KB/s ≈ 59% of the 460800-baud byte budget). Sim-mode only.
3. The **existing** report/snapshot flow runs unchanged (real Pipeline::process,
   spectral(), make_snapshot()).

## Protocol changes

- `frame_type::CSI_PHASE = 0x16`, `CsiPhase { seq: u32, t_us: u32, n_sub: u8,
  phase: [i16; 56] }` (raw per-subcarrier phase, radians×1000, i16). Produced
  only in sim mode.

## QEMU harness

- RX images carry a `simdata` partition (offset 0x300000, 512 KB) holding the
  scenario I/Q blob; harness merges it at image build.
- RX boards boot in sim mode, run the real loop for the scenario duration, emit
  CSI_PHASE + reports + snapshots to their UART logs. TX boots as today.
- Harness waits for the expected phase-frame count, stops boards, hands logs to
  the analyzer.
- (Stretch) connect RX/TX UARTs across machines with QEMU chardev socket pairs
  so the data plane is truly inter-board.

## Error analysis (host, on real firmware output)

Analytic floors (for comparison) and empirical estimates (from the firmware
output):

- **Phase error σφ**: per subcarrier, measured firmware phase vs ground truth.
  Floors: Cramér-Rao σφ_CRB = √(1/(2·SNR_lin)); quantization floor;
  phase-noise walk; CFO residual.
- **Position / displacement error**: σ_d = σφ·λ/(4π); minimum detectable
  displacement at the coherence window; range estimate from the across-
  subcarrier slope and its error.
- **Velocity error**: least-squares phase-rate fit over a sliding window of N
  packets at PRF; σ_v vs true v. Floors: slope-fit variance
  σ_{dφ/dt} = σφ·√(12/(Δt²·N·(N²−1))) → σ_v = σ_{dφ/dt}·c/(4π·f_c);
  Doppler resolution Δv = λ/(2·T_obs); unambiguous limit v_max = λ·PRF/4
  (= 6.25 m/s at 200 Hz).
- **What amplitude-only loses**: motion_energy (real pipeline output) vs
  ground-truth motion; the SNR at which amplitude fails but phase still works.

## Deliverables

- `tools/rf-sim/` (standalone workspace, mirrors tools/em-sim): `scenario`
  generator + `analyze` + analytic floors + report.
- Firmware sim mode + CSI_PHASE; QEMU harness extension; end-to-end report of
  σφ, σ_d, σ_v with sweeps (SNR/distance, PRF, window, velocity, CFO, clutter).
