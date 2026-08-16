# Host tools

Four small host-side binaries live under `tools/` for working with radar data
off the device. They are plain Cargo binaries (no ESP hardware) — they build and
run with the host toolchain, so no `esp-env.sh` is needed.

They share a common input convention: **a raw captured byte stream** — the exact
bytes that went over the wire, frames back-to-back (a file, or stdin when no
path is given). The three stream-consuming tools scan for the `"RDR1"` frame
magic, validate CRCs, and resync after corrupt frames, so captures with gaps
or partial frames still work.

Build and run from the repo root (the tools are workspace members, so `-p`
works; unknown flags print the usage line):

```bash
cargo run -p decoder -- capture.bin     # or: cargo run -p analysis -- ...
cargo run -p replay -- ...
cargo run -p test-generator -- ...
```

| Tool | Purpose |
|------|---------|
| `tools/decoder` | Turn a captured byte stream into human-readable frame text or JSON. |
| `tools/analysis` | Aggregate statistics over a capture: rates, per-kind/per-link counts, RSSI, packet loss, motion summaries. |
| `tools/replay` | Re-send a capture over UDP, preserving inter-frame timing, to exercise the live pipeline without hardware. |
| `tools/test-generator` | Synthesize a deterministic radar stream (data frames, feature reports, CSI snapshots) to stdout or a UDP target. |

## decoder

```
usage: decoder [--json] [--tolerate] [capture-file]
```

Reads a raw captured byte stream and prints every `radar_protocol` frame as
human-readable text: decoded header fields (kind, src/dst node, seq, t_us),
CRC validity, and a field dump for the known payload kinds (data, feature
report, CSI snapshot, calibration, status). Unknown/partial data is reported as
`BAD FRAME` / `TRUNCATED`.

* `--json` — emit one JSON object per frame on stdout (diagnostics go to
  stderr).
* `--tolerate` — exit 0 even if CRC/version errors or unknown frame kinds were
  seen. Without it, the tool exits nonzero if anything was malformed.

Examples:

```bash
cargo run -p decoder -- capture.bin
cargo run -p decoder -- --json capture.bin
cat capture.bin | cargo run -p decoder -- --tolerate
```

## analysis

```
usage: analysis [--csv <path>] [capture-file]
```

Reads a raw captured byte stream and prints summary statistics to stdout:

* total frames, time span, frames/second;
* per-kind counts and per-source (node) counts;
* CRC/version error count and truncated-frame count;
* RSSI samples (min/max/mean) from feature reports and CSI snapshots;
* a **packet-loss estimate** per source from sequence-number gaps
  (using `radar_transport::SequenceTracker`: lost, total, ratio, gaps,
  resyncs);
* `motion_energy` and `dominant_freq_hz` summaries from feature reports.

* `--csv <path>` — additionally write a per-frame CSV
  (`seq,t_us,kind,src,dst,rssi,snr,motion_energy,amp_mean,dominant_freq_hz`) to
  `<path>`; fields a frame kind does not carry are left empty.

Example:

```bash
cargo run -p analysis -- --csv out.csv capture.bin
```

## replay

```
usage: replay [--host <ip>] [--port <u16>] [--fast] [--loop] [capture-file]
```

Validates and extracts every intact frame from the input, then re-sends each
frame's **original bytes** as a UDP datagram to `host:port`. Inter-frame timing
is preserved from the frames' `t_us` header timestamps, so the live pipeline
sees the same pacing as the original capture.

* Default target: RADAR-TX's AP address at the **report port** (where TX
  listens for RX feature reports) — i.e. `192.168.4.1:4445`. Override with
  `--host` / `--port`.
* `--fast` — send as fast as possible (ignore inter-frame timing).
* `--loop` — repeat the stream forever, keeping timing monotonic across
  iterations.

Example:

```bash
cargo run -p replay -- capture.bin            # to 192.168.4.1:4445, realtime
cargo run -p replay -- --fast --host 127.0.0.1 --port 4445 capture.bin
```

## test-generator

```
usage: test-generator [--kind data|feature|snapshot|mixed] [--rate <hz>]
                      [--frames <n>] [--seed <u64>] [--host <ip>] [--port <u16>]
```

Synthesizes radar frames — measurement (`DataPayload`) frames, RX
`FeatureReport`s and low-rate `CsiSnapshot`s — with correct CRCs, monotonic
sequence numbers and timestamps, and deterministic pseudorandom CSI data. The
stream is written to **stdout** as raw frame bytes, or sent over **UDP** when
`--host`/`--port` are given (for exercising the live pipeline and dashboard
without hardware).

* `--kind` — one of `data`, `feature`, `snapshot`, or `mixed` (default
  `mixed`: a data frame every tick, a feature report every 50 ticks, a snapshot
  every 200).
* `--rate` — frame rate in Hz (default 200; used for timestamps and UDP
  pacing).
* `--frames` — number of frames to generate (default 1000).
* `--seed` — PRNG seed; the same seed reproduces the exact same stream.

Examples:

```bash
cargo run -p test-generator -- --frames 500 > capture.bin
cargo run -p test-generator -- --kind feature --host 192.168.4.1 --port 4445 --rate 10
```

A natural workflow: `test-generator` produces a capture, `replay` re-sends it
into the live pipeline, and `decoder`/`analysis` inspect the result (or a
capture taken on the device).

## Working with real on-device captures

The tools read the raw wire bytes. To get a capture off the device, record the
byte stream from either direction (for example via the UDP target ports used by
the firmware, or by capturing the dashboard telemetry) — the exact byte layout
is the `radar_protocol` wire format documented in `crates/radar_protocol/src/lib.rs`.
