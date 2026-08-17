#!/usr/bin/env python3
"""Count CSI_PHASE frames in a raw UART1 capture from the QEMU sim run.

The harness gates a sim run on evidence that the firmware actually emitted
phase telemetry on the wired UART. This mirrors the framing in
crates/radar_transport/src/framer.rs + crates/radar_protocol/src/lib.rs
(magic 0x52445231, `#[repr(C, packed)]` header, CRC-16/IBM) closely enough to
validate every candidate frame and count kind==0x16 (CSI_PHASE) records.

The AUTHORITATIVE parse (ground-truth correlation, error metrics) is
`rf-sim analyze <scenario.json> <capture.log>` in tools/rf-sim — that uses the
real Rust framer. This script is only the harness's fast pass/fail gate.

Usage:
    python scripts/csi_phase_count.py <capture.bin>

Prints a one-line summary; exits 0 if at least MIN_FRAMES CSI_PHASE frames
(with valid CRC) were found, else 1.
"""

import struct
import sys

MAGIC = 0x52445231          # "RDR1"
CSI_PHASE = 0x16
HEADER_LEN = 24
MIN_FRAMES = 16             # a handful of frames proves the plane is live

# CRC-16/XMODEM: poly 0x1021, init 0x0000, MSB-first — mirrors
# crates/radar_protocol/src/crc.rs exactly. Check value for b"123456789"
# is 0x31C3 (the crate's own test vector).
def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
    return crc & 0xFFFF


def count(path: str):
    data = open(path, "rb").read()
    n_csi = 0
    n_other = 0
    n_bad_crc = 0
    n_magic = 0
    first_seq = None
    last_seq = None
    i = 0
    while i < len(data):
        # Hunt for the magic.
        start = data.find(struct.pack("<I", MAGIC), i)
        if start < 0:
            break
        n_magic += 1
        hdr = data[start:start + HEADER_LEN]
        if len(hdr) < HEADER_LEN:
            break
        # Packed struct: magic u32 | ver u8 | kind u8 | src u8 | dst u8 |
        # seq u32 | t_us u64 | payload_len u16 | crc16 u16 (24 bytes LE).
        magic, ver, kind, src, dst, seq, t_us, payload_len, stored_crc = \
            struct.unpack("<IBBBBIQHH", hdr)
        if magic != MAGIC or ver != 2:
            i = start + 1
            continue
        frame_end = start + HEADER_LEN + payload_len
        if frame_end > len(data):
            break  # truncated tail (QEMU may have stopped mid-frame)
        # CRC over header (crc16 zeroed) + payload.
        crc_hdr = bytearray(hdr[:22]) + b"\x00\x00"
        body = data[start + HEADER_LEN:frame_end]
        if crc16(bytes(crc_hdr) + body) != stored_crc:
            n_bad_crc += 1
            i = start + 1
            continue
        if kind == CSI_PHASE:
            n_csi += 1
            first_seq = seq if first_seq is None else first_seq
            last_seq = seq
        else:
            n_other += 1
        i = frame_end

    print(f"{path}: magic={n_magic} csi_phase={n_csi} other={n_other} "
          f"bad_crc={n_bad_crc} seq={first_seq}..{last_seq}")
    return 0 if n_csi >= MIN_FRAMES else 1


if __name__ == "__main__":
    sys.exit(count(sys.argv[1]))
