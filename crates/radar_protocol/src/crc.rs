//! CRC-16/XMODEM (poly 0x1021, init 0x0000) used by all radar frames.

/// Compute CRC-16/XMODEM over `data`.
pub fn crc16(data: &[u8]) -> u16 {
    crc16_ext(0, data)
}

/// Extend a running CRC-16/XMODEM value over `data`. Enables checksumming a
/// frame in multiple chunks (header + payload).
pub fn crc16_ext(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xmodem_vector() {
        // Standard CRC-16/XMODEM check value.
        assert_eq!(crc16(b"123456789"), 0x31c3);
    }

    #[test]
    fn incremental_matches_whole() {
        let data = b"the quick brown fox jumps over the lazy dog";
        let whole = crc16(data);
        let mut inc = crc16(&data[..10]);
        inc = crc16_ext(inc, &data[10..]);
        assert_eq!(inc, whole);
    }
}
