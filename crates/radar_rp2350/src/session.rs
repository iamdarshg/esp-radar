//! Pure link-session helpers (host-testable): protocol version negotiation and
//! the coprocessor-local sequence counter.

/// Lowest wire-protocol version this firmware understands.
pub const MIN_VERSION: u8 = 1;

/// True if the coprocessor's advertised protocol version `remote` is compatible
/// with our local version.
///
/// The point of the versioned protocol (spec §12) is that the RP2350 can be
/// plugged in later — or upgraded independently — without rewriting the radar
/// network. Today both sides speak v1, so we accept exact matches at or above
/// [`MIN_VERSION`]. If a future v2 only *adds* message types this can be
/// relaxed to a range check without touching the RF protocol.
pub fn compatible(local: u8, remote: u8) -> bool {
    remote == local && remote >= MIN_VERSION
}

/// Coprocessor-local sequence counter, independent of the RF `seq`.
///
/// Used to tag outgoing frames so replies can be correlated and so the
/// coprocessor can detect gaps. Wraps at 2^32 — at a 100 Hz push rate that is
/// ~1.3 years between repeats, far longer than a session needs to care.
#[derive(Clone, Debug, Default)]
pub struct Seq(u32);

impl Seq {
    pub fn new() -> Self {
        Self(1)
    }
    /// Next sequence number.
    pub fn next(&mut self) -> u32 {
        let s = self.0;
        self.0 = self.0.wrapping_add(1);
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_matching_version() {
        assert!(compatible(1, 1));
    }

    #[test]
    fn rejects_different_or_older() {
        assert!(!compatible(1, 2), "future version not yet known");
        assert!(!compatible(2, 1), "local newer than remote");
        assert!(!compatible(1, 0), "below minimum");
    }

    #[test]
    fn seq_starts_at_one_and_advances() {
        let mut s = Seq::new();
        assert_eq!(s.next(), 1);
        assert_eq!(s.next(), 2);
        assert_eq!(s.next(), 3);
    }
}
