//! UART2 transport for the RP2350 coprocessor (`feature = "device"`).
//!
//! Wires the ESP32's UART2 (GPIO17 = TX → RP2350 RX, GPIO16 = RX ← RP2350 TX)
//! to the [`radar_protocol::cp`] protocol. **Best-effort by design**: every
//! operation returns cleanly even when no coprocessor is connected, so RADAR-TX
//! never blocks, panics, or degrades because of it (spec §12).

use esp_idf_hal::delay::TickType;
use esp_idf_hal::gpio::{Gpio16, Gpio17};
use esp_idf_hal::uart::{self, UartDriver};
use esp_idf_hal::units::Hertz;
use radar_protocol::cp;

use crate::framer::{Frame, FrameDecoder, MAX_FRAME};
use crate::session::{compatible, Seq};

/// Default link baud rate. 921600 keeps a full 1 KiB payload under ~9 ms on
/// the wire, so pushing telemetry is effectively non-blocking.
pub const DEFAULT_BAUD: u32 = 921_600;
/// Per-read blocking window while polling for a reply.
const READ_POLL_MS: u64 = 25;
/// How long to wait for the coprocessor to answer HELLO before declaring it
/// absent.
pub const DEFAULT_PROBE_TIMEOUT_MS: u64 = 500;

/// Link-level errors. All are recoverable; RADAR-TX treats the link as absent
/// after any of them.
#[derive(Debug)]
pub enum CoError {
    /// No coprocessor answered, or the link is not usable.
    NotPresent,
    /// UART driver failure.
    Uart(esp_idf_sys::EspError),
    /// The peer replied with an incompatible protocol version.
    Version(u8),
    /// No reply arrived within the timeout.
    Timeout,
    /// The peer answered but the payload was not well-formed.
    BadPayload,
}

impl core::fmt::Display for CoError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CoError::NotPresent => write!(f, "coprocessor not present"),
            CoError::Uart(e) => write!(f, "uart error: {e}"),
            CoError::Version(v) => write!(f, "coprocessor protocol version {v} unsupported"),
            CoError::Timeout => write!(f, "coprocessor did not answer in time"),
            CoError::BadPayload => write!(f, "coprocessor payload malformed"),
        }
    }
}

/// Outcome of a [`Link::probe`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CoState {
    /// The coprocessor answered HELLO_ACK; the link is usable.
    Present { caps: u16, fw_version: u32 },
    /// No coprocessor (or it stayed silent). RADAR-TX continues unaffected.
    Absent,
}

/// Owned UART2 link to the RP2350.
pub struct Link {
    uart: UartDriver<'static>,
    decoder: FrameDecoder,
    seq: Seq,
    present: bool,
}

impl Link {
    /// Open UART2 on the fixed GPIO17-TX / GPIO16-RX pins at [`DEFAULT_BAUD`].
    ///
    /// The pins must be `'static`: `UartDriver` borrows them for its whole
    /// lifetime, and `Link` hands the driver to a `'static` task, so an elided
    /// (short) pin lifetime would not outlive it.
    pub fn new(
        uart2: esp_idf_hal::uart::UART2<'static>,
        tx: Gpio17<'static>,
        rx: Gpio16<'static>,
    ) -> Result<Self, esp_idf_sys::EspError> {
        let config = uart::config::Config::new()
            .baudrate(Hertz(DEFAULT_BAUD))
            // Sized for the largest frame (header + 1 KiB payload) plus margin,
            // so a coprocessor burst never overflows the software FIFO before
            // the link loop reads it.
            .rx_fifo_size(2 * cp::MAX_PAYLOAD);
        let uart = UartDriver::new(uart2, tx, rx, None::<Gpio17>, None::<Gpio17>, &config)?;
        Ok(Self {
            uart,
            decoder: FrameDecoder::new(),
            seq: Seq::new(),
            present: false,
        })
    }

    /// Best-effort presence probe: send HELLO, wait for HELLO_ACK. Any outcome
    /// short of a *compatible* HELLO_ACK ⇒ [`CoState::Absent`]. Never fails the
    /// caller. Call once at startup (or on a timer to notice a coprocessor
    /// plugged in later).
    pub fn probe(&mut self, timeout_ms: u64) -> CoState {
        self.send_empty(cp::msg_type::HELLO);
        match self.wait_for(cp::msg_type::HELLO_ACK, timeout_ms) {
            Ok(Some(frame)) => {
                let remote_version = frame.header.version;
                if !compatible(cp::VERSION, remote_version) {
                    log::warn!(
                        "coprocessor protocol version {remote_version} incompatible (we speak {})",
                        cp::VERSION
                    );
                    self.present = false;
                    return CoState::Absent;
                }
                match parse_ack(&frame.payload) {
                    Some(ack) => {
                        self.present = true;
                        // `HelloAck` is a packed struct; copy the fields out before
                        // formatting (E0793: references to packed fields are unaligned).
                        let caps = ack.caps;
                        let fw_version = ack.fw_version;
                        log::info!(
                            "RP2350 coprocessor present: caps={caps:04x} fw={fw_version:08x}"
                        );
                        CoState::Present { caps, fw_version }
                    }
                    None => {
                        log::warn!("coprocessor HELLO_ACK payload too short");
                        self.present = false;
                        CoState::Absent
                    }
                }
            }
            _ => {
                self.present = false;
                CoState::Absent
            }
        }
    }

    /// Whether the last probe succeeded and the link is usable.
    pub fn is_present(&self) -> bool {
        self.present
    }

    /// Push a payload to the coprocessor (best-effort, fire-and-forget). No
    /// reply is required; if nobody is listening the bytes just land in the
    /// (empty) UART FIFO and are lost. Returns `Ok(())` even when the coprocessor
    /// is absent — the caller should check [`is_present`](Self::is_present) only
    /// to skip needless work.
    pub fn push(&mut self, msg_type: u8, payload: &[u8]) -> Result<(), CoError> {
        if payload.len() > cp::MAX_PAYLOAD {
            return Err(CoError::BadPayload);
        }
        let seq = self.seq.next();
        let hdr = cp::Header {
            magic: cp::MAGIC,
            version: cp::VERSION,
            msg_type,
            flags: 0,
            reserved: 0,
            seq,
            payload_len: payload.len() as u16,
            crc16: 0,
        };
        let mut buf = [0u8; MAX_FRAME];
        let n = cp::build(&mut buf, &hdr, payload);
        // SAFETY of cp::build: MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD ≥ written size.
        self.uart.write(&buf[..n]).map_err(CoError::Uart)?;
        Ok(())
    }

    /// Ask the coprocessor for its STATUS (a lightweight heartbeat / health
    /// check). Returns the parsed status, or `None` if it stayed silent.
    pub fn poll_status(&mut self, timeout_ms: u64) -> Result<Option<cp::Status>, CoError> {
        self.send_empty(cp::msg_type::STATUS);
        Ok(match self.wait_for(cp::msg_type::STATUS, timeout_ms)? {
            Some(frame) => parse_status(&frame.payload),
            None => None,
        })
    }

    /// Drain any inbound frames within `timeout_ms` (e.g. unsolicited ERRORs).
    /// Returns them for the caller to act on or log.
    pub fn drain(&mut self, timeout_ms: u64) -> Vec<Frame> {
        let deadline = now_ms().saturating_add(timeout_ms);
        let mut out = Vec::new();
        loop {
            if let Some(frame) = self.decoder.next() {
                out.push(frame);
                continue;
            }
            if now_ms() >= deadline {
                break;
            }
            let mut chunk = [0u8; 128];
            match self.read_chunk(&mut chunk) {
                Ok(n) if n > 0 => self.decoder.feed(&chunk[..n]),
                Err(e) => {
                    log::warn!("coprocessor uart read failed: {e:?}");
                    break;
                }
                Ok(_) => {}
            }
        }
        out
    }

    // -- internals -----------------------------------------------------------

    /// Send a header-only frame (HELLO, STATUS ping). Send errors are ignored —
    /// best-effort by design.
    fn send_empty(&mut self, msg_type: u8) {
        let seq = self.seq.next();
        let hdr = cp::Header {
            magic: cp::MAGIC,
            version: cp::VERSION,
            msg_type,
            flags: 0,
            reserved: 0,
            seq,
            payload_len: 0,
            crc16: 0,
        };
        let mut buf = [0u8; cp::HEADER_SIZE];
        let n = cp::build(&mut buf, &hdr, &[]);
        let _ = self.uart.write(&buf[..n]);
    }

    /// Blocking UART read with a short poll window; returns bytes read.
    fn read_chunk(&self, buf: &mut [u8]) -> Result<usize, esp_idf_sys::EspError> {
        self.uart
            .read(buf, TickType::new_millis(READ_POLL_MS).ticks())
    }

    /// Keep reading until a frame with `wanted` kind arrives or `timeout_ms`
    /// elapses. Frames of other kinds are drained and discarded.
    fn wait_for(&mut self, wanted: u8, timeout_ms: u64) -> Result<Option<Frame>, CoError> {
        let deadline = now_ms().saturating_add(timeout_ms);
        loop {
            if let Some(frame) = self.decoder.next() {
                if frame.kind() == wanted {
                    return Ok(Some(frame));
                }
                continue;
            }
            if now_ms() >= deadline {
                return Ok(None);
            }
            let mut chunk = [0u8; 128];
            match self.read_chunk(&mut chunk) {
                Ok(n) if n > 0 => self.decoder.feed(&chunk[..n]),
                Err(e) => return Err(CoError::Uart(e)),
                Ok(_) => {}
            }
        }
    }
}

/// Decode a HELLO_ACK payload into a [`cp::HelloAck`] (packed → by value).
fn parse_ack(payload: &[u8]) -> Option<cp::HelloAck> {
    if payload.len() < core::mem::size_of::<cp::HelloAck>() {
        return None;
    }
    // SAFETY: length checked above; read unaligned into an owned value.
    Some(unsafe { (payload.as_ptr() as *const cp::HelloAck).read_unaligned() })
}

/// Decode a STATUS payload into a [`cp::Status`] (packed → by value).
fn parse_status(payload: &[u8]) -> Option<cp::Status> {
    if payload.len() < core::mem::size_of::<cp::Status>() {
        return None;
    }
    // SAFETY: length checked above; read unaligned into an owned value.
    Some(unsafe { (payload.as_ptr() as *const cp::Status).read_unaligned() })
}

/// Milliseconds since boot, from the monotonic ESP timer.
fn now_ms() -> u64 {
    // SAFETY: esp_timer_get_time is always valid on-device; returns i64 µs.
    unsafe { esp_idf_sys::esp_timer_get_time() as u64 / 1000 }
}
