//! Wired UART link to RADAR-TX — the inter-board data plane.
//!
//! Carries everything the RX used to unicast back over WiFi UDP: RATE-2
//! `FeatureReport`s, `CalResp`s and `CsiSnapshot`s **up** to RADAR-TX, and
//! `CalCmd`s **down** from it. The measurement plane (RATE-1 WiFi
//! `DataFrame`s) stays on the radio — this link only replaces the *reports*,
//! so the RX board stops transmitting on the 2.4 GHz sensing band entirely
//! (spec §15).
//!
//! Frames use the shared `radar_protocol` "RDR1" wire format and are pulled
//! out of the byte stream by [`RadarFrameDecoder`] (magic hunt + CRC resync),
//! so the exact bytes the old UDP path carried now cross a 2-wire UART.

use esp_idf_hal::delay::TickType;
use esp_idf_hal::gpio::{Gpio0, InputPin, OutputPin};
use esp_idf_hal::uart::{self, UART1, UartDriver};
use esp_idf_hal::units::Hertz;
use radar_protocol::{
    CalResp, CsiSnapshot, FeatureReport, HEADER_SIZE, MAX_PAYLOAD, N_SUBCARRIERS,
};
use radar_transport::framer::{RadarFrame, RadarFrameDecoder, MAX_FRAME};
use radar_transport::udp::now_us;
use radar_transport::{
    build_cal_resp, build_csi_phase, build_csi_snapshot, build_feature_report,
};

/// Link baud. 460800 puts a full CSI snapshot (~380 B) on the wire in well
/// under 10 ms; drop to 230400 if the physical build shows CRC errors.
pub const DEFAULT_BAUD: u32 = 460_800;
/// Per-read blocking window while polling for inbound frames. Short so the
/// radar loop's UDP recv timeout still paces the cycle. Sim mode drops this
/// to 0 (non-blocking): nothing arrives on the wire in sim mode, so blocking
/// the full window every iteration is pure dead time that halves the frame
/// throughput QEMU can sustain.
const READ_POLL_MS: u64 = 5;

/// UART1 link to RADAR-TX: reports/snapshots/CAL_RESP up, CAL_CMD down.
pub struct WiredLink {
    uart: UartDriver<'static>,
    decoder: RadarFrameDecoder,
    /// Blocking window for each inbound read (see [`READ_POLL_MS`]).
    poll_ms: u64,
}

impl WiredLink {
    /// Open UART1 on the role's crossed pins at [`DEFAULT_BAUD`]. The software
    /// RX FIFO is sized for a full frame plus a burst so a snapshot can't
    /// overflow between polls (the 128-byte hardware FIFO sits underneath it).
    pub fn open(
        uart1: UART1<'static>,
        tx: impl OutputPin + 'static,
        rx: impl InputPin + 'static,
    ) -> Result<Self, esp_idf_sys::EspError> {
        let config = uart::config::Config::new()
            .baudrate(Hertz(DEFAULT_BAUD))
            .rx_fifo_size(2 * (HEADER_SIZE + MAX_PAYLOAD));
        let uart = UartDriver::new(uart1, tx, rx, None::<Gpio0>, None::<Gpio0>, &config)?;
        Ok(Self {
            uart,
            decoder: RadarFrameDecoder::new(),
            poll_ms: READ_POLL_MS,
        })
    }

    /// Override the inbound-poll blocking window. Sim mode uses 0 so the radar
    /// loop never idles waiting for a wire that has nothing to send it.
    pub fn set_poll_ms(&mut self, ms: u64) {
        self.poll_ms = ms;
    }

    /// Read whatever has arrived and return complete, CRC-valid frames. Never
    /// blocks more than [`READ_POLL_MS`].
    pub fn poll(&mut self) -> Vec<RadarFrame> {
        let mut out = Vec::new();
        loop {
            if let Some(frame) = self.decoder.next() {
                out.push(frame);
                continue;
            }
            let mut chunk = [0u8; 128];
            match self.read_chunk(&mut chunk) {
                Ok(n) if n > 0 => self.decoder.feed(&chunk[..n]),
                _ => break,
            }
        }
        out
    }

    /// Send a RATE-2 `FeatureReport` to RADAR-TX.
    pub fn send_feature_report(
        &mut self,
        src: u8,
        report: &FeatureReport,
    ) -> Result<(), esp_idf_sys::EspError> {
        let mut buf = [0u8; MAX_FRAME];
        let n = build_feature_report(&mut buf, src, report, now_us());
        self.uart.write(&buf[..n])?;
        Ok(())
    }

    /// Send a calibration response to RADAR-TX.
    pub fn send_cal_resp(
        &mut self,
        src: u8,
        resp: &CalResp,
    ) -> Result<(), esp_idf_sys::EspError> {
        let mut buf = [0u8; MAX_FRAME];
        let n = build_cal_resp(&mut buf, src, resp, now_us());
        self.uart.write(&buf[..n])?;
        Ok(())
    }

    /// Send a low-rate `CsiSnapshot` to RADAR-TX (waterfall/spectrogram source).
    pub fn send_csi_snapshot(
        &mut self,
        src: u8,
        snap: &CsiSnapshot,
    ) -> Result<(), esp_idf_sys::EspError> {
        let mut buf = [0u8; MAX_FRAME];
        let n = build_csi_snapshot(&mut buf, src, snap, now_us());
        self.uart.write(&buf[..n])?;
        Ok(())
    }

    /// Send a full-rate RAW per-subcarrier phase frame (sim-mode telemetry for
    /// the RF-sim analyzer). `seq`/`t_us` live in the header and MUST be
    /// producer-stamped: `seq` = the scenario frame index, `t_us` = the push
    /// instant — never consume-time.
    pub fn send_csi_phase(
        &mut self,
        src: u8,
        seq: u32,
        t_us: u64,
        phase: &[i16; N_SUBCARRIERS],
    ) -> Result<(), esp_idf_sys::EspError> {
        let mut buf = [0u8; MAX_FRAME];
        let n = build_csi_phase(&mut buf, src, seq, t_us, phase);
        self.uart.write(&buf[..n])?;
        Ok(())
    }

    fn read_chunk(&self, buf: &mut [u8]) -> Result<usize, esp_idf_sys::EspError> {
        self.uart.read(buf, TickType::new_millis(self.poll_ms).ticks())
    }
}
