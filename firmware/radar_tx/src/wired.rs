//! Wired UART links to the two RADAR-RX boards — the inter-board data plane.
//!
//! Each RX board reports over a crossed 2-wire UART instead of WiFi unicast:
//! `FeatureReport`s, `CalResp`s and `CsiSnapshot`s come **up** on these links,
//! and calibration `CalCmd`s go **down**. The measurement plane (RATE-1 WiFi
//! `DataFrame`s) is untouched — this only replaces the board-to-board bursts
//! so neither RX transmits on the 2.4 GHz sensing band (spec §15).
//!
//! [`WiredLink`] is not generic over the UART peripheral: `UartDriver` erases
//! it at construction (the UART type is a `new`-level generic), so one struct
//! serves both links — UART1 to RADAR-RX1 (GPIO18/19) and UART2 to
//! RADAR-RX2/CAM (GPIO17/16).

use esp_idf_hal::delay::TickType;
use esp_idf_hal::gpio::{Gpio0, InputPin, OutputPin};
use esp_idf_hal::uart::{self, UartDriver};
use esp_idf_hal::units::Hertz;
use radar_protocol::{CalCmd, Header, HEADER_SIZE, MAX_PAYLOAD, frame_type, node};
use radar_transport::framer::{RadarFrame, RadarFrameDecoder, MAX_FRAME};
use radar_transport::udp::now_us;

/// Link baud (must match the RX side).
pub const DEFAULT_BAUD: u32 = 460_800;
/// Per-read blocking window while polling for inbound frames.
const READ_POLL_MS: u64 = 5;

/// One wired link to an RX board.
pub struct WiredLink {
    uart: UartDriver<'static>,
    decoder: RadarFrameDecoder,
    /// Header sequence for outbound `CalCmd`s (traceable in RX logs).
    seq: u32,
}

impl WiredLink {
    /// Open a UART on the given crossed pins at [`DEFAULT_BAUD`]. The software
    /// RX FIFO is sized for a full frame plus a burst so a snapshot can't
    /// overflow between polls.
    pub fn open(
        uart: impl esp_idf_hal::uart::Uart + 'static,
        tx: impl OutputPin + 'static,
        rx: impl InputPin + 'static,
    ) -> Result<Self, esp_idf_sys::EspError> {
        let config = uart::config::Config::new()
            .baudrate(Hertz(DEFAULT_BAUD))
            .rx_fifo_size(2 * (HEADER_SIZE + MAX_PAYLOAD));
        let uart = UartDriver::new(uart, tx, rx, None::<Gpio0>, None::<Gpio0>, &config)?;
        Ok(Self {
            uart,
            decoder: RadarFrameDecoder::new(),
            seq: 0,
        })
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

    /// Send a calibration command down the link (fire-and-forget).
    pub fn send_cal_cmd(
        &mut self,
        stage: u8,
        action: u8,
        collect_ms: u32,
        tx_power_db: i16,
    ) -> Result<(), esp_idf_sys::EspError> {
        let cmd = CalCmd { stage, action, collect_ms, tx_power_db };
        let pl = unsafe {
            core::slice::from_raw_parts(
                (&cmd as *const CalCmd) as *const u8,
                core::mem::size_of::<CalCmd>(),
            )
        };
        let hdr = Header::new(frame_type::CAL_CMD, node::TX, 0, self.seq, now_us(), pl.len() as u16);
        self.seq = self.seq.wrapping_add(1);
        let mut buf = [0u8; MAX_FRAME];
        let n = radar_protocol::build(&mut buf, &hdr, pl);
        self.uart.write(&buf[..n])?;
        Ok(())
    }

    fn read_chunk(&self, buf: &mut [u8]) -> Result<usize, esp_idf_sys::EspError> {
        self.uart
            .read(buf, TickType::new_millis(READ_POLL_MS).ticks())
    }
}
