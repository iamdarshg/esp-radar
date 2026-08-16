//! Best-effort RP2350 coprocessor link (spec §12).
//!
//! The coprocessor is optional: RADAR-TX must run perfectly well with nothing
//! on UART2. This task probes for it, then keeps a STATUS heartbeat going and
//! drains any unsolicited frames. Any failure just logs and re-probes later —
//! it never blocks or fails the radar.

use std::time::{Duration, Instant};

use esp_idf_hal::gpio::{Gpio16, Gpio17};
use esp_idf_hal::uart::UART2;
use radar_protocol::cp;
use radar_rp2350::link::{DEFAULT_PROBE_TIMEOUT_MS, Link};
use radar_rp2350::link::CoState;

/// Re-probe cadence while the coprocessor is absent (it may be plugged in
/// later, or boot slower than the ESP).
const RETRY_DELAY: Duration = Duration::from_secs(1);
/// STATUS heartbeat period while the link is up.
const HEARTBEAT_PERIOD: Duration = Duration::from_secs(1);
/// Loop pause (the UART reads are also polled inside `drain`).
const LOOP_PAUSE: Duration = Duration::from_millis(100);

pub fn run(uart2: UART2<'static>, tx: Gpio17<'static>, rx: Gpio16<'static>) {
    let mut link = match Link::new(uart2, tx, rx) {
        Ok(l) => l,
        Err(e) => {
            log::warn!("coprocessor UART init failed: {e}; continuing without it");
            // Nothing left to do — park forever.
            loop {
                std::thread::sleep(Duration::from_secs(3600));
            }
        }
    };

    let mut next_probe = Instant::now();
    let mut next_heartbeat = Instant::now();

    loop {
        std::thread::sleep(LOOP_PAUSE);

        // (Re)establish the link until it answers.
        if !link.is_present() && Instant::now() >= next_probe {
            match link.probe(DEFAULT_PROBE_TIMEOUT_MS) {
                CoState::Present { caps, fw_version } => {
                    log::info!("RP2350 present: caps={caps:04x} fw={fw_version:08x}");
                }
                CoState::Absent => {
                    next_probe = Instant::now() + RETRY_DELAY;
                }
            }
        }

        if link.is_present() {
            // Health heartbeat.
            if Instant::now() >= next_heartbeat {
                next_heartbeat = Instant::now() + HEARTBEAT_PERIOD;
                match link.poll_status(100) {
                    Ok(Some(st)) => {
                        // `Status` is `#[repr(C, packed)]`; pass copies to
                        // `log::info`, whose format machinery would otherwise
                        // borrow the fields (E0793, misaligned reference).
                        let uptime_s = st.uptime_s;
                        let heap_free = st.heap_free;
                        log::info!("RP2350 status: uptime={uptime_s}s heap={heap_free}B");
                    }
                    Ok(None) => log::debug!("RP2350 silent on STATUS"),
                    Err(e) => log::warn!("RP2350 STATUS failed: {e:?}"),
                }
            }
            // Drain anything unsolicited (e.g. a coprocessor-side error).
            for frame in link.drain(20) {
                let kind = frame.kind();
                if kind == cp::msg_type::ERROR {
                    log::warn!("RP2350 ERROR seq={}: {:?}", frame.seq(), frame.payload);
                } else {
                    log::debug!(
                        "RP2350 frame kind=0x{kind:02x} seq={} len={}",
                        frame.seq(),
                        frame.payload.len()
                    );
                }
            }
        }
    }
}
