//! Measurement traffic generator (spec §6).
//!
//! Broadcasts one `DataFrame` per sequence number to the AP's subnet; both RX
//! stations hear the *same* packets, which is exactly what makes cross-link
//! sequence pairing work (§15). Traffic runs whether or not a dashboard is
//! connected — the radar never stops sensing.

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use radar_transport::udp::TrafficSender;
use radar_transport::MEASURE_PORT;

/// Throttle send-error logging to once per 5 s so a wedged network can't spam
/// the console at the measurement rate.
const ERROR_LOG_PERIOD: Duration = Duration::from_secs(5);

pub fn run(rate_hz: u16, tx_power: Arc<AtomicU8>, cal_active: Arc<AtomicBool>) {
    let mut sender = match TrafficSender::bind(MEASURE_PORT) {
        Ok(s) => s,
        Err(e) => {
            log::error!("traffic sender bind failed: {e}; measurement traffic disabled");
            return;
        }
    };
    let rate_hz = rate_hz.max(1) as u64;
    let period = Duration::from_micros(1_000_000 / rate_hz);
    log::info!("traffic task up: {rate_hz} frames/s on port {MEASURE_PORT}");

    let mut next = Instant::now();
    let mut last_warn = Instant::now() - ERROR_LOG_PERIOD;
    loop {
        let power = tx_power.load(Ordering::Relaxed);
        let cal = cal_active.load(Ordering::Relaxed);
        if let Err(e) = sender.send(power, cal) {
            if last_warn.elapsed() >= ERROR_LOG_PERIOD {
                log::warn!("traffic send failed: {e}");
                last_warn = Instant::now();
            }
        }
        next += period;
        let now = Instant::now();
        if now < next {
            std::thread::sleep(next - now);
        } else {
            // Fell behind (a burst or a hiccup); resync instead of spiralling.
            next = now;
        }
    }
}
