//! lwIP UDP binding for the radar transport (`feature = "device"`).
//!
//! ESP-IDF's modern Rust stack does not ship a safe UDP socket wrapper, so we
//! call the lwIP socket API directly through the generated `esp-idf-sys`
//! bindings (`lwip_socket`, `lwip_sendto`, `lwip_recvfrom`, ...). These are
//! the same functions esp-idf-svc's own HTTP server uses internally.
//!
//! Constants (AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_BROADCAST, SO_RCVTIMEO) are
//! the lwIP 2.1 values from `lwip/sockets.h`, which `esp-idf-sys`'s bindgen
//! run re-exports as `sys::AF_INET` etc.
//!
//! ## sockaddr layout
//!
//! lwIP's `sockaddr_in` on ESP32 is:
//! `{ u8 sin_len; u8 sin_family; u16 sin_port; u32 sin_addr; u8 sin_zero[8] }`
//! (16 bytes). We define our own `#[repr(C)]` copy rather than touching
//! bindgen's field names, and cast to `*const sys::sockaddr` for the FFI call
//! — the layouts are identical.

use crate::{Ipv4Addr, build_cal_resp, build_csi_snapshot, build_data_frame, build_feature_report, node};
use core::ffi::c_void;
use esp_idf_sys as sys;
use radar_protocol::{CalResp, CsiSnapshot, FeatureReport};

const AF_INET: u8 = 2;
const SOCK_DGRAM: u8 = 2;
const IPPROTO_UDP: u8 = 17;
const SOL_SOCKET: u32 = 0xFFF; // lwIP-specific value
const SO_BROADCAST: u32 = 0x0020;
const SO_RCVTIMEO: u32 = 0x1006;
const INADDR_ANY: u32 = 0;

/// `sockaddr_in` with the lwIP/ESP32 layout (see module docs).
#[repr(C)]
#[derive(Clone, Copy)]
struct SockAddrIn {
    sin_len: u8,
    sin_family: u8,
    sin_port: u16, // network byte order
    sin_addr: u32, // network byte order
    sin_zero: [u8; 8],
}

impl SockAddrIn {
    fn new(ip: Ipv4Addr, port: u16) -> Self {
        Self {
            sin_len: 16,
            sin_family: AF_INET,
            sin_port: port.to_be(),
            sin_addr: u32::from_be_bytes(ip.0),
            sin_zero: [0; 8],
        }
    }

    fn any(port: u16) -> Self {
        Self {
            sin_len: 16,
            sin_family: AF_INET,
            sin_port: port.to_be(),
            sin_addr: INADDR_ANY,
            sin_zero: [0; 8],
        }
    }

    fn as_ptr(&self) -> *const sys::sockaddr {
        self as *const SockAddrIn as *const sys::sockaddr
    }

    fn as_mut_ptr(&mut self) -> *mut sys::sockaddr {
        self as *mut SockAddrIn as *mut sys::sockaddr
    }
}

/// `struct timeval` (lwIP: two 32-bit `long`s on ESP32) for SO_RCVTIMEO.
#[repr(C)]
struct TimeVal {
    tv_sec: i32,
    tv_usec: i32,
}

/// Errors from the lwIP UDP binding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UdpError {
    Open(i32),
    SetOpt(i32),
    Send(i32),
    Recv(i32),
    /// Frame bigger than the caller's buffer.
    TooSmall,
}

impl core::fmt::Display for UdpError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            UdpError::Open(e) => write!(f, "lwip_socket failed ({e})"),
            UdpError::SetOpt(e) => write!(f, "lwip_setsockopt failed ({e})"),
            UdpError::Send(e) => write!(f, "lwip_sendto failed ({e})"),
            UdpError::Recv(e) => write!(f, "lwip_recvfrom failed ({e})"),
            UdpError::TooSmall => write!(f, "frame exceeds buffer"),
        }
    }
}

/// A thin blocking UDP socket over lwIP.
pub struct UdpSocket {
    fd: i32,
}

impl UdpSocket {
    /// Open a DGRAM socket bound to `0.0.0.0:port` (port 0 = ephemeral).
    pub fn bind(port: u16) -> Result<Self, UdpError> {
        let fd = unsafe { sys::lwip_socket(AF_INET as i32, SOCK_DGRAM as i32, IPPROTO_UDP as i32) };
        if fd < 0 {
            return Err(UdpError::Open(fd));
        }
        let addr = SockAddrIn::any(port);
        let rc = unsafe { sys::lwip_bind(fd, addr.as_ptr(), core::mem::size_of::<SockAddrIn>() as sys::socklen_t) };
        if rc < 0 {
            unsafe { sys::lwip_close(fd) };
            return Err(UdpError::Open(rc));
        }
        Ok(Self { fd })
    }

    pub fn fd(&self) -> i32 {
        self.fd
    }

    pub fn set_broadcast(&mut self, enabled: bool) -> Result<(), UdpError> {
        let v: i32 = if enabled { 1 } else { 0 };
        let rc = unsafe {
            sys::lwip_setsockopt(
                self.fd,
                SOL_SOCKET as i32,
                SO_BROADCAST as i32,
                &v as *const i32 as *const c_void,
                core::mem::size_of::<i32>() as sys::socklen_t,
            )
        };
        if rc < 0 {
            Err(UdpError::SetOpt(rc))
        } else {
            Ok(())
        }
    }

    /// Blocking receive timeout in milliseconds (0 = infinite).
    pub fn set_recv_timeout(&mut self, ms: i32) -> Result<(), UdpError> {
        let tv = TimeVal {
            tv_sec: ms / 1000,
            tv_usec: (ms % 1000) * 1000,
        };
        let rc = unsafe {
            sys::lwip_setsockopt(
                self.fd,
                SOL_SOCKET as i32,
                SO_RCVTIMEO as i32,
                &tv as *const TimeVal as *const c_void,
                core::mem::size_of::<TimeVal>() as sys::socklen_t,
            )
        };
        if rc < 0 {
            Err(UdpError::SetOpt(rc))
        } else {
            Ok(())
        }
    }

    /// Send `buf` to `ip:port`.
    pub fn send_to(&mut self, ip: Ipv4Addr, port: u16, buf: &[u8]) -> Result<usize, UdpError> {
        let addr = SockAddrIn::new(ip, port);
        let rc = unsafe {
            sys::lwip_sendto(
                self.fd,
                buf.as_ptr() as *const c_void,
                buf.len() as usize,
                0,
                addr.as_ptr(),
                core::mem::size_of::<SockAddrIn>() as sys::socklen_t,
            )
        };
        if rc < 0 {
            Err(UdpError::Send(rc as i32))
        } else {
            Ok(rc as usize)
        }
    }

    /// Receive into `buf`. Returns (len, peer ip, peer port). `None` on
    /// timeout or error (a timed-out read returns -1 from lwIP).
    pub fn recv_from(&mut self, buf: &mut [u8]) -> Option<(usize, Ipv4Addr, u16)> {
        let mut from = SockAddrIn::any(0);
        let mut fromlen = core::mem::size_of::<SockAddrIn>() as sys::socklen_t;
        let rc = unsafe {
            sys::lwip_recvfrom(
                self.fd,
                buf.as_mut_ptr() as *mut c_void,
                buf.len() as usize,
                0,
                from.as_mut_ptr(),
                &mut fromlen,
            )
        };
        if rc < 0 {
            return None;
        }
        let ip = Ipv4Addr(from.sin_addr.to_be_bytes());
        let port = u16::from_be(from.sin_port);
        Some((rc as usize, ip, port))
    }
}

impl Drop for UdpSocket {
    fn drop(&mut self) {
        unsafe {
            sys::lwip_close(self.fd);
        }
    }
}

/// Monotonic µs clock (ESP-IDF `esp_timer_get_time`).
pub fn now_us() -> u64 {
    unsafe { sys::esp_timer_get_time() as u64 }
}

/// RADAR-TX measurement traffic generator.
///
/// Sends one broadcast `DataFrame` per sequence number to the AP's subnet
/// broadcast address; every RX station on the same AP receives it. Maintains
/// the global TX sequence counter.
pub struct TrafficSender {
    sock: UdpSocket,
    dst: Ipv4Addr,
    port: u16,
    seq: u32,
    buf: [u8; 256],
    pub frames_sent: u64,
}

impl TrafficSender {
    pub fn bind(port: u16) -> Result<Self, UdpError> {
        let mut sock = UdpSocket::bind(port)?;
        sock.set_broadcast(true)?;
        Ok(Self {
            sock,
            dst: Ipv4Addr::AP_BROADCAST,
            port,
            seq: 0,
            buf: [0u8; 256],
            frames_sent: 0,
        })
    }

    /// Override the destination (default: AP subnet broadcast).
    pub fn set_target(&mut self, dst: Ipv4Addr, port: u16) {
        self.dst = dst;
        self.port = port;
    }

    pub fn seq(&self) -> u32 {
        self.seq
    }

    /// Send the next measurement frame. Returns the TX sequence used.
    pub fn send(&mut self, tx_power_db: u8, cal: bool) -> Result<u32, UdpError> {
        let seq = self.seq;
        let n = build_data_frame(&mut self.buf, node::TX, seq, now_us(), tx_power_db, cal);
        self.sock.send_to(self.dst, self.port, &self.buf[..n])?;
        self.seq = self.seq.wrapping_add(1);
        self.frames_sent += 1;
        Ok(seq)
    }
}

/// RADAR-RX feature reporter.
///
/// Sends a `FeatureReport` to RADAR-TX. The default target is the AP address;
/// `learn_target` can point it at the exact TX address observed in the
/// measurement stream (robust to a changed AP IP).
pub struct FeatureReporter {
    sock: UdpSocket,
    target: (Ipv4Addr, u16),
    buf: [u8; 512],
    pub reports_sent: u64,
}

impl FeatureReporter {
    pub fn bind(port: u16) -> Result<Self, UdpError> {
        let sock = UdpSocket::bind(port)?;
        Ok(Self {
            sock,
            target: (Ipv4Addr::AP, crate::REPORT_PORT),
            buf: [0u8; 512],
            reports_sent: 0,
        })
    }

    /// Point reports at a specific TX address (e.g. from the last data frame's
    /// source address).
    pub fn learn_target(&mut self, ip: Ipv4Addr, port: u16) {
        self.target = (ip, port);
    }

    pub fn target(&self) -> (Ipv4Addr, u16) {
        self.target
    }

    /// Send `report`. Returns the report seq on success.
    pub fn send(&mut self, src: u8, report: &FeatureReport) -> Result<u32, UdpError> {
        let n = build_feature_report(&mut self.buf, src, report, now_us());
        let (ip, port) = self.target;
        self.sock.send_to(ip, port, &self.buf[..n])?;
        self.reports_sent += 1;
        Ok(report.seq)
    }

    /// Send a calibration response to RADAR-TX. The TX matches replies by
    /// `CalResp.stage` from the payload, so no extra routing is needed.
    pub fn send_cal_resp(&mut self, src: u8, resp: &CalResp) -> Result<(), UdpError> {
        let n = build_cal_resp(&mut self.buf, src, resp, now_us());
        let (ip, port) = self.target;
        self.sock.send_to(ip, port, &self.buf[..n])?;
        Ok(())
    }

    /// Send a low-rate CSI snapshot to RADAR-TX for the LIVE WATERFALL and
    /// PER-SUBCARRIER dashboard plots (spec §6). Snapshot frames are ~380 B,
    /// comfortably inside the 512 B send buffer.
    pub fn send_csi_snapshot(&mut self, src: u8, snap: &CsiSnapshot) -> Result<(), UdpError> {
        let n = build_csi_snapshot(&mut self.buf, src, snap, now_us());
        let (ip, port) = self.target;
        self.sock.send_to(ip, port, &self.buf[..n])?;
        Ok(())
    }
}

/// Receive and parse the next radar frame. Returns
/// (kind, src, seq, payload-view). The payload borrows `buf`, which the caller
/// must keep alive. `None` on timeout/invalid frame.
pub fn recv_radar_frame<'a>(
    sock: &mut UdpSocket,
    buf: &'a mut [u8],
) -> Option<(u8, u8, u32, &'a [u8], Ipv4Addr, u16)> {
    let (n, peer_ip, peer_port) = sock.recv_from(buf)?;
    let (kind, src, seq, payload) = crate::parse_frame(&buf[..n])?;
    Some((kind, src, seq, payload, peer_ip, peer_port))
}
