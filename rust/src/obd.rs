// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! A real car, through an OBD-II dongle. `ObdSource` talks ELM327 -- the
//! AT-command dialect every consumer dongle speaks -- over Bluetooth RFCOMM,
//! TCP (Wi-Fi dongles) or a serial device, polls the standard Mode 01 PIDs
//! the cluster can show, and folds the answers into `Telemetry`.
//!
//! Only reads: Mode 01 requests to the broadcast address, nothing that
//! writes to an ECU. Speed, fuel level, ambient temperature, engine speed,
//! the MIL, and the odometer where the car reports it. Everything else
//! keeps its default; a make-specific request (e.g. a hybrid's battery SOC)
//! can be added to `Pid` once its answer is known.
//!
//! The link is reopened whenever it drops -- dongles sleep with the car --
//! so the cluster may start before the ignition.

use std::io::{self, BufRead, BufReader, Read, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::telemetry::{Telemetry, TelemetrySource, Warn};

/// How the dongle is reached.
#[derive(Clone, Debug, PartialEq)]
pub enum Link {
    /// Bluetooth Classic, RFCOMM channel 1 (the ELM327 convention).
    Bluetooth([u8; 6]),
    /// A Wi-Fi dongle: `host:port`, usually `192.168.0.10:35000`.
    Tcp(String),
    /// A serial device, e.g. `/dev/rfcomm0` or `/dev/ttyUSB0`.
    Serial(String),
}

impl Link {
    /// Parse `bt:<aa:bb:cc:dd:ee:ff>`, `tcp:<host:port>` or `serial:<path>`.
    pub fn parse(spec: &str) -> Result<Self, String> {
        let (kind, rest) = spec
            .split_once(':')
            .ok_or_else(|| format!("no link kind in {spec:?}"))?;
        match kind {
            "bt" => {
                let bytes: Vec<u8> = rest
                    .split(':')
                    .map(|h| u8::from_str_radix(h, 16).map_err(|e| e.to_string()))
                    .collect::<Result<_, _>>()?;
                let addr: [u8; 6] = bytes
                    .try_into()
                    .map_err(|_| format!("bad Bluetooth address {rest:?}"))?;
                Ok(Link::Bluetooth(addr))
            }
            "tcp" => Ok(Link::Tcp(rest.to_string())),
            "serial" => Ok(Link::Serial(rest.to_string())),
            other => Err(format!("unknown link kind {other:?}")),
        }
    }
}

/// The PIDs polled, with how each answer lands in `Telemetry`. Mode 01,
/// standard across makes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pid {
    /// 0D: vehicle speed, km/h.
    Speed,
    /// 2F: fuel tank level, %.
    FuelLevel,
    /// 46: ambient air temperature, deg C.
    AmbientTemp,
    /// 01: monitor status, including the MIL.
    MilStatus,
    /// A6: odometer, km (2020+ cars).
    Odometer,
}

impl Pid {
    fn code(self) -> u8 {
        match self {
            Pid::Speed => 0x0D,
            Pid::FuelLevel => 0x2F,
            Pid::AmbientTemp => 0x46,
            Pid::MilStatus => 0x01,
            Pid::Odometer => 0xA6,
        }
    }

    /// Fold the data bytes of an answer into `t`. `false` if too short.
    fn apply(self, t: &mut Telemetry, d: &[u8]) -> bool {
        match self {
            Pid::Speed if !d.is_empty() => t.speed_kmh = d[0] as f32,
            Pid::FuelLevel if !d.is_empty() => t.fuel_level = d[0] as f32 / 255.0,
            Pid::AmbientTemp if !d.is_empty() => t.outside_temp_c = d[0] as i32 - 40,
            Pid::MilStatus if !d.is_empty() => {
                t.engine = if d[0] & 0x80 != 0 {
                    Warn::Amber
                } else {
                    Warn::Off
                }
            }
            Pid::Odometer if d.len() >= 4 => {
                t.odo_km = u32::from_be_bytes([d[0], d[1], d[2], d[3]]) as f32 / 10.0
            }
            _ => return false,
        }
        true
    }
}

/// Polled in this order, round-robin: speed every time, the slow ones in
/// between.
const SCHEDULE: &[Pid] = &[
    Pid::Speed,
    Pid::FuelLevel,
    Pid::Speed,
    Pid::AmbientTemp,
    Pid::Speed,
    Pid::MilStatus,
    Pid::Speed,
    Pid::Odometer,
];

// ----- the ELM327 conversation ---------------------------------------------

/// An ELM327 on any byte stream.
pub struct Elm<S: Read + Write> {
    io: BufReader<S>,
    /// PIDs the car said it does not support; skipped from then on.
    unsupported: Vec<Pid>,
}

impl<S: Read + Write> Elm<S> {
    pub fn new(stream: S) -> Self {
        Self {
            io: BufReader::new(stream),
            unsupported: Vec::new(),
        }
    }

    /// Send a command and collect the reply up to the `>` prompt, as the
    /// lines the dongle printed (echo and blank lines dropped).
    pub fn command(&mut self, cmd: &str) -> io::Result<Vec<String>> {
        self.io.get_mut().write_all(cmd.as_bytes())?;
        self.io.get_mut().write_all(b"\r")?;
        self.io.get_mut().flush()?;
        let mut lines = Vec::new();
        let mut buf = Vec::new();
        loop {
            buf.clear();
            // Replies end with '>' and no newline; read up to it.
            let n = self.io.read_until(b'>', &mut buf)?;
            if n == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "dongle closed the link",
                ));
            }
            let text = String::from_utf8_lossy(&buf);
            for raw in text.split(['\r', '\n']) {
                let line = raw.trim().trim_end_matches('>').trim();
                if line.is_empty() || line.eq_ignore_ascii_case(cmd) {
                    continue;
                }
                lines.push(line.to_string());
            }
            if buf.ends_with(b">") {
                return Ok(lines);
            }
        }
    }

    /// Reset and set up: no echo, no spaces, ISO 15765 at 500 kbit/s (what
    /// every car since 2008 speaks), and a short timeout so a silent PID
    /// does not stall the loop.
    pub fn init(&mut self) -> io::Result<String> {
        let version = self.command("ATZ")?.join(" ");
        for cmd in ["ATE0", "ATL0", "ATS0", "ATH0", "ATSP6", "ATST19"] {
            self.command(cmd)?;
        }
        Ok(version)
    }

    /// Ask for one PID; `Ok(None)` when the car does not answer it.
    pub fn query(&mut self, pid: Pid) -> io::Result<Option<Vec<u8>>> {
        if self.unsupported.contains(&pid) {
            return Ok(None);
        }
        let lines = self.command(&format!("01{:02X}", pid.code()))?;
        let want = format!("41{:02X}", pid.code());
        for line in &lines {
            // With headers on (or a dongle that ignores ATH0) the answer
            // follows the CAN id and a length byte; find it wherever it is.
            let hex: String = line.chars().filter(|c| c.is_ascii_hexdigit()).collect();
            if let Some(at) = hex.find(&want) {
                let data = &hex[at + want.len()..];
                let bytes: Vec<u8> = (0..data.len() / 2)
                    .filter_map(|i| u8::from_str_radix(&data[2 * i..2 * i + 2], 16).ok())
                    .collect();
                return Ok(Some(bytes));
            }
        }
        if lines
            .iter()
            .any(|l| l.contains("NO DATA") || l.contains("UNABLE") || l.starts_with('7'))
        {
            // "NO DATA" or a negative response: this car does not do it.
            self.unsupported.push(pid);
        }
        Ok(None)
    }
}

// ----- the source ------------------------------------------------------------

/// A `TelemetrySource` fed by an ELM327 dongle.
pub struct ObdSource {
    latest: Arc<Mutex<Telemetry>>,
}

impl ObdSource {
    /// Start polling the dongle at `spec` (see `Link::parse`). Never fails:
    /// a link that cannot be opened is retried in the background.
    pub fn connect(spec: &str) -> Self {
        let latest = Arc::new(Mutex::new(Telemetry::default()));
        let shared = Arc::clone(&latest);
        let spec = spec.to_owned();
        thread::Builder::new()
            .name("obd".into())
            .spawn(move || match Link::parse(&spec) {
                Ok(link) => poll_loop(&link, &shared),
                Err(e) => eprintln!("agl-slint-cluster: CLUSTER_SOURCE=obd:{spec}: {e}"),
            })
            .expect("spawn OBD thread");
        Self { latest }
    }
}

impl TelemetrySource for ObdSource {
    fn poll(&mut self, _dt: f32) -> Telemetry {
        self.latest
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

fn poll_loop(link: &Link, shared: &Mutex<Telemetry>) {
    let mut last_err: Option<String> = None;
    loop {
        match open(link).and_then(|s| {
            let mut elm = Elm::new(s);
            let version = elm.init()?;
            eprintln!("agl-slint-cluster: OBD dongle {version} on {link:?}");
            last_err = None;
            run(&mut elm, shared)
        }) {
            Ok(()) => {}
            Err(e) => {
                let msg = e.to_string();
                if last_err.as_deref() != Some(&msg) {
                    eprintln!("agl-slint-cluster: OBD {link:?}: {msg}; retrying");
                    last_err = Some(msg);
                }
            }
        }
        thread::sleep(Duration::from_secs(2));
    }
}

/// Poll round-robin until the link fails.
fn run<S: Read + Write>(elm: &mut Elm<S>, shared: &Mutex<Telemetry>) -> io::Result<()> {
    loop {
        for &pid in SCHEDULE {
            if let Some(data) = elm.query(pid)? {
                let mut t = shared.lock().unwrap_or_else(|e| e.into_inner());
                pid.apply(&mut t, &data);
            }
        }
    }
}

/// A connected byte stream to the dongle.
enum Stream {
    Tcp(std::net::TcpStream),
    File(std::fs::File),
}

impl Read for Stream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            Stream::Tcp(s) => s.read(buf),
            Stream::File(f) => f.read(buf),
        }
    }
}

impl Write for Stream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Stream::Tcp(s) => s.write(buf),
            Stream::File(f) => f.write(buf),
        }
    }
    fn flush(&mut self) -> io::Result<()> {
        match self {
            Stream::Tcp(s) => s.flush(),
            Stream::File(f) => f.flush(),
        }
    }
}

fn open(link: &Link) -> io::Result<Stream> {
    match link {
        Link::Tcp(addr) => {
            let s = std::net::TcpStream::connect(addr)?;
            s.set_read_timeout(Some(Duration::from_secs(5)))?;
            Ok(Stream::Tcp(s))
        }
        Link::Serial(path) => {
            let f = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .open(path)?;
            raw_tty(&f)?;
            Ok(Stream::File(f))
        }
        Link::Bluetooth(addr) => Ok(Stream::File(rfcomm_connect(addr, 1)?)),
    }
}

/// Put a serial device into raw 8N1 mode at 115200 baud (ignored, harmlessly,
/// on an RFCOMM tty).
fn raw_tty(f: &std::fs::File) -> io::Result<()> {
    use std::os::unix::io::AsRawFd;
    // SAFETY: termios is a plain C struct; tcgetattr fills it in.
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(f.as_raw_fd(), &mut t) != 0 {
            // Not a tty (e.g. a pipe in tests): nothing to set.
            return Ok(());
        }
        libc::cfmakeraw(&mut t);
        libc::cfsetispeed(&mut t, libc::B115200);
        libc::cfsetospeed(&mut t, libc::B115200);
        t.c_cc[libc::VMIN] = 1;
        t.c_cc[libc::VTIME] = 50; // 5 s
        if libc::tcsetattr(f.as_raw_fd(), libc::TCSANOW, &t) != 0 {
            return Err(io::Error::last_os_error());
        }
    }
    Ok(())
}

/// From <bluetooth/bluetooth.h>; the libc crate does not define it.
const BTPROTO_RFCOMM: libc::c_int = 3;

/// Open an RFCOMM connection to a paired Bluetooth Classic device, with no
/// help from the `rfcomm` tool (BlueZ 5 no longer ships it by default).
fn rfcomm_connect(addr: &[u8; 6], channel: u8) -> io::Result<std::fs::File> {
    use std::os::unix::io::FromRawFd;
    #[repr(C, packed)]
    struct SockaddrRc {
        family: libc::sa_family_t,
        bdaddr: [u8; 6],
        channel: u8,
    }
    // Kernel order is the reverse of the printed address.
    let mut bdaddr = *addr;
    bdaddr.reverse();
    let sa = SockaddrRc {
        family: libc::AF_BLUETOOTH as libc::sa_family_t,
        bdaddr,
        channel,
    };
    // SAFETY: plain socket calls with a correctly sized address struct.
    unsafe {
        let fd = libc::socket(libc::AF_BLUETOOTH, libc::SOCK_STREAM, BTPROTO_RFCOMM);
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let tv = libc::timeval {
            tv_sec: 5,
            tv_usec: 0,
        };
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_RCVTIMEO,
            &tv as *const _ as *const libc::c_void,
            std::mem::size_of::<libc::timeval>() as libc::socklen_t,
        );
        if libc::connect(
            fd,
            &sa as *const SockaddrRc as *const libc::sockaddr,
            std::mem::size_of::<SockaddrRc>() as libc::socklen_t,
        ) != 0
        {
            let e = io::Error::last_os_error();
            libc::close(fd);
            return Err(e);
        }
        Ok(std::fs::File::from_raw_fd(fd))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A scripted dongle: answers each command from a queue, ELM style.
    struct Fake {
        script: VecDeque<(&'static str, &'static str)>,
        out: Vec<u8>,
        pending: Vec<u8>,
        sent: Vec<String>,
    }

    impl Fake {
        fn new(script: &[(&'static str, &'static str)]) -> Self {
            Self {
                script: script.iter().copied().collect(),
                out: Vec::new(),
                pending: Vec::new(),
                sent: Vec::new(),
            }
        }
    }

    impl Read for Fake {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.pending.is_empty() {
                return Ok(0);
            }
            let n = buf.len().min(self.pending.len());
            buf[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            Ok(n)
        }
    }

    impl Write for Fake {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.out.extend_from_slice(buf);
            if let Some(pos) = self.out.iter().position(|&b| b == b'\r') {
                let cmd = String::from_utf8_lossy(&self.out[..pos]).to_string();
                self.out.drain(..=pos);
                let (expect, reply) = self
                    .script
                    .pop_front()
                    .unwrap_or_else(|| panic!("unexpected command {cmd}"));
                assert_eq!(cmd, expect);
                self.sent.push(cmd.clone());
                // Real dongles echo the command, then answer, then prompt.
                self.pending
                    .extend_from_slice(format!("{cmd}\r{reply}\r\r>").as_bytes());
            }
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn links_parse() {
        assert_eq!(
            Link::parse("bt:00:1D:A5:68:98:8B"),
            Ok(Link::Bluetooth([0x00, 0x1d, 0xa5, 0x68, 0x98, 0x8b]))
        );
        assert_eq!(
            Link::parse("tcp:192.168.0.10:35000"),
            Ok(Link::Tcp("192.168.0.10:35000".into()))
        );
        assert_eq!(
            Link::parse("serial:/dev/rfcomm0"),
            Ok(Link::Serial("/dev/rfcomm0".into()))
        );
        assert!(Link::parse("bt:zz").is_err());
        assert!(Link::parse("usb:0").is_err());
    }

    #[test]
    fn init_then_a_round_of_pids() {
        let fake = Fake::new(&[
            ("ATZ", "ELM327 v2.2"),
            ("ATE0", "OK"),
            ("ATL0", "OK"),
            ("ATS0", "OK"),
            ("ATH0", "OK"),
            ("ATSP6", "OK"),
            ("ATST19", "OK"),
            ("010D", "410D3C"),       // 60 km/h
            ("012F", "412F80"),       // 50 % fuel
            ("0146", "41463D"),       // 61 - 40 = 21 C
            ("0101", "41018107E500"), // MIL on
            ("01A6", "41A600012D4A"), // 77130 -> 7713.0 km
        ]);
        let mut elm = Elm::new(fake);
        assert_eq!(elm.init().unwrap(), "ELM327 v2.2");
        let mut t = Telemetry::default();
        for pid in [
            Pid::Speed,
            Pid::FuelLevel,
            Pid::AmbientTemp,
            Pid::MilStatus,
            Pid::Odometer,
        ] {
            let data = elm
                .query(pid)
                .unwrap()
                .unwrap_or_else(|| panic!("{pid:?} unanswered"));
            assert!(pid.apply(&mut t, &data));
        }
        assert_eq!(t.speed_kmh, 60.0);
        assert!((t.fuel_level - 128.0 / 255.0).abs() < 1e-6);
        assert_eq!(t.outside_temp_c, 21);
        assert_eq!(t.engine, Warn::Amber);
        assert!((t.odo_km - 7713.0).abs() < 1e-3);
    }

    #[test]
    fn spaced_answers_and_headers_are_fine() {
        let fake = Fake::new(&[("010D", "7E8 03 41 0D 28")]);
        let mut elm = Elm::new(fake);
        assert_eq!(elm.query(Pid::Speed).unwrap(), Some(vec![0x28]));
    }

    #[test]
    fn unsupported_pids_are_asked_once() {
        let fake = Fake::new(&[("01A6", "NO DATA"), ("010D", "410D00")]);
        let mut elm = Elm::new(fake);
        assert_eq!(elm.query(Pid::Odometer).unwrap(), None);
        assert_eq!(elm.query(Pid::Odometer).unwrap(), None); // not sent again
        assert_eq!(elm.query(Pid::Speed).unwrap(), Some(vec![0]));
        assert_eq!(elm.io.get_ref().sent, vec!["01A6", "010D"]);
    }
}
