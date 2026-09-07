// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! The cluster's CAN interface: a small set of classic (8-byte, 11-bit ID)
//! frames that carry exactly the fields of `Telemetry`, an encoder and a
//! decoder for them, and `CanSource`, which reads them off a SocketCAN
//! interface. The same layout is described for other tools in
//! `can/agl-slint-cluster.dbc` at the repository root; keep the two in sync.
//!
//! All multi-byte fields are little-endian (Intel byte order in DBC terms).
//! A frame updates only the fields it carries, so a sender is free to emit a
//! subset -- e.g. just `MOTION` from a bench script -- and the rest keeps its
//! last (or default) value.

use std::io;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Frame, Socket};

use crate::telemetry::{Beam, Cruise, Lane, Telemetry, TelemetrySource, Warn};

/// Speed and drivetrain: `speed_kmh` u16 x0.01, `power` i16 x0.001,
/// `efficiency_percent` u8, `gear` ASCII u8, byte 6 bit 0 `ready`.
pub const ID_MOTION: u16 = 0x100;
/// Energy: `battery_level` u8 %, `fuel_level` u8 %, `range_km` u16,
/// `avg_efficiency_kwh` u16 x0.01, `outside_temp_c` i8.
pub const ID_ENERGY: u16 = 0x101;
/// Trip: `odo_km` u32 x0.1, `trip_seconds` u32.
pub const ID_TRIP: u16 = 0x102;
/// Driver assistance: `cruise_speed_kmh` u8, `cruise` u8, `lane` u8,
/// flags (bit 0 `lta_on`, bit 1 `lca_on`), `turn_distance_km` u16 x0.01.
pub const ID_ADAS: u16 = 0x103;
/// Telltales: byte 0 flags (bit 0 left, 1 right, 2-3 headlights, 4 parking
/// brake, 5 seatbelt, 6 door), then `engine`, `oil`, `battery_warn`, `abs`
/// as 0 off / 1 amber / 2 red.
pub const ID_TELLTALES: u16 = 0x104;
/// Next-turn street name, as UTF-8 in chunks: byte 0 is the chunk index
/// (0..STREET_CHUNKS), bytes 1..8 are up to 7 bytes of text. Chunk 0 starts a
/// new name, so a short name needs only one frame.
pub const ID_STREET: u16 = 0x105;

/// Text bytes per `ID_STREET` frame.
const STREET_CHUNK: usize = 7;
/// Chunks per street name; 4 x 7 = 28 bytes.
const STREET_CHUNKS: usize = 4;

/// One classic CAN frame: an 11-bit ID and up to 8 data bytes. Kept
/// library-neutral so the encoder and decoder can be tested without a socket.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message {
    pub id: u16,
    pub data: Vec<u8>,
}

/// Reassembly buffer for the street name, which spans several frames.
#[derive(Default)]
pub struct StreetBuffer {
    bytes: [u8; STREET_CHUNK * STREET_CHUNKS],
}

impl StreetBuffer {
    fn text(&self) -> String {
        let end = self
            .bytes
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.bytes.len());
        String::from_utf8_lossy(&self.bytes[..end]).into_owned()
    }
}

// ----- encoding --------------------------------------------------------------

fn warn_code(w: Warn) -> u8 {
    match w {
        Warn::Off => 0,
        Warn::Amber => 1,
        Warn::Red => 2,
    }
}

fn warn_from(code: u8) -> Warn {
    match code {
        1 => Warn::Amber,
        2 => Warn::Red,
        _ => Warn::Off,
    }
}

/// Scale a physical value into an integer field, saturating at the type's range.
fn quant(v: f32, scale: f32, lo: f32, hi: f32) -> i64 {
    (v * scale).round().clamp(lo, hi) as i64
}

/// Encode a whole `Telemetry` frame as the set of CAN messages that carry it.
pub fn encode(t: &Telemetry) -> Vec<Message> {
    let mut out = Vec::with_capacity(5 + STREET_CHUNKS);

    let speed = quant(t.speed_kmh, 100.0, 0.0, 65535.0) as u16;
    let power = quant(t.power, 1000.0, -1000.0, 1000.0) as i16;
    let mut d = Vec::with_capacity(8);
    d.extend_from_slice(&speed.to_le_bytes());
    d.extend_from_slice(&power.to_le_bytes());
    d.push(quant(t.efficiency_percent, 1.0, 0.0, 255.0) as u8);
    d.push(if t.gear.is_ascii() {
        t.gear as u8
    } else {
        b'?'
    });
    d.push(t.ready as u8);
    d.push(0);
    out.push(Message {
        id: ID_MOTION,
        data: d,
    });

    let range = quant(t.range_km, 1.0, 0.0, 65535.0) as u16;
    let avg = quant(t.avg_efficiency_kwh, 100.0, 0.0, 65535.0) as u16;
    let mut d = Vec::with_capacity(8);
    d.push(quant(t.battery_level, 100.0, 0.0, 100.0) as u8);
    d.push(quant(t.fuel_level, 100.0, 0.0, 100.0) as u8);
    d.extend_from_slice(&range.to_le_bytes());
    d.extend_from_slice(&avg.to_le_bytes());
    d.push(t.outside_temp_c.clamp(-128, 127) as i8 as u8);
    d.push(0);
    out.push(Message {
        id: ID_ENERGY,
        data: d,
    });

    let odo = quant(t.odo_km, 10.0, 0.0, u32::MAX as f32) as u32;
    let mut d = Vec::with_capacity(8);
    d.extend_from_slice(&odo.to_le_bytes());
    d.extend_from_slice(&t.trip_seconds.to_le_bytes());
    out.push(Message {
        id: ID_TRIP,
        data: d,
    });

    let turn = quant(t.turn_distance_km, 100.0, 0.0, 65535.0) as u16;
    let mut d = Vec::with_capacity(8);
    d.push(quant(t.cruise_speed_kmh, 1.0, 0.0, 255.0) as u8);
    d.push(match t.cruise {
        Cruise::Off => 0,
        Cruise::Set => 1,
        Cruise::Active => 2,
    });
    d.push(match t.lane {
        Lane::Off => 0,
        Lane::Tracking => 1,
        Lane::Warn => 2,
        Lane::Depart => 3,
    });
    d.push((t.lta_on as u8) | ((t.lca_on as u8) << 1));
    d.extend_from_slice(&turn.to_le_bytes());
    d.extend_from_slice(&[0, 0]);
    out.push(Message {
        id: ID_ADAS,
        data: d,
    });

    let beam = match t.headlights {
        Beam::Off => 0,
        Beam::Low => 1,
        Beam::High => 2,
    };
    let flags = (t.turn_left as u8)
        | ((t.turn_right as u8) << 1)
        | (beam << 2)
        | ((t.parking_brake as u8) << 4)
        | ((t.seatbelt as u8) << 5)
        | ((t.door_open as u8) << 6);
    out.push(Message {
        id: ID_TELLTALES,
        data: vec![
            flags,
            warn_code(t.engine),
            warn_code(t.oil),
            warn_code(t.battery_warn),
            warn_code(t.abs),
            0,
            0,
            0,
        ],
    });

    // The street name is cut at a character boundary so no chunk carries a
    // torn UTF-8 sequence -- the decoder tolerates one, but tools like
    // candump are nicer to read this way.
    let max = STREET_CHUNK * STREET_CHUNKS;
    let mut text = t.turn_street.as_str();
    if text.len() > max {
        let mut cut = max;
        while !text.is_char_boundary(cut) {
            cut -= 1;
        }
        text = &text[..cut];
    }
    let bytes = text.as_bytes();
    let chunks = bytes.len().div_ceil(STREET_CHUNK).max(1);
    for i in 0..chunks {
        let start = i * STREET_CHUNK;
        let end = (start + STREET_CHUNK).min(bytes.len());
        let mut d = Vec::with_capacity(8);
        d.push(i as u8);
        d.extend_from_slice(&bytes[start..end]);
        out.push(Message {
            id: ID_STREET,
            data: d,
        });
    }

    out
}

// ----- decoding --------------------------------------------------------------

fn u16_at(d: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([d[i], d[i + 1]])
}

fn u32_at(d: &[u8], i: usize) -> u32 {
    u32::from_le_bytes([d[i], d[i + 1], d[i + 2], d[i + 3]])
}

/// Apply one CAN message to `t`. A frame may be shorter than its message:
/// only the fields it fully contains are updated, so `cansend vcan0 100#1027`
/// sets the speed and nothing else. Returns `false` for an ID this cluster
/// does not know; `t` is then untouched.
pub fn decode(t: &mut Telemetry, street: &mut StreetBuffer, id: u16, d: &[u8]) -> bool {
    let n = d.len();
    match id {
        ID_MOTION => {
            if n >= 2 {
                t.speed_kmh = u16_at(d, 0) as f32 * 0.01;
            }
            if n >= 4 {
                t.power = (u16_at(d, 2) as i16) as f32 * 0.001;
            }
            if n >= 5 {
                t.efficiency_percent = d[4] as f32;
            }
            if n >= 6 {
                t.gear = d[5] as char;
            }
            if n >= 7 {
                t.ready = d[6] & 1 != 0;
            }
        }
        ID_ENERGY => {
            if n >= 1 {
                t.battery_level = d[0] as f32 * 0.01;
            }
            if n >= 2 {
                t.fuel_level = d[1] as f32 * 0.01;
            }
            if n >= 4 {
                t.range_km = u16_at(d, 2) as f32;
            }
            if n >= 6 {
                t.avg_efficiency_kwh = u16_at(d, 4) as f32 * 0.01;
            }
            if n >= 7 {
                t.outside_temp_c = d[6] as i8 as i32;
            }
        }
        ID_TRIP => {
            if n >= 4 {
                t.odo_km = u32_at(d, 0) as f32 * 0.1;
            }
            if n >= 8 {
                t.trip_seconds = u32_at(d, 4);
            }
        }
        ID_ADAS => {
            if n >= 1 {
                t.cruise_speed_kmh = d[0] as f32;
            }
            if n >= 2 {
                t.cruise = match d[1] {
                    1 => Cruise::Set,
                    2 => Cruise::Active,
                    _ => Cruise::Off,
                };
            }
            if n >= 3 {
                t.lane = match d[2] {
                    1 => Lane::Tracking,
                    2 => Lane::Warn,
                    3 => Lane::Depart,
                    _ => Lane::Off,
                };
            }
            if n >= 4 {
                t.lta_on = d[3] & 1 != 0;
                t.lca_on = d[3] & 2 != 0;
            }
            if n >= 6 {
                t.turn_distance_km = u16_at(d, 4) as f32 * 0.01;
            }
        }
        ID_TELLTALES => {
            if n >= 1 {
                let f = d[0];
                t.turn_left = f & 1 != 0;
                t.turn_right = f & 2 != 0;
                t.headlights = match (f >> 2) & 3 {
                    1 => Beam::Low,
                    2 => Beam::High,
                    _ => Beam::Off,
                };
                t.parking_brake = f & 0x10 != 0;
                t.seatbelt = f & 0x20 != 0;
                t.door_open = f & 0x40 != 0;
            }
            if n >= 2 {
                t.engine = warn_from(d[1]);
            }
            if n >= 3 {
                t.oil = warn_from(d[2]);
            }
            if n >= 4 {
                t.battery_warn = warn_from(d[3]);
            }
            if n >= 5 {
                t.abs = warn_from(d[4]);
            }
        }
        ID_STREET => {
            if n >= 1 && (d[0] as usize) < STREET_CHUNKS {
                let i = d[0] as usize;
                if i == 0 {
                    street.bytes.fill(0);
                }
                let slot = &mut street.bytes[i * STREET_CHUNK..(i + 1) * STREET_CHUNK];
                slot.fill(0);
                let text = &d[1..n.min(1 + STREET_CHUNK)];
                slot[..text.len()].copy_from_slice(text);
                t.turn_street = street.text();
            }
        }
        _ => return false,
    }
    true
}

// ----- the source ------------------------------------------------------------

/// A `TelemetrySource` fed from a SocketCAN interface.
///
/// A background thread owns the socket and decodes frames into a shared
/// `Telemetry`; `poll` just clones the latest state, so it never blocks the UI.
/// The interface is (re)opened whenever it is missing or goes down, so the
/// cluster can start before `can0` is up and shows the quiet default frame
/// until the first message arrives.
pub struct CanSource {
    latest: Arc<Mutex<Telemetry>>,
}

impl CanSource {
    /// Start reading `iface` (e.g. `"can0"`, `"vcan0"`). Never fails: an
    /// interface that cannot be opened is retried in the background.
    pub fn open(iface: &str) -> Self {
        let latest = Arc::new(Mutex::new(Telemetry::default()));
        let shared = Arc::clone(&latest);
        let iface = iface.to_owned();
        thread::Builder::new()
            .name("can-rx".into())
            .spawn(move || rx_loop(&iface, &shared))
            .expect("spawn CAN reader thread");
        Self { latest }
    }
}

impl TelemetrySource for CanSource {
    fn poll(&mut self, _dt: f32) -> Telemetry {
        self.latest
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

fn rx_loop(iface: &str, shared: &Mutex<Telemetry>) {
    let mut street = StreetBuffer::default();
    let mut last_err: Option<String> = None;
    loop {
        match CanSocket::open(iface) {
            Ok(sock) => {
                eprintln!("agl-slint-cluster: reading CAN frames from {iface}");
                last_err = None;
                let err = read_until_error(&sock, shared, &mut street);
                eprintln!("agl-slint-cluster: {iface}: {err}; reopening");
            }
            Err(e) => {
                // Log each distinct failure once, not once a second.
                let msg = e.to_string();
                if last_err.as_deref() != Some(&msg) {
                    eprintln!("agl-slint-cluster: cannot open {iface}: {msg}; retrying");
                    last_err = Some(msg);
                }
            }
        }
        thread::sleep(Duration::from_secs(1));
    }
}

fn read_until_error(
    sock: &CanSocket,
    shared: &Mutex<Telemetry>,
    street: &mut StreetBuffer,
) -> io::Error {
    loop {
        let frame = match sock.read_frame() {
            Ok(f) => f,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return e,
        };
        // Remote and error frames carry no signal data; only standard IDs
        // are assigned.
        let CanFrame::Data(data) = frame else {
            continue;
        };
        if data.is_extended() {
            continue;
        }
        let id = data.raw_id() as u16;
        let mut t = shared.lock().unwrap_or_else(|e| e.into_inner());
        decode(&mut t, street, id, data.data());
    }
}

/// Build a SocketCAN frame from a `Message`, for senders such as `cansim`.
pub fn to_frame(m: &Message) -> CanFrame {
    CanFrame::from_raw_id(m.id as u32, &m.data).expect("valid standard ID and <= 8 bytes")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(t: &Telemetry) -> Telemetry {
        let mut out = Telemetry::default();
        let mut street = StreetBuffer::default();
        for m in encode(t) {
            assert!(m.data.len() <= 8, "frame too long: {m:?}");
            assert!(
                decode(&mut out, &mut street, m.id, &m.data),
                "unrecognised: {m:?}"
            );
        }
        out
    }

    /// Field-by-field comparison with the tolerance of each field's encoding.
    fn assert_matches(t: &Telemetry, r: &Telemetry) {
        assert!((r.speed_kmh - t.speed_kmh).abs() < 0.006);
        assert!((r.power - t.power).abs() < 0.0006);
        assert_eq!(r.efficiency_percent, t.efficiency_percent);
        assert!((r.avg_efficiency_kwh - t.avg_efficiency_kwh).abs() < 0.006);
        assert_eq!(r.range_km, t.range_km);
        assert!((r.battery_level - t.battery_level).abs() < 0.006);
        assert!((r.fuel_level - t.fuel_level).abs() < 0.006);
        assert_eq!(r.outside_temp_c, t.outside_temp_c);
        assert!((r.odo_km - t.odo_km).abs() < 0.06);
        assert_eq!(r.trip_seconds, t.trip_seconds);
        assert!((r.turn_distance_km - t.turn_distance_km).abs() < 0.006);
        assert_eq!(r.turn_street, t.turn_street);
        assert_eq!(r.cruise_speed_kmh, t.cruise_speed_kmh);
        assert_eq!(r.lta_on, t.lta_on);
        assert_eq!(r.lca_on, t.lca_on);
        assert_eq!(r.gear, t.gear);
        assert_eq!(r.ready, t.ready);
        assert_eq!(r.turn_left, t.turn_left);
        assert_eq!(r.turn_right, t.turn_right);
        assert_eq!(r.headlights, t.headlights);
        assert_eq!(r.engine, t.engine);
        assert_eq!(r.oil, t.oil);
        assert_eq!(r.battery_warn, t.battery_warn);
        assert_eq!(r.abs, t.abs);
        assert_eq!(r.parking_brake, t.parking_brake);
        assert_eq!(r.seatbelt, t.seatbelt);
        assert_eq!(r.door_open, t.door_open);
        assert_eq!(r.cruise, t.cruise);
        assert_eq!(r.lane, t.lane);
    }

    #[test]
    fn default_frame_survives_roundtrip() {
        let t = Telemetry::default();
        assert_matches(&t, &roundtrip(&t));
    }

    #[test]
    fn every_field_roundtrips() {
        let t = Telemetry {
            speed_kmh: 123.45,
            power: -0.375,
            efficiency_percent: 67.0,
            avg_efficiency_kwh: 5.25,
            range_km: 321.0,
            battery_level: 0.42,
            fuel_level: 0.07,
            outside_temp_c: -12,
            odo_km: 98_765.4,
            trip_seconds: 3_723,
            turn_distance_km: 0.35,
            turn_street: "Sakuragicho Station Rd".into(),
            cruise_speed_kmh: 100.0,
            lta_on: false,
            lca_on: true,
            gear: 'R',
            ready: false,
            turn_left: true,
            turn_right: false,
            headlights: Beam::High,
            engine: Warn::Amber,
            oil: Warn::Red,
            battery_warn: Warn::Amber,
            abs: Warn::Red,
            parking_brake: true,
            seatbelt: true,
            door_open: true,
            cruise: Cruise::Set,
            lane: Lane::Depart,
        };
        assert_matches(&t, &roundtrip(&t));
    }

    #[test]
    fn street_chunk_zero_starts_a_new_name() {
        let mut t = Telemetry::default();
        let mut s = StreetBuffer::default();
        let long = Telemetry {
            turn_street: "Minatomirai Boulevard".into(),
            ..Telemetry::default()
        };
        for m in encode(&long).into_iter().filter(|m| m.id == ID_STREET) {
            decode(&mut t, &mut s, m.id, &m.data);
        }
        assert_eq!(t.turn_street, "Minatomirai Boulevard");
        // A single short chunk 0 replaces the whole name.
        decode(&mut t, &mut s, ID_STREET, b"\0Home");
        assert_eq!(t.turn_street, "Home");
    }

    #[test]
    fn long_street_is_cut_at_a_char_boundary() {
        let t = Telemetry {
            turn_street: "みなとみらい大通り横浜駅前".into(),
            ..Telemetry::default()
        };
        let r = roundtrip(&t);
        assert!(t.turn_street.starts_with(&r.turn_street));
        assert!(r.turn_street.len() <= STREET_CHUNK * STREET_CHUNKS);
        assert!(r.turn_street.len() > STREET_CHUNK * STREET_CHUNKS - 3);
    }

    #[test]
    fn short_frames_update_only_the_fields_they_carry() {
        let mut t = Telemetry::default();
        let mut s = StreetBuffer::default();
        // 100.00 km/h and nothing else: power, gear, ready stay as they were.
        assert!(decode(&mut t, &mut s, ID_MOTION, &[0x10, 0x27]));
        assert_eq!(t.speed_kmh, 100.0);
        assert_eq!(t.gear, 'D');
        assert!(t.ready);
        // One flag byte: lamps change, the warning levels do not.
        t.engine = Warn::Red;
        assert!(decode(&mut t, &mut s, ID_TELLTALES, &[0x15]));
        assert!(t.turn_left && t.parking_brake && !t.turn_right);
        assert_eq!(t.headlights, Beam::Low);
        assert_eq!(t.engine, Warn::Red);
        // A single byte cannot carry a 16-bit field; it is left alone.
        assert!(decode(&mut t, &mut s, ID_MOTION, &[0xff]));
        assert_eq!(t.speed_kmh, 100.0);
    }

    #[test]
    fn unknown_ids_are_ignored() {
        let mut t = Telemetry::default();
        let mut s = StreetBuffer::default();
        assert!(!decode(&mut t, &mut s, 0x7ff, &[0xff; 8]));
        assert!(!decode(&mut t, &mut s, 0x000, &[]));
        assert_eq!(format!("{t:?}"), format!("{:?}", Telemetry::default()));
    }
}
