// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! The boundary between where the numbers come from and how the cluster shows
//! them. Everything upstream produces a `Telemetry` frame; the UI layer (see
//! `apply` in `main.rs`) is the only place that turns it into widgets.
//!
//! To feed the cluster from a real vehicle bus (CAN / VSS / DDS) or an emulator
//! instead of the built-in simulation, implement `TelemetrySource` and return
//! it from `select_source` in `main.rs`. Nothing else changes: the UI still
//! only reads properties, and the mapping in `apply` stays put.

/// Headlight state.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(dead_code)] // full lamp API for real sources; the sim uses a subset
pub enum Beam {
    Off,
    Low,
    High,
}

/// A warning lamp with the usual two escalation colours.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(dead_code)] // full lamp API for real sources; the sim uses a subset
pub enum Warn {
    Off,
    Amber,
    Red,
}

/// Cruise-control lamp.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(dead_code)] // full lamp API for real sources; the sim uses a subset
pub enum Cruise {
    Off,
    Set,
    Active,
}

/// Lane-keeping lamp.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(dead_code)] // full lamp API for real sources; the sim uses a subset
pub enum Lane {
    Off,
    Tracking,
    Warn,
    Depart,
}

/// One frame of vehicle signals, in physical / semantic units -- never display
/// strings. A bus reader or a replay file fills exactly this struct.
#[derive(Clone, Debug)]
pub struct Telemetry {
    /// Road speed, km/h.
    pub speed_kmh: f32,
    /// Energy flow right now: -1.0 (full regen / charge) .. +1.0 (full power).
    pub power: f32,
    /// Instantaneous drive efficiency, 0..100 %.
    pub efficiency_percent: f32,
    /// Average efficiency, km/kWh.
    pub avg_efficiency_kwh: f32,
    /// Remaining range, km.
    pub range_km: f32,
    /// Traction battery charge, 0.0..1.0.
    pub battery_level: f32,
    /// Fuel remaining, 0.0..1.0.
    pub fuel_level: f32,
    /// Outside temperature, deg C.
    pub outside_temp_c: i32,
    /// Odometer, km.
    pub odo_km: f32,
    /// Time elapsed on this trip, seconds.
    pub trip_seconds: u32,
    /// Distance to the next navigation turn, km.
    pub turn_distance_km: f32,
    /// Name of the street at the next turn.
    pub turn_street: String,
    /// Cruise-control set speed, km/h.
    pub cruise_speed_kmh: f32,
    /// Lane Tracing Assist active.
    pub lta_on: bool,
    /// Lane Change Assist active.
    pub lca_on: bool,
    /// Selected gear: 'P', 'R', 'N', 'D'.
    pub gear: char,
    /// Drivetrain ready to move.
    pub ready: bool,

    // --- Telltale lamps (top band). Off states leave the band clean. ---
    /// Left turn signal requested.
    pub turn_left: bool,
    /// Right turn signal requested.
    pub turn_right: bool,
    /// Headlights.
    pub headlights: Beam,
    /// Engine / MIL.
    pub engine: Warn,
    /// Oil pressure.
    pub oil: Warn,
    /// 12 V / charging system.
    pub battery_warn: Warn,
    /// Anti-lock brakes.
    pub abs: Warn,
    /// Parking brake engaged (red).
    pub parking_brake: bool,
    /// Seatbelt unfastened (red).
    pub seatbelt: bool,
    /// A door is open (red).
    pub door_open: bool,
    /// Cruise control.
    pub cruise: Cruise,
    /// Lane keeping.
    pub lane: Lane,
}

impl Default for Telemetry {
    /// A quiet, stationary vehicle -- the values the UI falls back to before the
    /// first real frame arrives.
    fn default() -> Self {
        Self {
            speed_kmh: 0.0,
            power: 0.0,
            efficiency_percent: 85.0,
            avg_efficiency_kwh: 4.8,
            range_km: 560.0,
            battery_level: 0.85,
            fuel_level: 0.55,
            outside_temp_c: 21,
            odo_km: 12_345.0,
            trip_seconds: 0,
            turn_distance_km: 2.4,
            turn_street: "Minatomirai Blvd".into(),
            cruise_speed_kmh: 80.0,
            lta_on: true,
            lca_on: true,
            gear: 'D',
            ready: true,
            turn_left: false,
            turn_right: false,
            headlights: Beam::Off,
            engine: Warn::Off,
            oil: Warn::Off,
            battery_warn: Warn::Off,
            abs: Warn::Off,
            parking_brake: false,
            seatbelt: false,
            door_open: false,
            cruise: Cruise::Off,
            lane: Lane::Off,
        }
    }
}

/// A source of cluster signals.
///
/// `poll` is called on the UI thread from the render timer (~20 ms), so it
/// **must not block**: no synchronous socket reads, no I/O waits. A real
/// bus-backed source (CAN / VSS / KUKSA) should run its own background thread
/// and have `poll` return the latest frame it has received (e.g. cloned from a
/// shared cell or drained from a channel), ignoring `dt`.
pub trait TelemetrySource {
    /// Advance by `dt` seconds (wall time since the last call) and return the
    /// signals to show now.
    fn poll(&mut self, dt: f32) -> Telemetry;
}
