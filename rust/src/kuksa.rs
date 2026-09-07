// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! The cluster as a client of the Eclipse Kuksa databroker: `KuksaSource`
//! subscribes to a fixed set of VSS (Vehicle Signal Specification) paths over
//! gRPC (`kuksa.val.v2`) and folds every update into `Telemetry`. The mapping
//! from VSS to `Telemetry` and back lives here, in `apply` and `signals`, so
//! the `kuksasim` binary can publish the built-in simulation with the same
//! table the cluster reads with.
//!
//! The paths exist unchanged in VSS 4.0, 5.0 and 5.1 (see docs/kuksa.md).
//! A few `Telemetry` fields have no VSS counterpart (instantaneous
//! efficiency, the next-turn street and distance, Lane Change Assist); those
//! keep their default and are listed there too.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::telemetry::{Beam, Cruise, Lane, Telemetry, TelemetrySource, Warn};

/// Generated `kuksa.val.v2` types and the `VAL` client.
#[allow(clippy::all)]
pub mod proto {
    tonic::include_proto!("kuksa.val.v2");
}

use proto::value::TypedValue;
use proto::{Datapoint, Value};

// ----- the mapping -----------------------------------------------------------

/// Motor power is normalised against `MaxPower`; this is used until the
/// vehicle has reported one.
const DEFAULT_MAX_POWER_KW: f64 = 150.0;

/// Every VSS path the cluster subscribes to, in one place.
pub const PATHS: &[&str] = &[
    "Vehicle.Speed",
    "Vehicle.TraveledDistance",
    "Vehicle.TripDuration",
    "Vehicle.Exterior.AirTemperature",
    "Vehicle.LowVoltageSystemState",
    "Vehicle.LowVoltageBattery.CurrentVoltage",
    "Vehicle.Powertrain.Range",
    "Vehicle.Powertrain.TractionBattery.StateOfCharge.Current",
    "Vehicle.Powertrain.FuelSystem.RelativeLevel",
    "Vehicle.Powertrain.ElectricMotor.Power",
    "Vehicle.Powertrain.ElectricMotor.MaxPower",
    "Vehicle.Powertrain.Transmission.SelectedGear",
    "Vehicle.Powertrain.CombustionEngine.EngineOilLevel",
    "Vehicle.OBD.Status.IsMILOn",
    "Vehicle.ADAS.CruiseControl.IsEnabled",
    "Vehicle.ADAS.CruiseControl.IsActive",
    "Vehicle.ADAS.CruiseControl.SpeedSet",
    "Vehicle.ADAS.LaneDepartureDetection.IsEnabled",
    "Vehicle.ADAS.LaneDepartureDetection.IsWarning",
    "Vehicle.ADAS.ABS.IsError",
    "Vehicle.Body.Lights.DirectionIndicator.Left.IsSignaling",
    "Vehicle.Body.Lights.DirectionIndicator.Right.IsSignaling",
    "Vehicle.Body.Lights.Beam.Low.IsOn",
    "Vehicle.Body.Lights.Beam.High.IsOn",
    "Vehicle.Chassis.ParkingBrake.IsEngaged",
    "Vehicle.Cabin.Seat.Row1.DriverSide.IsBelted",
    "Vehicle.Cabin.Door.Row1.DriverSide.IsOpen",
    "Vehicle.Cabin.Door.Row1.PassengerSide.IsOpen",
    "Vehicle.Cabin.Door.Row2.DriverSide.IsOpen",
    "Vehicle.Cabin.Door.Row2.PassengerSide.IsOpen",
];

/// The raw signals that several `Telemetry` fields are derived from, kept so
/// an update to one of them can recompute the field.
#[derive(Default)]
pub struct Signals {
    motor_power_kw: f64,
    max_power_kw: Option<f64>,
    cruise_enabled: bool,
    cruise_active: bool,
    lane_enabled: bool,
    lane_warning: bool,
    low_beam: bool,
    high_beam: bool,
    doors_open: [bool; 4],
}

fn as_f64(v: &TypedValue) -> Option<f64> {
    Some(match *v {
        TypedValue::Float(x) => x as f64,
        TypedValue::Double(x) => x,
        TypedValue::Int32(x) => x as f64,
        TypedValue::Int64(x) => x as f64,
        TypedValue::Uint32(x) => x as f64,
        TypedValue::Uint64(x) => x as f64,
        _ => return None,
    })
}

fn as_bool(v: &TypedValue) -> Option<bool> {
    match v {
        TypedValue::Bool(b) => Some(*b),
        _ => None,
    }
}

fn as_str(v: &TypedValue) -> Option<&str> {
    match v {
        TypedValue::String(s) => Some(s),
        _ => None,
    }
}

/// Fold one VSS update into `t`. Returns `false` if the path is not one the
/// cluster uses or the value has the wrong type; `t` is then untouched.
pub fn apply(t: &mut Telemetry, sig: &mut Signals, path: &str, v: &TypedValue) -> bool {
    macro_rules! num {
        () => {
            match as_f64(v) {
                Some(x) => x,
                None => return false,
            }
        };
    }
    macro_rules! flag {
        () => {
            match as_bool(v) {
                Some(b) => b,
                None => return false,
            }
        };
    }
    match path {
        "Vehicle.Speed" => t.speed_kmh = num!() as f32,
        "Vehicle.TraveledDistance" => t.odo_km = num!() as f32,
        "Vehicle.TripDuration" => t.trip_seconds = num!().max(0.0) as u32,
        "Vehicle.Exterior.AirTemperature" => t.outside_temp_c = num!().round() as i32,
        "Vehicle.LowVoltageSystemState" => {
            let Some(s) = as_str(v) else { return false };
            t.ready = matches!(s, "ON" | "START");
        }
        "Vehicle.LowVoltageBattery.CurrentVoltage" => {
            let volts = num!();
            t.battery_warn = if volts < 11.0 {
                Warn::Red
            } else if volts < 11.8 {
                Warn::Amber
            } else {
                Warn::Off
            };
        }
        "Vehicle.Powertrain.Range" => t.range_km = (num!() / 1000.0) as f32,
        "Vehicle.Powertrain.TractionBattery.StateOfCharge.Current" => {
            t.battery_level = (num!() / 100.0).clamp(0.0, 1.0) as f32
        }
        "Vehicle.Powertrain.FuelSystem.RelativeLevel" => {
            t.fuel_level = (num!() / 100.0).clamp(0.0, 1.0) as f32
        }
        "Vehicle.Powertrain.ElectricMotor.Power" => {
            sig.motor_power_kw = num!();
            t.power = power(sig);
        }
        "Vehicle.Powertrain.ElectricMotor.MaxPower" => {
            let max = num!();
            sig.max_power_kw = (max > 0.0).then_some(max);
            t.power = power(sig);
        }
        "Vehicle.Powertrain.Transmission.SelectedGear" => {
            t.gear = match num!() as i64 {
                126 => 'P',
                127 => 'D',
                0 => 'N',
                g if g < 0 => 'R',
                _ => 'D',
            }
        }
        "Vehicle.Powertrain.CombustionEngine.EngineOilLevel" => {
            let Some(s) = as_str(v) else { return false };
            t.oil = match s {
                "CRITICALLY_LOW" => Warn::Red,
                "LOW" => Warn::Amber,
                _ => Warn::Off,
            };
        }
        "Vehicle.OBD.Status.IsMILOn" => t.engine = if flag!() { Warn::Amber } else { Warn::Off },
        "Vehicle.ADAS.CruiseControl.IsEnabled" => {
            sig.cruise_enabled = flag!();
            t.cruise = cruise(sig);
        }
        "Vehicle.ADAS.CruiseControl.IsActive" => {
            sig.cruise_active = flag!();
            t.cruise = cruise(sig);
        }
        "Vehicle.ADAS.CruiseControl.SpeedSet" => t.cruise_speed_kmh = num!() as f32,
        "Vehicle.ADAS.LaneDepartureDetection.IsEnabled" => {
            sig.lane_enabled = flag!();
            t.lta_on = sig.lane_enabled;
            t.lane = lane(sig);
        }
        "Vehicle.ADAS.LaneDepartureDetection.IsWarning" => {
            sig.lane_warning = flag!();
            t.lane = lane(sig);
        }
        "Vehicle.ADAS.ABS.IsError" => t.abs = if flag!() { Warn::Red } else { Warn::Off },
        "Vehicle.Body.Lights.DirectionIndicator.Left.IsSignaling" => t.turn_left = flag!(),
        "Vehicle.Body.Lights.DirectionIndicator.Right.IsSignaling" => t.turn_right = flag!(),
        "Vehicle.Body.Lights.Beam.Low.IsOn" => {
            sig.low_beam = flag!();
            t.headlights = beam(sig);
        }
        "Vehicle.Body.Lights.Beam.High.IsOn" => {
            sig.high_beam = flag!();
            t.headlights = beam(sig);
        }
        "Vehicle.Chassis.ParkingBrake.IsEngaged" => t.parking_brake = flag!(),
        // The lamp shows an *unfastened* belt.
        "Vehicle.Cabin.Seat.Row1.DriverSide.IsBelted" => t.seatbelt = !flag!(),
        "Vehicle.Cabin.Door.Row1.DriverSide.IsOpen" => door(t, sig, 0, flag!()),
        "Vehicle.Cabin.Door.Row1.PassengerSide.IsOpen" => door(t, sig, 1, flag!()),
        "Vehicle.Cabin.Door.Row2.DriverSide.IsOpen" => door(t, sig, 2, flag!()),
        "Vehicle.Cabin.Door.Row2.PassengerSide.IsOpen" => door(t, sig, 3, flag!()),
        _ => return false,
    }
    true
}

fn power(sig: &Signals) -> f32 {
    let max = sig.max_power_kw.unwrap_or(DEFAULT_MAX_POWER_KW);
    (sig.motor_power_kw / max).clamp(-1.0, 1.0) as f32
}

fn cruise(sig: &Signals) -> Cruise {
    if sig.cruise_active {
        Cruise::Active
    } else if sig.cruise_enabled {
        Cruise::Set
    } else {
        Cruise::Off
    }
}

fn lane(sig: &Signals) -> Lane {
    if !sig.lane_enabled {
        Lane::Off
    } else if sig.lane_warning {
        Lane::Warn
    } else {
        Lane::Tracking
    }
}

fn beam(sig: &Signals) -> Beam {
    if sig.high_beam {
        Beam::High
    } else if sig.low_beam {
        Beam::Low
    } else {
        Beam::Off
    }
}

fn door(t: &mut Telemetry, sig: &mut Signals, i: usize, open: bool) {
    sig.doors_open[i] = open;
    t.door_open = sig.doors_open.iter().any(|&d| d);
}

/// The VSS signals that describe `t`, typed as the databroker's VSS metadata
/// expects them (float, uint32 for uint8/16/32, int32 for int8/16, bool,
/// string). This is the publishing half of the mapping, used by `kuksasim`.
pub fn signals(t: &Telemetry) -> Vec<(&'static str, TypedValue)> {
    use TypedValue::*;
    let gear = match t.gear {
        'P' => 126,
        'R' => -1,
        'N' => 0,
        _ => 127,
    };
    let oil = match t.oil {
        Warn::Red => "CRITICALLY_LOW",
        Warn::Amber => "LOW",
        Warn::Off => "NORMAL",
    };
    let volts = match t.battery_warn {
        Warn::Red => 10.5,
        Warn::Amber => 11.5,
        Warn::Off => 12.6,
    };
    vec![
        ("Vehicle.Speed", Float(t.speed_kmh)),
        ("Vehicle.TraveledDistance", Float(t.odo_km)),
        ("Vehicle.TripDuration", Float(t.trip_seconds as f32)),
        (
            "Vehicle.Exterior.AirTemperature",
            Float(t.outside_temp_c as f32),
        ),
        (
            "Vehicle.LowVoltageSystemState",
            String(if t.ready { "ON" } else { "ACC" }.into()),
        ),
        ("Vehicle.LowVoltageBattery.CurrentVoltage", Float(volts)),
        (
            "Vehicle.Powertrain.Range",
            Uint32((t.range_km * 1000.0).max(0.0) as u32),
        ),
        (
            "Vehicle.Powertrain.TractionBattery.StateOfCharge.Current",
            Float(t.battery_level * 100.0),
        ),
        (
            "Vehicle.Powertrain.FuelSystem.RelativeLevel",
            Uint32((t.fuel_level * 100.0).round() as u32),
        ),
        (
            "Vehicle.Powertrain.ElectricMotor.MaxPower",
            Uint32(DEFAULT_MAX_POWER_KW as u32),
        ),
        (
            "Vehicle.Powertrain.ElectricMotor.Power",
            Int32((t.power as f64 * DEFAULT_MAX_POWER_KW).round() as i32),
        ),
        ("Vehicle.Powertrain.Transmission.SelectedGear", Int32(gear)),
        (
            "Vehicle.Powertrain.CombustionEngine.EngineOilLevel",
            String(oil.into()),
        ),
        ("Vehicle.OBD.Status.IsMILOn", Bool(t.engine != Warn::Off)),
        (
            "Vehicle.ADAS.CruiseControl.IsEnabled",
            Bool(t.cruise != Cruise::Off),
        ),
        (
            "Vehicle.ADAS.CruiseControl.IsActive",
            Bool(t.cruise == Cruise::Active),
        ),
        (
            "Vehicle.ADAS.CruiseControl.SpeedSet",
            Float(t.cruise_speed_kmh),
        ),
        (
            "Vehicle.ADAS.LaneDepartureDetection.IsEnabled",
            Bool(t.lane != Lane::Off),
        ),
        (
            "Vehicle.ADAS.LaneDepartureDetection.IsWarning",
            Bool(matches!(t.lane, Lane::Warn | Lane::Depart)),
        ),
        ("Vehicle.ADAS.ABS.IsError", Bool(t.abs != Warn::Off)),
        (
            "Vehicle.Body.Lights.DirectionIndicator.Left.IsSignaling",
            Bool(t.turn_left),
        ),
        (
            "Vehicle.Body.Lights.DirectionIndicator.Right.IsSignaling",
            Bool(t.turn_right),
        ),
        (
            "Vehicle.Body.Lights.Beam.Low.IsOn",
            Bool(t.headlights == Beam::Low),
        ),
        (
            "Vehicle.Body.Lights.Beam.High.IsOn",
            Bool(t.headlights == Beam::High),
        ),
        (
            "Vehicle.Chassis.ParkingBrake.IsEngaged",
            Bool(t.parking_brake),
        ),
        (
            "Vehicle.Cabin.Seat.Row1.DriverSide.IsBelted",
            Bool(!t.seatbelt),
        ),
        (
            "Vehicle.Cabin.Door.Row1.DriverSide.IsOpen",
            Bool(t.door_open),
        ),
    ]
}

/// Wrap a value as a databroker `Datapoint` (timestamp left to the server).
pub fn datapoint(v: TypedValue) -> Datapoint {
    Datapoint {
        timestamp: None,
        value: Some(Value {
            typed_value: Some(v),
        }),
    }
}

// ----- the source ------------------------------------------------------------

/// A `TelemetrySource` fed by a Kuksa databroker subscription.
///
/// A background thread runs the gRPC client and folds every update into a
/// shared `Telemetry`; `poll` clones the latest state and never blocks. The
/// connection is retried whenever the databroker is unreachable or the stream
/// ends, so the cluster can start before the broker.
pub struct KuksaSource {
    latest: Arc<Mutex<Telemetry>>,
}

impl KuksaSource {
    /// Subscribe to `url` (e.g. `"http://127.0.0.1:55555"`). Never fails: a
    /// broker that cannot be reached is retried in the background.
    pub fn connect(url: &str) -> Self {
        let latest = Arc::new(Mutex::new(Telemetry::default()));
        let shared = Arc::clone(&latest);
        let url = url.to_owned();
        thread::Builder::new()
            .name("kuksa-rx".into())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("tokio runtime");
                rt.block_on(rx_loop(&url, &shared));
            })
            .expect("spawn Kuksa client thread");
        Self { latest }
    }
}

impl TelemetrySource for KuksaSource {
    fn poll(&mut self, _dt: f32) -> Telemetry {
        self.latest
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

async fn rx_loop(url: &str, shared: &Mutex<Telemetry>) {
    let mut sig = Signals::default();
    let mut last_err: Option<String> = None;
    loop {
        match subscribe(url, shared, &mut sig).await {
            Ok(()) => {
                eprintln!("agl-slint-cluster: {url}: subscription ended; reconnecting");
                last_err = None;
            }
            Err(e) => {
                // Log each distinct failure once, not once a second.
                let msg = e.to_string();
                if last_err.as_deref() != Some(&msg) {
                    eprintln!("agl-slint-cluster: {url}: {msg}; retrying");
                    last_err = Some(msg);
                }
            }
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}

async fn subscribe(
    url: &str,
    shared: &Mutex<Telemetry>,
    sig: &mut Signals,
) -> Result<(), tonic::Status> {
    let channel = tonic::transport::Endpoint::from_shared(url.to_owned())
        .map_err(|e| tonic::Status::invalid_argument(e.to_string()))?
        .connect_timeout(Duration::from_secs(3))
        .connect()
        .await
        .map_err(|e| tonic::Status::unavailable(e.to_string()))?;
    let mut client = proto::val_client::ValClient::new(channel);
    let request = proto::SubscribeRequest {
        signal_paths: PATHS.iter().map(|p| p.to_string()).collect(),
        buffer_size: 0,
        filter: None,
    };
    let mut stream = client.subscribe(request).await?.into_inner();
    eprintln!(
        "agl-slint-cluster: subscribed to {} VSS signals at {url}",
        PATHS.len()
    );
    while let Some(update) = stream.message().await? {
        let mut t = shared.lock().unwrap_or_else(|e| e.into_inner());
        for (path, dp) in &update.entries {
            if let Some(v) = dp.value.as_ref().and_then(|v| v.typed_value.as_ref()) {
                apply(&mut t, sig, path, v);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(t: &Telemetry) -> Telemetry {
        let mut out = Telemetry::default();
        let mut sig = Signals::default();
        for (path, v) in signals(t) {
            assert!(PATHS.contains(&path), "{path} is not subscribed");
            assert!(apply(&mut out, &mut sig, path, &v), "{path} rejected");
        }
        out
    }

    #[test]
    fn every_mapped_field_roundtrips() {
        let t = Telemetry {
            speed_kmh: 123.45,
            power: -0.4,
            range_km: 321.0,
            battery_level: 0.42,
            fuel_level: 0.07,
            outside_temp_c: -12,
            odo_km: 98_765.4,
            trip_seconds: 3_723,
            cruise_speed_kmh: 100.0,
            lta_on: true,
            gear: 'R',
            ready: false,
            turn_left: true,
            headlights: Beam::High,
            engine: Warn::Amber,
            oil: Warn::Red,
            battery_warn: Warn::Amber,
            abs: Warn::Red,
            parking_brake: true,
            seatbelt: true,
            door_open: true,
            cruise: Cruise::Set,
            lane: Lane::Warn,
            ..Telemetry::default()
        };
        let r = roundtrip(&t);
        assert!((r.speed_kmh - t.speed_kmh).abs() < 1e-3);
        assert!((r.power - t.power).abs() < 0.01);
        assert!((r.range_km - t.range_km).abs() < 1e-3);
        assert!((r.battery_level - t.battery_level).abs() < 1e-3);
        assert!((r.fuel_level - t.fuel_level).abs() < 0.006);
        assert_eq!(r.outside_temp_c, t.outside_temp_c);
        assert!((r.odo_km - t.odo_km).abs() < 0.01);
        assert_eq!(r.trip_seconds, t.trip_seconds);
        assert_eq!(r.cruise_speed_kmh, t.cruise_speed_kmh);
        assert_eq!(r.lta_on, t.lta_on);
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
    fn numeric_signals_accept_any_numeric_encoding() {
        let mut t = Telemetry::default();
        let mut s = Signals::default();
        assert!(apply(
            &mut t,
            &mut s,
            "Vehicle.Speed",
            &TypedValue::Double(50.0)
        ));
        assert_eq!(t.speed_kmh, 50.0);
        assert!(apply(
            &mut t,
            &mut s,
            "Vehicle.Powertrain.Range",
            &TypedValue::Uint32(250_000)
        ));
        assert_eq!(t.range_km, 250.0);
        assert!(!apply(
            &mut t,
            &mut s,
            "Vehicle.Speed",
            &TypedValue::String("fast".into())
        ));
        assert_eq!(t.speed_kmh, 50.0);
    }

    #[test]
    fn derived_fields_follow_their_last_inputs() {
        let mut t = Telemetry::default();
        let mut s = Signals::default();
        // Any open door lights the lamp; it stays lit until all are closed.
        apply(
            &mut t,
            &mut s,
            "Vehicle.Cabin.Door.Row2.PassengerSide.IsOpen",
            &TypedValue::Bool(true),
        );
        apply(
            &mut t,
            &mut s,
            "Vehicle.Cabin.Door.Row1.DriverSide.IsOpen",
            &TypedValue::Bool(true),
        );
        apply(
            &mut t,
            &mut s,
            "Vehicle.Cabin.Door.Row1.DriverSide.IsOpen",
            &TypedValue::Bool(false),
        );
        assert!(t.door_open);
        apply(
            &mut t,
            &mut s,
            "Vehicle.Cabin.Door.Row2.PassengerSide.IsOpen",
            &TypedValue::Bool(false),
        );
        assert!(!t.door_open);
        // Power is normalised by MaxPower once it is known.
        apply(
            &mut t,
            &mut s,
            "Vehicle.Powertrain.ElectricMotor.Power",
            &TypedValue::Int32(-75),
        );
        assert_eq!(t.power, -0.5);
        apply(
            &mut t,
            &mut s,
            "Vehicle.Powertrain.ElectricMotor.MaxPower",
            &TypedValue::Uint32(300),
        );
        assert_eq!(t.power, -0.25);
        // High beam wins over low beam while both are on.
        apply(
            &mut t,
            &mut s,
            "Vehicle.Body.Lights.Beam.Low.IsOn",
            &TypedValue::Bool(true),
        );
        apply(
            &mut t,
            &mut s,
            "Vehicle.Body.Lights.Beam.High.IsOn",
            &TypedValue::Bool(true),
        );
        assert_eq!(t.headlights, Beam::High);
        apply(
            &mut t,
            &mut s,
            "Vehicle.Body.Lights.Beam.High.IsOn",
            &TypedValue::Bool(false),
        );
        assert_eq!(t.headlights, Beam::Low);
    }

    #[test]
    fn unknown_paths_are_ignored() {
        let mut t = Telemetry::default();
        let mut s = Signals::default();
        assert!(!apply(
            &mut t,
            &mut s,
            "Vehicle.Cabin.Sunroof.Position",
            &TypedValue::Int32(3)
        ));
        assert_eq!(format!("{t:?}"), format!("{:?}", Telemetry::default()));
    }
}
