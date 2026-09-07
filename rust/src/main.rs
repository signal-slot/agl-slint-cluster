// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

use std::error::Error;
use std::time::Duration;

use agl_slint_cluster::can::CanSource;
use agl_slint_cluster::kuksa::KuksaSource;
use agl_slint_cluster::sim::SimSource;
use agl_slint_cluster::telemetry::{Beam, Cruise, Lane, Telemetry, TelemetrySource, Warn};

slint::include_modules!();

// Exactly one backend feature must be selected; they pull in conflicting
// backends. `desktop` is the default, so device/qt builds need
// `--no-default-features --features <name>`.
#[cfg(any(
    all(feature = "desktop", feature = "device"),
    all(feature = "desktop", feature = "qt"),
    all(feature = "device", feature = "qt"),
))]
compile_error!(
    "enable only one of the `desktop`, `device`, or `qt` features (use --no-default-features)"
);

/// Push one telemetry frame into the UI. This is the ONLY place that turns
/// signals into display values -- string formatting, the dial-angle mapping and
/// the wall clock all live here -- so a new data source never has to know about
/// widgets. It just fills `Telemetry`.
fn apply(ui: &AppWindow, t: &Telemetry, blink: bool) {
    ui.set_speed(t.speed_kmh);
    // Left dial reads REGEN .. CHARGE .. POWER; idle sits a little above centre.
    ui.set_charge_angle((-6.0 + t.power * 118.0).clamp(-138.0, 138.0));
    ui.set_efficiency_percent(t.efficiency_percent);
    ui.set_avg_efficiency(format!("{:.1}", t.avg_efficiency_kwh).into());
    ui.set_range_km(format!("{}", t.range_km as u32).into());
    ui.set_battery_level(t.battery_level);
    ui.set_fuel_level(t.fuel_level);
    ui.set_outside_temp(format!("{}", t.outside_temp_c).into());
    ui.set_odo(group_thousands(t.odo_km as u32).into());
    ui.set_drive_time(format!("{:02}:{:02}", t.trip_seconds / 60, t.trip_seconds % 60).into());
    ui.set_turn_distance(format!("{:.1}", t.turn_distance_km).into());
    ui.set_turn_street(t.turn_street.clone().into());
    ui.set_cruise_speed(format!("{}", t.cruise_speed_kmh as u32).into());
    ui.set_lta_on(t.lta_on);
    ui.set_lca_on(t.lca_on);
    ui.set_gear(t.gear.to_string().into());
    ui.set_ready(t.ready);

    // Telltale band. Each lamp's level indexes its ISO-colour variants (0 off);
    // turn arrows carry the already-blinked state. See app-window.slint.
    ui.set_turn_left_on(t.turn_left && blink);
    ui.set_turn_right_on(t.turn_right && blink);
    ui.set_tt_lights(match t.headlights {
        Beam::Off => 0,
        Beam::Low => 1,
        Beam::High => 2,
    });
    ui.set_tt_engine(warn_level(t.engine));
    ui.set_tt_oil(warn_level(t.oil));
    ui.set_tt_battery(warn_level(t.battery_warn));
    ui.set_tt_abs(warn_level(t.abs));
    ui.set_tt_parking_brake(t.parking_brake as i32);
    ui.set_tt_seatbelt(t.seatbelt as i32);
    ui.set_tt_door(t.door_open as i32);
    ui.set_tt_cruise(match t.cruise {
        Cruise::Off => 0,
        Cruise::Set => 1,
        Cruise::Active => 2,
    });
    ui.set_tt_lane(match t.lane {
        Lane::Off => 0,
        Lane::Tracking => 1,
        Lane::Warn => 2,
        Lane::Depart => 3,
    });

    // The clock is wall time, independent of the vehicle signals.
    ui.set_time(chrono::Local::now().format("%H:%M").to_string().into());
}

/// Map a two-stage warning lamp to its variant index (0 off, 1 amber, 2 red).
fn warn_level(w: Warn) -> i32 {
    match w {
        Warn::Off => 0,
        Warn::Amber => 1,
        Warn::Red => 2,
    }
}

/// Group an integer with thousands separators, as the odometer shows them.
fn group_thousands(n: u32) -> String {
    let s = n.to_string();
    let mut out = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    out
}

/// Choose where the cluster's signals come from. `CLUSTER_SOURCE` selects at
/// runtime and defaults to the built-in simulation:
///
/// - `sim` -- the built-in drive cycle (`sim.rs`).
/// - `can` or `can:<iface>` -- frames from a SocketCAN interface (`can.rs`),
///   `can0` unless named; `can:vcan0` pairs with the `cansim` binary.
/// - `kuksa` or `kuksa:<url>` -- VSS signals from a Kuksa databroker
///   (`kuksa.rs`), `http://127.0.0.1:55555` unless named; pairs with the
///   `kuksasim` binary.
///
/// To add another (VSS / KUKSA, a replay file, ...), implement
/// `TelemetrySource` (see `telemetry.rs`) and add an arm here. A fallible
/// source should surface its own error (or fall back) inside its constructor;
/// `select_source` returns an infallible source.
fn select_source() -> Box<dyn TelemetrySource> {
    let var = std::env::var("CLUSTER_SOURCE").unwrap_or_default();
    let (kind, arg) = var.split_once(':').unwrap_or((var.as_str(), ""));
    match (kind, arg) {
        ("sim", _) | ("", _) => Box::new(SimSource::new()),
        ("can", "") => Box::new(CanSource::open("can0")),
        ("can", iface) => Box::new(CanSource::open(iface)),
        ("kuksa", "") => Box::new(KuksaSource::connect("http://127.0.0.1:55555")),
        ("kuksa", url) => Box::new(KuksaSource::connect(url)),
        _ => {
            eprintln!("agl-slint-cluster: unknown CLUSTER_SOURCE={var:?}, using the simulation");
            Box::new(SimSource::new())
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let ui = AppWindow::new()?;

    let mut source = select_source();
    let weak = ui.as_weak();
    let timer = slint::Timer::default();
    // 20 Hz is plenty: the UI interpolates between updates.
    let period = Duration::from_millis(50);
    let dt = period.as_secs_f32();

    // Turn-signal blink: ~1.25 Hz, resolved here so the UI stays declarative.
    let mut ticks: u64 = 0;
    timer.start(slint::TimerMode::Repeated, period, move || {
        let Some(ui) = weak.upgrade() else { return };
        let frame = source.poll(dt);
        let blink = (ticks / 8) % 2 == 0;
        apply(&ui, &frame, blink);
        ticks += 1;
    });

    ui.run()?;
    Ok(())
}
