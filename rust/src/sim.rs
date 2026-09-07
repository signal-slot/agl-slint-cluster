// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! The built-in demo source: a repeating drive cycle, with every reading
//! derived from one speed trace the way the real signals relate to each other.
//! This is the default `TelemetrySource`; swap it out at `select_source` in
//! `main.rs`. It never touches the UI -- it only produces `Telemetry`.

use crate::telemetry::{Beam, Cruise, Lane, Telemetry, TelemetrySource, Warn};

/// Length of one drive cycle, seconds.
const CYCLE: f32 = 66.0;

pub struct SimSource {
    /// Seconds since the cycle started.
    t: f32,
    /// Speed actually shown, low-pass filtered so the needle never jumps.
    speed: f32,
    /// Distance travelled this cycle, km.
    trip_km: f32,
    odo_km: f32,
}

impl Default for SimSource {
    fn default() -> Self {
        Self::new()
    }
}

impl SimSource {
    pub fn new() -> Self {
        Self {
            t: 0.0,
            speed: 0.0,
            trip_km: 0.0,
            odo_km: 12_345.0,
        }
    }

    /// Target speed at time `t`, as a piecewise trace: pull away, cruise, slow
    /// for a junction, accelerate onto a faster road, then come to a stop.
    fn target_speed(t: f32) -> f32 {
        let t = t % CYCLE;
        // (until, speed at that point) -- linear between the points.
        const TRACE: &[(f32, f32)] = &[
            (0.0, 0.0),
            (11.0, 62.0),
            (22.0, 68.0),
            (28.0, 34.0),
            (34.0, 30.0),
            (45.0, 118.0),
            (52.0, 104.0),
            (61.0, 0.0),
            (CYCLE, 0.0),
        ];
        let mut prev = TRACE[0];
        for &pt in &TRACE[1..] {
            if t <= pt.0 {
                let span = pt.0 - prev.0;
                let k = if span > 0.0 { (t - prev.0) / span } else { 0.0 };
                // Smoothstep so the transitions are not visibly linear.
                let k = k * k * (3.0 - 2.0 * k);
                return prev.1 + (pt.1 - prev.1) * k;
            }
            prev = pt;
        }
        0.0
    }

    /// Acceleration in km/h per second, used to drive the power/charge meter.
    fn accel(&self) -> f32 {
        (Self::target_speed(self.t + 0.25) - Self::target_speed(self.t - 0.25)) / 0.5
    }
}

impl TelemetrySource for SimSource {
    fn poll(&mut self, dt: f32) -> Telemetry {
        self.t += dt;
        let target = Self::target_speed(self.t);
        // First-order lag: the shown speed chases the trace.
        self.speed += (target - self.speed) * (1.0 - (-dt * 2.5).exp());
        let km = self.speed * dt / 3600.0;
        self.trip_km += km;
        self.odo_km += km;

        let a = self.accel();
        // Remaining range and charge fall as the trip goes on.
        let used = self.trip_km / 560.0;
        // Efficiency drops while accelerating (spending energy) and rises while
        // regenerating (recovering it), so the readout tracks the power meter in
        // both directions instead of sitting at the baseline during regen.
        let eff = 85.0 - a.max(0.0) * 1.6 + (-a).max(0.0) * 1.4;

        // A healthy vehicle: headlights on, lane keeping tracking, cruise held
        // on the open stretches, and the indicator blinking as the junction
        // nears. The hazard/warning lamps stay off. A real source lights them.
        let to_turn = (2.4 - (self.t % CYCLE) / CYCLE * 2.4).max(0.0);
        let cruising = self.speed > 55.0 && a.abs() < 3.0;

        Telemetry {
            speed_kmh: self.speed,
            // Idle sits a little above the middle of the sweep; the UI maps this
            // signed power to the left dial's REGEN..CHARGE..POWER angle.
            power: (a / 9.0).clamp(-1.0, 1.0),
            efficiency_percent: eff.clamp(40.0, 99.0),
            avg_efficiency_kwh: (4.8 - a.max(0.0) * 0.06).max(2.0),
            range_km: (560.0 - self.trip_km).max(0.0),
            battery_level: (0.85 - used * 0.6).clamp(0.05, 1.0),
            fuel_level: (0.55 - used * 0.4).clamp(0.02, 1.0),
            outside_temp_c: 21,
            odo_km: self.odo_km,
            trip_seconds: self.t as u32,
            // Distance to the turn counts down and repeats with the cycle.
            turn_distance_km: to_turn,
            turn_street: "Minatomirai Blvd".into(),
            cruise_speed_kmh: 80.0,
            lta_on: true,
            lca_on: true,
            gear: 'D',
            ready: true,
            turn_left: false,
            turn_right: to_turn > 0.05 && to_turn < 1.2,
            headlights: Beam::Low,
            engine: Warn::Off,
            oil: Warn::Off,
            battery_warn: Warn::Off,
            abs: Warn::Off,
            parking_brake: false,
            seatbelt: false,
            door_open: false,
            cruise: if cruising {
                Cruise::Active
            } else {
                Cruise::Off
            },
            lane: Lane::Tracking,
        }
    }
}
