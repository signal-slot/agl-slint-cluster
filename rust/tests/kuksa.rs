// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! End-to-end against a live Kuksa databroker: publish a few VSS values with
//! the generated client and watch them arrive through `KuksaSource`. Skips
//! (passes) when no broker answers on 127.0.0.1:55555; start one with
//! `docker run --rm -p 55555:55555 ghcr.io/eclipse-kuksa/kuksa-databroker:0.6.0 --insecure`.
//! Stop `kuksasim` first: it keeps overwriting the very signals this publishes.

use std::thread;
use std::time::{Duration, Instant};

use agl_slint_cluster::kuksa::proto::val_client::ValClient;
use agl_slint_cluster::kuksa::proto::value::TypedValue;
use agl_slint_cluster::kuksa::proto::{PublishValueRequest, SignalId, signal_id};
use agl_slint_cluster::kuksa::{self, KuksaSource};
use agl_slint_cluster::telemetry::{Beam, TelemetrySource, Warn};

const URL: &str = "http://127.0.0.1:55555";

#[test]
fn published_values_reach_the_source() {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    let Ok(mut client) = rt.block_on(async {
        tokio::time::timeout(Duration::from_secs(2), ValClient::connect(URL))
            .await
            .map_err(|_| ())?
            .map_err(|_| ())
    }) else {
        eprintln!("no databroker at {URL}: skipping");
        return;
    };

    let mut src = KuksaSource::connect(URL);
    // Let the subscription settle before publishing.
    thread::sleep(Duration::from_millis(500));

    let values = [
        ("Vehicle.Speed", TypedValue::Float(77.5)),
        ("Vehicle.Body.Lights.Beam.High.IsOn", TypedValue::Bool(true)),
        (
            "Vehicle.Chassis.ParkingBrake.IsEngaged",
            TypedValue::Bool(true),
        ),
        ("Vehicle.OBD.Status.IsMILOn", TypedValue::Bool(true)),
        (
            "Vehicle.Powertrain.CombustionEngine.EngineOilLevel",
            TypedValue::String("CRITICALLY_LOW".into()),
        ),
        (
            "Vehicle.Cabin.Seat.Row1.DriverSide.IsBelted",
            TypedValue::Bool(false),
        ),
        (
            "Vehicle.Cabin.Door.Row1.PassengerSide.IsOpen",
            TypedValue::Bool(true),
        ),
    ];
    for (path, v) in values {
        let req = PublishValueRequest {
            signal_id: Some(SignalId {
                signal: Some(signal_id::Signal::Path(path.into())),
            }),
            data_point: Some(kuksa::datapoint(v)),
        };
        rt.block_on(client.publish_value(req))
            .unwrap_or_else(|e| panic!("publish {path}: {e}"));
    }

    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let got = src.poll(0.0);
        if got.speed_kmh == 77.5 && got.headlights == Beam::High && got.door_open {
            assert!(got.parking_brake);
            assert_eq!(got.engine, Warn::Amber);
            assert_eq!(got.oil, Warn::Red);
            assert!(got.seatbelt);
            return;
        }
        assert!(Instant::now() < deadline, "values not received: {got:?}");
        thread::sleep(Duration::from_millis(50));
    }
}
