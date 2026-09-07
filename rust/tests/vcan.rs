// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! End-to-end over a real SocketCAN interface: encode a frame, write it to
//! `vcan0`, and read it back through `CanSource`. Skips (passes) when no
//! `vcan0` exists, so the ordinary test run does not need root; set one up
//! with `ip link add dev vcan0 type vcan && ip link set up vcan0`.

use std::thread;
use std::time::{Duration, Instant};

use socketcan::{CanSocket, Socket};

use agl_slint_cluster::can::{self, CanSource};
use agl_slint_cluster::telemetry::{Beam, Telemetry, TelemetrySource};

#[test]
fn frames_on_vcan0_reach_the_source() {
    let Ok(tx) = CanSocket::open("vcan0") else {
        eprintln!("no vcan0: skipping");
        return;
    };
    let mut src = CanSource::open("vcan0");
    // Give the reader thread a moment to bind before the first write.
    thread::sleep(Duration::from_millis(200));

    let sent = Telemetry {
        speed_kmh: 88.8,
        headlights: Beam::High,
        turn_street: "Test Street 1".into(),
        ..Telemetry::default()
    };
    for m in can::encode(&sent) {
        tx.write_frame(&can::to_frame(&m)).unwrap();
    }

    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let got = src.poll(0.0);
        if got.headlights == Beam::High && got.turn_street == "Test Street 1" {
            assert!((got.speed_kmh - 88.8).abs() < 0.01);
            return;
        }
        assert!(Instant::now() < deadline, "frames not received: {got:?}");
        thread::sleep(Duration::from_millis(20));
    }
}
