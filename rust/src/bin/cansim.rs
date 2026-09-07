// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! Play the vehicle: run the built-in drive-cycle simulation and put it on a
//! CAN bus, so the cluster (started with `CLUSTER_SOURCE=can:<iface>`) shows
//! it, and `candump` shows the frames. Pointed at a virtual interface this
//! needs no hardware at all:
//!
//! ```sh
//! sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
//! cargo run --bin cansim            # defaults to vcan0
//! CLUSTER_SOURCE=can:vcan0 cargo run
//! ```

use std::env;
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use socketcan::{CanSocket, Socket};

use agl_slint_cluster::can;
use agl_slint_cluster::sim::SimSource;
use agl_slint_cluster::telemetry::TelemetrySource;

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let iface = args.next().unwrap_or_else(|| "vcan0".into());
    if iface.starts_with('-') || args.next().is_some() {
        eprintln!("usage: cansim [INTERFACE]   (default: vcan0)");
        return ExitCode::from(2);
    }

    let sock = match CanSocket::open(&iface) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("cansim: cannot open {iface}: {e}");
            return ExitCode::FAILURE;
        }
    };
    eprintln!("cansim: sending the drive cycle on {iface} (Ctrl-C to stop)");

    let mut sim = SimSource::new();
    let period = Duration::from_millis(50);
    let mut next = Instant::now();
    let mut last = next;
    loop {
        next += period;
        thread::sleep(next.saturating_duration_since(Instant::now()));
        let now = Instant::now();
        let dt = now.duration_since(last).as_secs_f32();
        last = now;

        let frame = sim.poll(dt);
        for m in can::encode(&frame) {
            if let Err(e) = sock.write_frame(&can::to_frame(&m)) {
                eprintln!("cansim: {iface}: write failed: {e}");
                return ExitCode::FAILURE;
            }
        }
    }
}
