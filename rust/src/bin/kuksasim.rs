// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! Play the vehicle on a Kuksa databroker: run the built-in drive-cycle
//! simulation and publish it as VSS signals, so the cluster (started with
//! `CLUSTER_SOURCE=kuksa`) shows it and `kuksa-client` can watch the values.
//!
//! ```sh
//! docker run --rm -p 55555:55555 ghcr.io/eclipse-kuksa/kuksa-databroker:0.6.0 --insecure
//! cargo run --bin kuksasim            # defaults to http://127.0.0.1:55555
//! CLUSTER_SOURCE=kuksa cargo run
//! ```

use std::env;
use std::process::ExitCode;
use std::time::Duration;

use agl_slint_cluster::kuksa::proto::val_client::ValClient;
use agl_slint_cluster::kuksa::proto::{PublishValueRequest, SignalId, signal_id};
use agl_slint_cluster::kuksa::{self, proto::value::TypedValue};
use agl_slint_cluster::sim::SimSource;
use agl_slint_cluster::telemetry::TelemetrySource;

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let url = args
        .next()
        .unwrap_or_else(|| "http://127.0.0.1:55555".into());
    if url.starts_with('-') || args.next().is_some() {
        eprintln!("usage: kuksasim [URL]   (default: http://127.0.0.1:55555)");
        return ExitCode::from(2);
    }
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio runtime");
    match rt.block_on(run(&url)) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("kuksasim: {url}: {e}");
            ExitCode::FAILURE
        }
    }
}

async fn run(url: &str) -> Result<(), Box<dyn std::error::Error>> {
    let mut client = ValClient::connect(url.to_owned()).await?;
    eprintln!("kuksasim: publishing the drive cycle to {url} (Ctrl-C to stop)");

    let mut sim = SimSource::new();
    // 10 Hz is plenty for a broker; only signals that changed are sent.
    let period = Duration::from_millis(100);
    let mut ticker = tokio::time::interval(period);
    let mut last: Vec<(&str, TypedValue)> = Vec::new();
    loop {
        ticker.tick().await;
        let frame = sim.poll(period.as_secs_f32());
        let now = kuksa::signals(&frame);
        for (i, (path, value)) in now.iter().enumerate() {
            if last.get(i).is_some_and(|(_, prev)| prev == value) {
                continue;
            }
            let request = PublishValueRequest {
                signal_id: Some(SignalId {
                    signal: Some(signal_id::Signal::Path(path.to_string())),
                }),
                data_point: Some(kuksa::datapoint(value.clone())),
            };
            client.publish_value(request).await?;
        }
        last = now;
    }
}
