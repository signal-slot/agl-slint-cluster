// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

fn main() {
    slint_build::compile("../ui/app-window.slint").expect("Slint build failed");

    // kuksa.val.v2 client for the `kuksa` telemetry source. protox compiles
    // the vendored .proto files (and the well-known types they import) in
    // pure Rust; tonic then generates the client code.
    println!("cargo:rerun-if-changed=proto");
    let fds = protox::compile(["kuksa/val/v2/val.proto"], ["proto"]).expect("compile Kuksa protos");
    tonic_prost_build::configure()
        .build_server(false)
        .compile_fds(fds)
        .expect("generate Kuksa client");
}
