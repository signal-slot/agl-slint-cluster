// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Signal Slot Inc.

//! The non-UI half of the cluster: the `Telemetry` frame, the sources that
//! produce it, and the CAN encoding of it. The binary in `main.rs` adds the
//! Slint window on top; `bin/cansim.rs` reuses the simulation and the CAN
//! encoder to play the part of the vehicle on a (virtual) bus.

pub mod can;
pub mod sim;
pub mod telemetry;
