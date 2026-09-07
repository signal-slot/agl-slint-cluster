# Kuksa / VSS interface

The cluster can take its signals from an [Eclipse Kuksa](https://eclipse-kuksa.github.io/kuksa-website/)
databroker, the vehicle-signal server that Automotive Grade Linux ships. The
broker holds a tree of signals named by the COVESA
[Vehicle Signal Specification](https://covesa.github.io/vehicle_signal_specification/)
(VSS); providers such as `kuksa-can-provider` write into it, and the cluster
subscribes to the paths it shows over gRPC (`kuksa.val.v2`).

This is the AGL-native alternative to reading CAN directly (see
[can.md](can.md)): the same cluster, but the bus and its DBC are the
provider's business, and any source that speaks VSS -- CAN, SOME/IP, a
simulator, a real car over OBD -- lights the same gauges.

## Quick start with a local broker

```sh
docker run --rm -p 55555:55555 ghcr.io/eclipse-kuksa/kuksa-databroker:0.6.0 --insecure

cd rust
CLUSTER_SOURCE=kuksa cargo run &       # the cluster, subscribed to the broker
cargo run --bin kuksasim               # the "vehicle", publishing the drive cycle
```

`kuksasim` publishes the built-in simulation as VSS signals at 10 Hz. Poke at
individual signals with the [kuksa-client](https://github.com/eclipse-kuksa/kuksa-python-sdk)
CLI (`pip install kuksa-client`):

```sh
kuksa-client grpc://127.0.0.1:55555
> publishValue Vehicle.Speed 100
> publishValue Vehicle.Body.Lights.DirectionIndicator.Left.IsSignaling true
> publishValue Vehicle.Chassis.ParkingBrake.IsEngaged true
> publishValue Vehicle.Powertrain.CombustionEngine.EngineOilLevel CRITICALLY_LOW
```

`CLUSTER_SOURCE=kuksa` connects to `http://127.0.0.1:55555`;
`CLUSTER_SOURCE=kuksa:http://host:port` names another broker. The connection
is retried whenever the broker is unreachable or the stream ends, so the
cluster may start before the broker. No TLS or token support: on AGL the
broker runs on the same board.

## Signal mapping

The paths below exist unchanged in VSS 4.0, 5.0 and 5.1 (the databroker
image above ships 5.1). The table lives in `rust/src/kuksa.rs` as `PATHS`
(what is subscribed), `apply` (VSS to `Telemetry`) and `signals` (the
reverse, used by `kuksasim`).

| `Telemetry` field   | VSS path(s)                                                   | Note                                          |
|---------------------|---------------------------------------------------------------|-----------------------------------------------|
| `speed_kmh`         | `Vehicle.Speed`                                               | km/h                                          |
| `power`             | `Vehicle.Powertrain.ElectricMotor.Power`, `.MaxPower`         | Power / MaxPower; 150 kW until MaxPower is set |
| `range_km`          | `Vehicle.Powertrain.Range`                                    | metres in VSS                                 |
| `battery_level`     | `Vehicle.Powertrain.TractionBattery.StateOfCharge.Current`    | percent                                       |
| `fuel_level`        | `Vehicle.Powertrain.FuelSystem.RelativeLevel`                 | percent                                       |
| `outside_temp_c`    | `Vehicle.Exterior.AirTemperature`                             |                                               |
| `odo_km`            | `Vehicle.TraveledDistance`                                    |                                               |
| `trip_seconds`      | `Vehicle.TripDuration`                                        |                                               |
| `cruise_speed_kmh`  | `Vehicle.ADAS.CruiseControl.SpeedSet`                         |                                               |
| `cruise`            | `Vehicle.ADAS.CruiseControl.IsEnabled`, `.IsActive`           | active > set > off                            |
| `lta_on`, `lane`    | `Vehicle.ADAS.LaneDepartureDetection.IsEnabled`, `.IsWarning` | off / tracking / warn                         |
| `gear`              | `Vehicle.Powertrain.Transmission.SelectedGear`                | 126 P, 127 D, 0 N, negative R                 |
| `ready`             | `Vehicle.LowVoltageSystemState`                               | `ON` or `START`                               |
| `turn_left/right`   | `Vehicle.Body.Lights.DirectionIndicator.Left/Right.IsSignaling` | request; the cluster blinks                 |
| `headlights`        | `Vehicle.Body.Lights.Beam.Low.IsOn`, `.High.IsOn`             | high wins                                     |
| `engine`            | `Vehicle.OBD.Status.IsMILOn`                                  | amber                                         |
| `oil`               | `Vehicle.Powertrain.CombustionEngine.EngineOilLevel`          | `LOW` amber, `CRITICALLY_LOW` red             |
| `battery_warn`      | `Vehicle.LowVoltageBattery.CurrentVoltage`                    | below 11.8 V amber, below 11.0 V red          |
| `abs`               | `Vehicle.ADAS.ABS.IsError`                                    | red                                           |
| `parking_brake`     | `Vehicle.Chassis.ParkingBrake.IsEngaged`                      |                                               |
| `seatbelt`          | `Vehicle.Cabin.Seat.Row1.DriverSide.IsBelted`                 | lamp shows *unfastened*                       |
| `door_open`         | `Vehicle.Cabin.Door.Row1/Row2.DriverSide/PassengerSide.IsOpen` | any open door                                |

Not in VSS, so left at their defaults: instantaneous `efficiency_percent`,
`avg_efficiency_kwh`, the next-turn `turn_distance_km` / `turn_street`, and
Lane Change Assist (`lca_on`).

## Build notes

The `kuksa.val.v2` protos are vendored in `rust/proto/` (Apache-2.0, from
the Eclipse Kuksa project) and compiled at build time by
[protox](https://crates.io/crates/protox), a pure-Rust protobuf compiler, so
no `protoc` is needed on the build host or in a Yocto cross build.
