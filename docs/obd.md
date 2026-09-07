# OBD-II interface

The cluster can take its signals from a real car through an OBD-II dongle:
`CLUSTER_SOURCE=obd:<link>` polls the standard Mode 01 PIDs through any
ELM327-compatible adapter (the dialect every consumer dongle speaks) and folds
the answers into `Telemetry`. It only reads -- Mode 01 requests to the
broadcast address, nothing that writes to an ECU.

## Links

| `CLUSTER_SOURCE`                | Dongle                                             |
|---------------------------------|----------------------------------------------------|
| `obd:bt:00:1D:A5:68:98:8B`      | Bluetooth Classic (RFCOMM channel 1), once paired  |
| `obd:tcp:192.168.0.10:35000`    | Wi-Fi dongle                                       |
| `obd:serial:/dev/ttyUSB0`       | USB dongle, or an `rfcomm bind` device             |

The Bluetooth link is opened as a raw RFCOMM socket, so nothing but a paired
device and BlueZ's `bluetoothd` is needed -- no `rfcomm` tool. Pair once:

```sh
bluetoothctl
[bluetooth]# scan on
[bluetooth]# pair 00:1D:A5:68:98:8B      # PIN 1234 if asked
[bluetooth]# trust 00:1D:A5:68:98:8B
```

The link is reopened whenever it drops (dongles sleep with the car), so the
cluster may start before the ignition. Ignition must be on for the ECUs to
answer.

## What is read

| `Telemetry` field | PID  | Note                                   |
|-------------------|------|----------------------------------------|
| `speed_kmh`       | 0D   | polled every other request             |
| `fuel_level`      | 2F   | tank level, %                          |
| `outside_temp_c`  | 46   | ambient air                            |
| `engine`          | 01   | the MIL, amber                         |
| `odo_km`          | A6   | where the car reports it (2020+)       |

A PID the car answers with `NO DATA` is asked once and then skipped.
Everything else keeps its default. Make-specific data -- a hybrid's battery
state of charge, gear, steering angle -- lives behind make-specific requests
(Mode 21/22 to a particular ECU); once such a request and its answer are
known for the car, add a `Pid` variant in `rust/src/obd.rs`.

## Trying it on the bench

Any ELM327 emulator works, e.g. [ELM327-emulator](https://github.com/Ircama/ELM327-emulator)
on a pty or a TCP port:

```sh
elm -n 35000                                   # emulator listening on TCP
CLUSTER_SOURCE=obd:tcp:127.0.0.1:35000 cargo run
```
