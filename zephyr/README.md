# Zephyr target (planned)

A C++ port of the cluster on the [Zephyr RTOS](https://zephyrproject.org/)
(target board e.g. Raspberry Pi 4).

The shared UI in [`../ui`](../ui) compiles unchanged through Slint's **C++**
API — Zephyr is a C/C++, CMake/Kconfig environment. Only the application logic
(the `Telemetry` source and the property glue, ~300 lines) is reimplemented in
C++; the `.slint` design is reused as-is.

Not yet implemented.
