# Bare-metal target (planned)

A `no_std` **Rust** port for a microcontroller (target e.g. Raspberry Pi
Pico 2 / RP2350), using Slint's **software renderer** against a small SPI
display.

The shared components in [`../ui`](../ui) (theme, gauges, telltales) are
reused, but the full-resolution backdrop image and the Gaussian-blur glow are
too heavy for an MCU's RAM and CPU. This target therefore uses a slimmer,
software-renderer-friendly UI profile: a small resolution and pre-baked, flat
visuals.

Not yet implemented.
