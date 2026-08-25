# Credits and third-party assets

The Rust and Slint source in this repository is original work, released under
the MIT license (see `LICENSE`).

Some bundled assets originate elsewhere:

- **Telltale icons** (`ui/assets/telltales/AGL_Icons_*.svg`) come from the
  [Automotive Grade Linux](https://www.automotivelinux.org/) instrument-cluster
  demo (`agl-cluster-demo-vis`, upstream at git.automotivelinux.org; mirror:
  https://github.com/xen-troops/agl-cluster-demo-vis), and remain licensed under
  the **Apache License 2.0** (Copyright the AGL contributors / Konsulko Group).
  A copy of that license (the canonical Apache-2.0 text) is in
  `ui/assets/telltales/LICENSE`.

  **Modifications:** the base grey glyphs are unchanged from upstream. The
  ISO-colour variants (`*_red`, `*_amber`, `*_green`, `*_white`, `*_blue`) are
  derivative works produced here by recolouring the SVG fill; they are likewise
  Apache-2.0. The MIT license of this repository covers the original Rust/Slint
  source only, **not** these icons.

- **Backdrop plate** (`ui/assets/backdrop-plate.png`) is an AI-generated image
  used as the dark cluster housing / road scene. The gauge rings, ticks, value
  arcs, numbers and telltales on top of it are all drawn by the application.

- **Ring glow** (`ui/assets/ring-left.svg`, `ring-right.svg`) is generated
  (perfect-circle arcs with a Gaussian-blur glow) and is original to this repo.

- [Slint](https://slint.dev/) provides the UI toolkit. See its own licensing.
