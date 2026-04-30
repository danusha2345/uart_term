## Project Map

- Purpose: Rust/egui Serial and BLE terminal for hardware debugging and binary protocol analysis.
- Entry point: `src/main.rs`; UI: `src/app.rs`.
- Transports: `src/serial.rs`, `src/ble.rs`; framing/parsing: `src/parser.rs`; logging: `src/logger.rs`.
- Docs: `README.md`, `docs/UC6580I.md`.
- Runtime logs and scratch captures are kept at repo root or `.scratch/`.

## Build And Test

- Check: `cargo check`.
- Run GUI: `cargo run --release`.
- Build binary: `cargo build --release`.

## Local Pitfalls

- Keep generated logs/captures out of commits unless the user explicitly asks.
- For GNSS/UC6580I debugging, preserve raw logs and document interpretation separately.
