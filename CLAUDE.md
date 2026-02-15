# UART HEX Terminal

Cross-platform serial port monitor built with Rust + eframe/egui.

## Structure

- `src/main.rs` — Entry point, eframe initialization
- `src/app.rs` — Main application state and UI (toolbar, packet view, send bar)
- `src/parser.rs` — Stream parser (delimiter-based), hex utilities, UBX label lookup
- `src/serial.rs` — Serial port handle with background reader thread
- `src/logger.rs` — File logger for packets

## Build & Run

```bash
cargo build          # Debug build
cargo build --release # Release build
cargo run            # Run debug
```

## Key Design Decisions

- Serial I/O runs on a background thread; GUI polls via `mpsc::try_recv()`
- Packets are split by a configurable delimiter (default: `B5 62` for u-blox UBX)
- File dialogs use `rfd` crate for native cross-platform support
- Send input accepts hex with or without spaces (`B562` and `B5 62` both work)
