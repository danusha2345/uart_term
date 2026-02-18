# UART HEX Terminal

Cross-platform serial port monitor built with Rust + eframe/egui.

## Structure

- `src/main.rs` — Entry point, eframe initialization
- `src/app.rs` — Main application state and UI (toolbar, packet view, send bar)
- `src/parser.rs` — Stream parser (delimiter-based), hex utilities, UBX label lookup, `Packet` struct with cached hex/ascii
- `src/serial.rs` — Serial port handle with background reader thread
- `src/logger.rs` — File logger for packets
- `src/ble.rs` — BLE transport (NUS profile support, auto-detection)

## Build & Run

```bash
cargo build          # Debug build
cargo build --release # Release build
cargo run            # Run debug
```

## Key Design Decisions

- Serial I/O runs on a background thread; GUI polls via `mpsc::try_recv()`
- Dual serial connections (UART1/UART2) with shared packet view
- BLE transport with Nordic UART Service (NUS) auto-detection
- Packets are split by a configurable delimiter (default: `B5 62` for u-blox UBX)
- `Packet` pre-computes hex/ascii strings at creation time (cached, not per-frame)
- Packet view uses `ScrollArea::show_rows()` for virtualized rendering (only visible rows)
- `StreamParser::flush()` skips delimiter-only buffers to avoid ghost packets on disconnect
- Serial polling runs regardless of transport mode (prevents channel backlog on mode switch)
- File dialogs use `rfd` crate for native cross-platform support
- Send input accepts hex with or without spaces (`B562` and `B5 62` both work)
