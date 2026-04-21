# UART HEX Terminal

Cross-platform serial port monitor built with Rust + eframe/egui.

## Structure

- `src/main.rs` — Entry point, eframe initialization
- `src/app.rs` — Main application state and UI (toolbar, packet view, send bar)
- `src/parser.rs` — Stream parser (delimiter-based), hex utilities, UBX label lookup + CRC validation, `Packet` struct with cached hex/ascii
- `src/serial.rs` — Serial port handle with background reader thread
- `src/logger.rs` — File logger for packets
- `src/ble.rs` — BLE transport (NUS profile support, auto-detection)

## Build & Run

```bash
cargo build          # Debug build
cargo build --release # Release build (use `-j 2` on low-RAM hosts to avoid OOM)
cargo run            # Run debug
```

### System dependencies (Linux)

`rfd` is built with the `gtk3` feature, so GTK 3 dev headers are required:

```bash
sudo apt install libgtk-3-dev libglib2.0-dev libatk1.0-dev \
                 libgdk-pixbuf-2.0-dev libpango1.0-dev libcairo2-dev pkg-config
```

## Key Design Decisions

- Serial I/O runs on a background thread; GUI polls via `mpsc::try_recv()`
- Dual serial connections (UART1/UART2) with shared packet view
- BLE transport with Nordic UART Service (NUS) auto-detection
- Packets are split by a configurable delimiter (default: `B5 62` for u-blox UBX)
- `Packet` pre-computes hex/ascii strings at creation time (cached, not per-frame)
- Packet view uses `ScrollArea::show_rows()` for virtualized rendering (only visible rows)
- `StreamParser::flush()` skips delimiter-only buffers to avoid ghost packets on disconnect
- `gap_flush()` preserves in-progress framed packets (avoids splitting on OS scheduling jitter)
- UBX packets with bad Fletcher-8 checksum get `[CRC!]` label (hardware data loss diagnostic)
- Serial polling runs regardless of transport mode (prevents channel backlog on mode switch)
- File dialogs use `rfd` crate for native cross-platform support
- Send input accepts hex with or without spaces (`B562` and `B5 62` both work)
- Packet timestamps are relative to the first received packet (starts at 0), reset on Clear
- `MAX_BUFFER_SIZE = 72 KiB` fits the UBX worst case (64 KiB payload + framing) without clipping valid frames
- Data captured before the very first delimiter in a session is marked `noise=true` (startup garbage)
- COBS decoder preserves a trailing `0x00` byte inside the payload (no blind trim)
- `SerialConn::disconnect` drains any pending channel messages and flushes the partial buffer as a final packet before killing the reader thread — no tail packets are lost on user-initiated Disconnect
