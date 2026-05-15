# UART HEX Terminal

Cross-platform serial port monitor built with Rust + eframe/egui.

## Structure

- `src/main.rs` — Entry point, eframe initialization
- `src/app.rs` — Main application state and UI (toolbar, packet view, send bar)
- `src/parser.rs` — Stream parser (Raw/Delimiter/SLIP/COBS/NMEA), hex utilities, UBX/NMEA label lookup + CRC validation, `Packet` struct with cached hex/ascii
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
- Packets are split by selectable decoders: Raw, Delimiter (default `B5 62` for u-blox UBX), SLIP, COBS, or NMEA
- `Packet` pre-computes hex/ascii strings at creation time (cached, not per-frame)
- Packet view uses `ScrollArea::show_rows()` for virtualized rendering (only visible rows)
- `StreamParser::flush()` skips delimiter-only buffers to avoid ghost packets on disconnect
- `gap_flush()` preserves in-progress framed packets for **all** decoders (Delimiter/NMEA/SLIP/COBS) via `is_mid_frame()` — avoids splitting on OS scheduling jitter
- `flush()`, `gap_flush()`, `flush_stale()` share `finalize_buffer()` so label/noise logic stays consistent across all flush paths
- UBX packets with bad Fletcher-8 checksum get `[CRC!]` label (hardware data loss diagnostic)
- NMEA decoder frames `$...LF`, strips trailing CR/LF, labels sentences like `[GN-RMC]`, and appends `[CRC!]` on checksum mismatch
- Serial polling runs regardless of transport mode (prevents channel backlog on mode switch)
- File dialogs use `rfd` crate for native cross-platform support
- Send input accepts hex with or without spaces (`B562` and `B5 62` both work)
- Packet timestamps are relative to the first received packet (starts at 0), reset on Clear
- `MAX_BUFFER_SIZE = 72 KiB` fits the UBX worst case (64 KiB payload + framing) without clipping valid frames
- Data captured before the very first delimiter in a session is marked `noise=true` (startup garbage)
- COBS decoder preserves a trailing `0x00` byte inside the payload (no blind trim)
- `SerialConn::disconnect` drains any pending channel messages and flushes the partial buffer as a final packet before killing the reader thread — no tail packets are lost on user-initiated Disconnect
- `SerialHandle::disconnect` **joins** the reader thread (bounded by the read timeout, ≤50 ms) so the OS port handle is released before any reconnect — prevents "port busy" races
- Read buffer in `read_loop` is **64 KiB** (heap) — sized for typical USB-UART driver buffers at multi-Mbps rates; 1 KiB risks driver-side overruns
- Adaptive read timeout is clamped to **5–50 ms** — anything below 5 ms produces spurious `Gap` events from OS scheduling jitter on high baud rates and splits frames in Raw/SLIP/COBS modes
- `SerialStats` (lock-free `AtomicU64` counters: `bytes_read`, `gaps`, `read_errors`) is shared between the reader thread and the GUI; status bar shows aggregate counts so packet loss on the wire is visible
- `SerialConn::poll` is throttled to **`MAX_MSGS_PER_POLL = 256`** messages per GUI tick — caps how long one frame can be blocked on a deep mpsc backlog; the rest is drained on the next repaint
- `flush_stale(500 ms)` runs in **both** Serial and BLE poll paths as a safety net — fires only on long quiet periods, never on mid-frame buffers (mid-frame guard is in `is_mid_frame`)
- Packet store is `VecDeque<Packet>` (not `Vec`) — `push_back`/`pop_front` are O(1), so the `max_packets` cap doesn't shift the whole vector on every overflow
- Logger uses `BufWriter<File>` (64 KiB) and a `Drop` flush — high-rate sessions don't generate one syscall per packet
- `set_delimiter` clears the buffer, escape state, and `unframed` flag — prevents stale bytes from merging into the next packet under the new delimiter
- `probe_baud_rate` bails out after 10 consecutive non-timeout errors so a broken port doesn't burn CPU for the full probe duration
- Parser regression tests cover COBS trailing zero, delimiter framing/splitting/overflow, UBX CRC, NMEA framing/checksum, and SLIP/COBS mid-frame `gap_flush` guards
- Loopback integration tests (`#[cfg(unix)]`, `serialport::TTYPort::pair()`) exercise the full read_loop ↔ mpsc ↔ poll path with byte-level verification at high write rates and with Gap-forcing pauses
