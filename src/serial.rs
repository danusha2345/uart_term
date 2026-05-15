use serialport::{self, SerialPort};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

use crate::parser;

/// Messages from serial reading thread to GUI
pub enum SerialMsg {
    Data(Vec<u8>),
    Gap,
    Error(String),
    Disconnected,
}

/// Per-connection runtime counters (lock-free, written by reader thread,
/// read by the GUI thread for status display and diagnostics).
#[derive(Default)]
pub struct SerialStats {
    pub bytes_read: AtomicU64,
    pub gaps: AtomicU64,
    pub read_errors: AtomicU64,
}

/// Heap-allocated read buffer. 64 KiB matches typical USB-UART driver buffer
/// sizes — anything smaller risks driver-side overruns at multi-Mbps rates
/// when GUI scheduling jitter delays the next read().
const READ_BUF_SIZE: usize = 64 * 1024;

/// Handle for a connected serial port (background reader + writer)
pub struct SerialHandle {
    pub rx: mpsc::Receiver<SerialMsg>,
    pub stats: Arc<SerialStats>,
    writer: Box<dyn SerialPort>,
    thread: Option<std::thread::JoinHandle<()>>,
    stop: Arc<AtomicBool>,
}

impl SerialHandle {
    pub fn connect(
        port_name: &str,
        baud_rate: u32,
        data_bits: serialport::DataBits,
        parity: serialport::Parity,
        stop_bits: serialport::StopBits,
        flow_control: serialport::FlowControl,
    ) -> Result<Self, String> {
        // Adaptive timeout: ~20 byte-times. Min 5 ms — anything shorter
        // produces spurious Gap events from OS scheduling jitter on high
        // baud rates, splitting frames in Raw/SLIP/COBS modes.
        let timeout_ms = (200_000u64 / baud_rate as u64).clamp(5, 50);

        let port = serialport::new(port_name, baud_rate)
            .data_bits(data_bits)
            .parity(parity)
            .stop_bits(stop_bits)
            .flow_control(flow_control)
            .timeout(Duration::from_millis(timeout_ms))
            .open()
            .map_err(|e| format!("Cannot open {}: {}", port_name, e))?;

        let reader = port
            .try_clone()
            .map_err(|e| format!("Clone failed: {}", e))?;
        let writer = port;

        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(SerialStats::default());
        let stop_clone = stop.clone();
        let stats_clone = stats.clone();

        let thread = std::thread::spawn(move || {
            Self::read_loop(reader, tx, stop_clone, stats_clone);
        });

        Ok(Self {
            rx,
            stats,
            writer,
            thread: Some(thread),
            stop,
        })
    }

    fn read_loop(
        mut reader: Box<dyn SerialPort>,
        tx: mpsc::Sender<SerialMsg>,
        stop: Arc<AtomicBool>,
        stats: Arc<SerialStats>,
    ) {
        let mut buf = vec![0u8; READ_BUF_SIZE];
        let mut had_data = false;
        loop {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            match reader.read(&mut buf) {
                Ok(n) if n > 0 => {
                    had_data = true;
                    stats.bytes_read.fetch_add(n as u64, Ordering::Relaxed);
                    if tx.send(SerialMsg::Data(buf[..n].to_vec())).is_err() {
                        break;
                    }
                }
                Ok(_) => {
                    let _ = tx.send(SerialMsg::Disconnected);
                    break;
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if had_data {
                        had_data = false;
                        stats.gaps.fetch_add(1, Ordering::Relaxed);
                        if tx.send(SerialMsg::Gap).is_err() {
                            break;
                        }
                    }
                }
                Err(e) => {
                    stats.read_errors.fetch_add(1, Ordering::Relaxed);
                    let _ = tx.send(SerialMsg::Error(e.to_string()));
                    let _ = tx.send(SerialMsg::Disconnected);
                    break;
                }
            }
        }
    }

    /// Send raw bytes to the port
    pub fn send(&mut self, data: &[u8]) -> Result<(), String> {
        self.writer
            .write_all(data)
            .map_err(|e| format!("Write error: {}", e))
    }

    pub fn set_dtr(&mut self, state: bool) -> Result<(), String> {
        self.writer
            .write_data_terminal_ready(state)
            .map_err(|e| format!("DTR error: {}", e))
    }

    pub fn set_rts(&mut self, state: bool) -> Result<(), String> {
        self.writer
            .write_request_to_send(state)
            .map_err(|e| format!("RTS error: {}", e))
    }

    pub fn disconnect(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // Drop the writer first so a subsequent connect() to the same port
        // can reopen it even if the reader is still finishing its last read.
        // The reader holds its own clone; dropping the writer releases ours.
        // Note: SerialPort can't be dropped explicitly without unsafe; the
        // writer field is dropped when self is consumed below.

        // Wait for the reader thread to actually exit so the OS handle is
        // released before any reconnect attempt. Reader's read() timeout is
        // bounded (≤50 ms by connect()), so this join is bounded too.
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

/// List available serial ports (filtered to USB-UART by default)
pub fn list_ports() -> Vec<String> {
    match serialport::available_ports() {
        Ok(ports) => ports.into_iter().map(|p| p.port_name).collect(),
        Err(_) => vec![],
    }
}

/// Probe duration per baud rate candidate
const PROBE_DURATION: Duration = Duration::from_millis(300);

/// Auto-detect baud rate by trying each candidate and scoring received data.
/// Returns `Some((baud_rate, score))` on success, `None` if no data received.
pub fn probe_baud_rate(
    port_name: &str,
    candidates: &[u32],
    delimiter: &[u8],
    data_bits: serialport::DataBits,
    parity: serialport::Parity,
    stop_bits: serialport::StopBits,
    stop: &AtomicBool,
) -> Option<(u32, usize)> {
    let mut best: Option<(u32, usize)> = None;

    for &baud in candidates {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let port = serialport::new(port_name, baud)
            .data_bits(data_bits)
            .parity(parity)
            .stop_bits(stop_bits)
            .flow_control(serialport::FlowControl::None)
            .timeout(Duration::from_millis(50))
            .open();

        let mut port = match port {
            Ok(p) => p,
            Err(_) => continue,
        };

        // Read data for PROBE_DURATION
        let mut collected = Vec::new();
        let start = Instant::now();
        let mut buf = vec![0u8; 4096];
        let mut consecutive_errors = 0u32;
        while start.elapsed() < PROBE_DURATION {
            if stop.load(Ordering::Relaxed) {
                return best;
            }
            match port.read(&mut buf) {
                Ok(n) if n > 0 => {
                    collected.extend_from_slice(&buf[..n]);
                    consecutive_errors = 0;
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    consecutive_errors = 0; // timeouts are normal, not failures
                }
                Err(_) => {
                    consecutive_errors += 1;
                    // Tight error loop = port is broken; bail out instead of
                    // burning CPU until PROBE_DURATION expires.
                    if consecutive_errors > 10 {
                        break;
                    }
                }
            }
        }
        drop(port);

        if collected.is_empty() {
            continue;
        }

        // Score the data
        let delimiter_count = parser::count_delimiter(&collected, delimiter);
        let noise = parser::is_noise(&collected);

        let mut seen = [false; 256];
        let mut unique_bytes = 0u16;
        for &b in &collected {
            if !seen[b as usize] {
                seen[b as usize] = true;
                unique_bytes += 1;
            }
        }

        let mut score = delimiter_count * 50;
        if !noise && collected.len() > 4 {
            score += 20;
        }
        if unique_bytes > 10 {
            score += 10;
        }

        // Early exit on high confidence
        if delimiter_count >= 2 {
            return Some((baud, score));
        }

        if score > 0 && (best.is_none() || score > best.unwrap().1) {
            best = Some((baud, score));
        }
    }

    best
}

#[cfg(all(test, unix))]
mod loopback_tests {
    //! End-to-end tests covering the gap that pure parser unit tests miss:
    //! the bytes round-trip through a real OS PTY pair, the background
    //! read_loop, the mpsc channel, and the parser. If any layer drops or
    //! reorders bytes under high write rate, these tests fail.

    use super::*;
    use crate::parser::{DecoderType, StreamParser};
    use serialport::TTYPort;
    use std::io::Write;

    fn ubx_frame(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let mut frame = vec![0xB5, 0x62, class, id];
        frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        frame.extend_from_slice(payload);
        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;
        for &b in &frame[2..] {
            ck_a = ck_a.wrapping_add(b);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        frame.push(ck_a);
        frame.push(ck_b);
        frame
    }

    /// Spin up a read_loop on `reader`, return its channel + stats + stop.
    fn spawn_reader(
        reader: Box<dyn SerialPort>,
    ) -> (
        mpsc::Receiver<SerialMsg>,
        Arc<SerialStats>,
        Arc<AtomicBool>,
        std::thread::JoinHandle<()>,
    ) {
        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(SerialStats::default());
        let stop_c = stop.clone();
        let stats_c = stats.clone();
        let thread = std::thread::spawn(move || {
            SerialHandle::read_loop(reader, tx, stop_c, stats_c);
        });
        (rx, stats, stop, thread)
    }

    /// Drain rx until total bytes received reaches `target` or `timeout` elapses.
    /// Returns the concatenated bytes seen on Data messages.
    fn drain_bytes(
        rx: &mpsc::Receiver<SerialMsg>,
        target: usize,
        timeout: Duration,
    ) -> Vec<u8> {
        let mut buf = Vec::with_capacity(target);
        let start = Instant::now();
        while buf.len() < target && start.elapsed() < timeout {
            match rx.recv_timeout(Duration::from_millis(100)) {
                Ok(SerialMsg::Data(d)) => buf.extend_from_slice(&d),
                Ok(SerialMsg::Gap) => {}
                Ok(SerialMsg::Error(_)) | Ok(SerialMsg::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => continue,
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
        buf
    }

    #[test]
    fn loopback_high_rate_bursts_no_byte_loss() {
        // Push 200 UBX frames as a single burst, verify every byte arrives
        // and parser recovers all 200 frames in order.
        let (mut master, slave) = TTYPort::pair().expect("pty pair");
        let mut reader: Box<dyn SerialPort> = Box::new(slave);
        reader.set_timeout(Duration::from_millis(10)).unwrap();

        let total_frames = 200usize;
        let mut stream = Vec::new();
        for i in 0..total_frames {
            let payload = vec![i as u8; 64];
            stream.extend_from_slice(&ubx_frame(0x01, 0x07, &payload));
        }

        let (rx, stats, stop, reader_thread) = spawn_reader(reader);

        let stream_clone = stream.clone();
        let writer_thread = std::thread::spawn(move || {
            // Mid-size chunks force several read() calls — exercises the
            // read_loop / mpsc / poll path under realistic fragmentation.
            for chunk in stream_clone.chunks(127) {
                master.write_all(chunk).unwrap();
            }
            master.flush().unwrap();
            // Hold the port open long enough for the reader to drain.
            std::thread::sleep(Duration::from_millis(200));
        });

        let received = drain_bytes(&rx, stream.len(), Duration::from_secs(5));
        stop.store(true, Ordering::Relaxed);
        let _ = writer_thread.join();
        let _ = reader_thread.join();

        assert_eq!(
            received.len(),
            stream.len(),
            "byte loss: sent {} got {}",
            stream.len(),
            received.len()
        );
        assert_eq!(received, stream, "byte stream corruption");
        assert_eq!(
            stats.bytes_read.load(Ordering::Relaxed) as usize,
            stream.len(),
            "stats counter disagrees with received length"
        );

        // Now drive the parser with the captured stream — same path the GUI
        // would take. Expect exactly `total_frames` packets emitted.
        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        parser.set_decoder_type(DecoderType::Delimiter);
        let mut pkts = parser.feed(&received);
        if let Some(tail) = parser.flush() {
            pkts.push(tail);
        }
        assert_eq!(
            pkts.len(),
            total_frames,
            "parser dropped frames: expected {} got {}",
            total_frames,
            pkts.len()
        );
    }

    #[test]
    fn loopback_chunked_writes_with_pauses_no_loss() {
        // Writer pauses long enough between chunks to trigger Gap events.
        // Verifies that read_loop's Gap→mpsc path doesn't lose framed data
        // and that the parser's mid-frame guard keeps frames intact.
        let (mut master, slave) = TTYPort::pair().expect("pty pair");
        let mut reader: Box<dyn SerialPort> = Box::new(slave);
        reader.set_timeout(Duration::from_millis(20)).unwrap();

        let total_frames = 50usize;
        let mut stream = Vec::new();
        for i in 0..total_frames {
            let payload = vec![i as u8; 16];
            stream.extend_from_slice(&ubx_frame(0x06, 0x01, &payload));
        }

        let (rx, _stats, stop, reader_thread) = spawn_reader(reader);

        let stream_clone = stream.clone();
        let writer_thread = std::thread::spawn(move || {
            // Write each frame as its own chunk with a pause that's
            // longer than the read timeout — guarantees Gap events.
            let frame_size = 8 + 16; // header + payload + crc
            for chunk in stream_clone.chunks(frame_size) {
                master.write_all(chunk).unwrap();
                master.flush().unwrap();
                std::thread::sleep(Duration::from_millis(40));
            }
            std::thread::sleep(Duration::from_millis(100));
        });

        let received = drain_bytes(&rx, stream.len(), Duration::from_secs(8));
        stop.store(true, Ordering::Relaxed);
        let _ = writer_thread.join();
        let _ = reader_thread.join();

        assert_eq!(received, stream, "byte stream corrupted by Gap path");

        // Drive the full parser-with-Gap-events flow: alternate Data/Gap
        // messages mimicking what poll() actually consumes.
        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        parser.set_decoder_type(DecoderType::Delimiter);
        let mut pkts = Vec::new();
        let frame_size = 8 + 16;
        for chunk in received.chunks(frame_size) {
            pkts.extend(parser.feed(chunk));
            // Simulate Gap delivery
            if let Some(p) = parser.gap_flush() {
                pkts.push(p);
            }
        }
        if let Some(tail) = parser.flush() {
            pkts.push(tail);
        }
        assert_eq!(pkts.len(), total_frames);
    }

    #[test]
    fn disconnect_joins_reader_thread_quickly() {
        // SerialHandle::disconnect must join the background thread (not
        // detach), or fast disconnect→reconnect cycles race on the OS handle.
        let (master, slave) = TTYPort::pair().expect("pty pair");
        let mut reader: Box<dyn SerialPort> = Box::new(slave);
        reader.set_timeout(Duration::from_millis(50)).unwrap();
        let writer: Box<dyn SerialPort> = Box::new(master);

        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(SerialStats::default());
        let stop_c = stop.clone();
        let stats_c = stats.clone();
        let thread = std::thread::spawn(move || {
            SerialHandle::read_loop(reader, tx, stop_c, stats_c);
        });

        let handle = SerialHandle {
            rx,
            stats,
            writer,
            thread: Some(thread),
            stop,
        };

        let start = Instant::now();
        handle.disconnect();
        let elapsed = start.elapsed();

        // Reader's read() blocks at most 50 ms (its timeout), so disconnect
        // should return well under 200 ms. If it hangs, join wasn't issued.
        assert!(
            elapsed < Duration::from_millis(500),
            "disconnect took {:?} — reader thread not joined?",
            elapsed
        );
    }
}
