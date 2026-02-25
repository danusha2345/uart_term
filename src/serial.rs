use serialport::{self, SerialPort};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use crate::parser;

/// Messages from serial reading thread to GUI
pub enum SerialMsg {
    Data(Vec<u8>),
    Gap,
    Error(String),
    Disconnected,
}

/// Handle for a connected serial port (background reader + writer)
pub struct SerialHandle {
    pub rx: mpsc::Receiver<SerialMsg>,
    writer: Box<dyn SerialPort>,
    _thread: std::thread::JoinHandle<()>,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
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
        // Adaptive timeout: ~20 byte-times, min 2ms, max 50ms
        let timeout_ms = (200_000u64 / baud_rate as u64).max(2).min(50);

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
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let stop_clone = stop.clone();

        let thread = std::thread::spawn(move || {
            Self::read_loop(reader, tx, stop_clone);
        });

        Ok(Self {
            rx,
            writer,
            _thread: thread,
            stop,
        })
    }

    fn read_loop(
        mut reader: Box<dyn SerialPort>,
        tx: mpsc::Sender<SerialMsg>,
        stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    ) {
        let mut buf = [0u8; 1024];
        let mut had_data = false;
        loop {
            if stop.load(std::sync::atomic::Ordering::Relaxed) {
                break;
            }
            match reader.read(&mut buf) {
                Ok(n) if n > 0 => {
                    had_data = true;
                    if tx.send(SerialMsg::Data(buf[..n].to_vec())).is_err() {
                        break;
                    }
                }
                Ok(_) => {
                    if had_data {
                        had_data = false;
                        if tx.send(SerialMsg::Gap).is_err() {
                            break;
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if had_data {
                        had_data = false;
                        if tx.send(SerialMsg::Gap).is_err() {
                            break;
                        }
                    }
                }
                Err(e) => {
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

    pub fn disconnect(self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        // Thread will stop on next iteration
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
        let mut buf = [0u8; 1024];
        while start.elapsed() < PROBE_DURATION {
            if stop.load(Ordering::Relaxed) {
                return best;
            }
            match port.read(&mut buf) {
                Ok(n) if n > 0 => collected.extend_from_slice(&buf[..n]),
                _ => {}
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

        if score > 0 {
            if best.is_none() || score > best.unwrap().1 {
                best = Some((baud, score));
            }
        }
    }

    best
}
