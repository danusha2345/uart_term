use serialport::{self, SerialPort};
use std::sync::mpsc;
use std::time::Duration;

/// Messages from serial reading thread to GUI
pub enum SerialMsg {
    Data(Vec<u8>),
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
        let port = serialport::new(port_name, baud_rate)
            .data_bits(data_bits)
            .parity(parity)
            .stop_bits(stop_bits)
            .flow_control(flow_control)
            .timeout(Duration::from_millis(50))
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
        loop {
            if stop.load(std::sync::atomic::Ordering::Relaxed) {
                break;
            }
            match reader.read(&mut buf) {
                Ok(n) if n > 0 => {
                    if tx.send(SerialMsg::Data(buf[..n].to_vec())).is_err() {
                        break;
                    }
                }
                Ok(_) => {} // timeout, no data
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
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
