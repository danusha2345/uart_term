use crate::app::LogFormat;
use crate::parser::{Direction, Packet};
use std::fs::File;
use std::io::Write;

pub struct Logger {
    file: File,
    format: LogFormat,
}

impl Logger {
    pub fn new(path: &str, format: LogFormat) -> Result<Self, String> {
        let file = File::options()
            .create(true)
            .append(true)
            .open(path)
            .map_err(|e| format!("Cannot open log file: {}", e))?;

        let mut logger = Logger { file, format };

        // Write session header
        let now = chrono::Local::now().format("%Y-%m-%d %H:%M:%S");
        let _ = writeln!(logger.file, "\n--- Session {} ---", now);

        Ok(logger)
    }

    pub fn log_packet(&mut self, packet: &Packet) {
        let dir = match packet.direction {
            Direction::Rx => "RX",
            Direction::Tx => "TX",
        };
        let label = packet.label.as_deref().unwrap_or("");

        match self.format {
            LogFormat::Hex => {
                let _ = writeln!(
                    self.file,
                    "{:10.3}s {} {} {}",
                    packet.timestamp,
                    dir,
                    packet.hex_string(),
                    label
                );
            }
            LogFormat::Ascii => {
                let _ = writeln!(
                    self.file,
                    "{:10.3}s {} {} {}",
                    packet.timestamp,
                    dir,
                    packet.ascii_string(),
                    label
                );
            }
            LogFormat::HexAscii => {
                let _ = writeln!(
                    self.file,
                    "{:10.3}s {} {} |{}| {}",
                    packet.timestamp,
                    dir,
                    packet.hex_string(),
                    packet.ascii_string(),
                    label
                );
            }
        }
    }

    pub fn flush(&mut self) {
        let _ = self.file.flush();
    }
}
