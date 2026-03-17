use crate::app::LogFormat;
use crate::parser::{Direction, Packet};
use std::fs::File;
use std::io::Write;

pub struct Logger {
    file: File,
    format: LogFormat,
    pub last_error: Option<String>,
    /// Timestamp of the first packet (subtracted to get relative time from 0)
    first_packet_time: Option<f64>,
}

impl Logger {
    pub fn new(path: &str, format: LogFormat) -> Result<Self, String> {
        let file = File::options()
            .create(true)
            .append(true)
            .open(path)
            .map_err(|e| format!("Cannot open log file: {}", e))?;

        let mut logger = Logger { file, format, last_error: None, first_packet_time: None };

        // Write session header
        let now = chrono::Local::now().format("%Y-%m-%d %H:%M:%S");
        let _ = writeln!(logger.file, "\n--- Session {} ---", now);

        Ok(logger)
    }

    pub fn log_packet(&mut self, packet: &Packet) {
        if self.first_packet_time.is_none() {
            self.first_packet_time = Some(packet.timestamp);
        }
        let relative_time = packet.timestamp - self.first_packet_time.unwrap_or(0.0);

        let dir = match packet.direction {
            Direction::Rx => "<",
            Direction::Tx => ">",
        };
        let label = packet.label.as_deref().unwrap_or("");
        let source_prefix = match packet.source {
            Some(ref s) => format!("{} ", s),
            None => String::new(),
        };

        let result = match self.format {
            LogFormat::Hex => {
                writeln!(
                    self.file,
                    "{}{} {:.3}s {} {}",
                    source_prefix, dir, relative_time, packet.hex_string(), label
                )
            }
            LogFormat::Ascii => {
                writeln!(
                    self.file,
                    "{}{} {:.3}s {} {}",
                    source_prefix, dir, relative_time, packet.ascii_string(), label
                )
            }
            LogFormat::HexAscii => {
                writeln!(
                    self.file,
                    "{}{} {:.3}s {} |{}| {}",
                    source_prefix, dir, relative_time, packet.hex_string(), packet.ascii_string(), label
                )
            }
        };
        if let Err(e) = result {
            self.last_error = Some(format!("Log write error: {}", e));
        }
    }

    pub fn flush(&mut self) {
        let _ = self.file.flush();
    }
}
