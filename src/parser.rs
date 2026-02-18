/// Packet direction
#[derive(Clone, Copy, PartialEq)]
pub enum Direction {
    Rx,
    Tx,
}

/// A parsed packet with timestamp, direction, and raw bytes
#[derive(Clone)]
pub struct Packet {
    pub timestamp: f64,
    pub direction: Direction,
    pub data: Vec<u8>,
    pub label: Option<String>,
    pub source: Option<String>,
    pub hex: String,
    pub ascii: String,
}

impl Packet {
    /// Create a new packet, pre-computing hex and ascii caches.
    pub fn new(
        timestamp: f64,
        direction: Direction,
        data: Vec<u8>,
        label: Option<String>,
        source: Option<String>,
    ) -> Self {
        let hex = data
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ");
        let ascii = data
            .iter()
            .map(|&b| {
                if b.is_ascii_graphic() || b == b' ' {
                    b as char
                } else {
                    '.'
                }
            })
            .collect();
        Self {
            timestamp,
            direction,
            data,
            label,
            source,
            hex,
            ascii,
        }
    }

    pub fn hex_string(&self) -> &str {
        &self.hex
    }

    pub fn ascii_string(&self) -> &str {
        &self.ascii
    }
}

/// UBX class name lookup (only used when delimiter is B5 62)
pub fn ubx_class_name(class_byte: u8) -> &'static str {
    match class_byte {
        0x01 => "NAV",
        0x02 => "RXM",
        0x04 => "INF",
        0x05 => "ACK",
        0x06 => "CFG",
        0x09 => "UPD",
        0x0A => "MON",
        0x0D => "TIM",
        0x10 => "ESF",
        0x13 => "MGA",
        0x21 => "LOG",
        0x27 => "SEC",
        0x28 => "HNR",
        0xF0 => "NMEA",
        0xF1 => "PUBX",
        _ => "",
    }
}

/// Try to generate a UBX label for the packet.
/// Only applies when delimiter is [0xB5, 0x62].
pub fn ubx_label(data: &[u8], delimiter: &[u8]) -> Option<String> {
    if delimiter != [0xB5, 0x62] {
        return None;
    }
    // Need at least delimiter + class + id = 4 bytes
    if data.len() < 4 {
        return None;
    }
    let cls = data[2];
    let id = data[3];
    let name = ubx_class_name(cls);
    if name.is_empty() {
        Some(format!("[0x{:02X}-0x{:02X}]", cls, id))
    } else {
        Some(format!("[{}-0x{:02X}]", name, id))
    }
}

/// Maximum buffer size before forced flush (protection against OOM)
const MAX_BUFFER_SIZE: usize = 65536;

/// Stream parser that splits incoming bytes by a configurable delimiter.
pub struct StreamParser {
    delimiter: Vec<u8>,
    buffer: Vec<u8>,
    start_time: std::time::Instant,
    last_data_time: Option<std::time::Instant>,
}

impl StreamParser {
    pub fn new(delimiter: Vec<u8>) -> Self {
        Self {
            delimiter,
            buffer: Vec::with_capacity(1024),
            start_time: std::time::Instant::now(),
            last_data_time: None,
        }
    }

    pub fn set_delimiter(&mut self, delimiter: Vec<u8>) {
        self.delimiter = delimiter;
    }

    /// Feed bytes into the parser, returns completed packets.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Packet> {
        let mut packets = Vec::new();
        self.last_data_time = Some(std::time::Instant::now());

        for &byte in data {
            self.buffer.push(byte);

            // Flush buffer as partial packet if it exceeds max size (OOM protection)
            if self.buffer.len() >= MAX_BUFFER_SIZE {
                let data = std::mem::take(&mut self.buffer);
                packets.push(Packet::new(
                    self.start_time.elapsed().as_secs_f64(),
                    Direction::Rx,
                    data,
                    None,
                    None,
                ));
                continue;
            }

            // Check if the buffer ends with the delimiter
            if self.buffer.len() >= self.delimiter.len() && !self.delimiter.is_empty() {
                let buf_len = self.buffer.len();
                let delim_len = self.delimiter.len();

                if self.buffer[buf_len - delim_len..] == self.delimiter[..] {
                    // Everything before the delimiter is a complete packet
                    let packet_data: Vec<u8> = self.buffer[..buf_len - delim_len].to_vec();

                    if !packet_data.is_empty() {
                        let label = ubx_label(&packet_data, &self.delimiter);
                        packets.push(Packet::new(
                            self.start_time.elapsed().as_secs_f64(),
                            Direction::Rx,
                            packet_data,
                            label,
                            None,
                        ));
                        self.last_data_time = None;
                    }

                    // Keep the delimiter as the start of the next packet
                    self.buffer = self.delimiter.clone();
                }
            }
        }

        packets
    }

    /// Flush buffered data if no new data arrived within the timeout.
    pub fn flush_stale(&mut self, timeout: std::time::Duration) -> Option<Packet> {
        if let Some(last) = self.last_data_time {
            if last.elapsed() >= timeout
                && !self.buffer.is_empty()
                && self.buffer != self.delimiter
            {
                self.last_data_time = None;
                return self.flush();
            }
        }
        None
    }

    /// Flush any remaining data as a packet.
    /// Skips if buffer is empty or contains only the delimiter prefix.
    pub fn flush(&mut self) -> Option<Packet> {
        if self.buffer.is_empty() || self.buffer == self.delimiter {
            self.buffer.clear();
            return None;
        }
        let data = std::mem::take(&mut self.buffer);
        let label = ubx_label(&data, &self.delimiter);
        Some(Packet::new(
            self.start_time.elapsed().as_secs_f64(),
            Direction::Rx,
            data,
            label,
            None,
        ))
    }

    pub fn elapsed(&self) -> f64 {
        self.start_time.elapsed().as_secs_f64()
    }
}

/// Parse delimiter input:
/// - `"$"` or `"GP"` — ASCII literal in quotes
/// - `\n`, `\r\n` — escape sequences
/// - `B5 62` — hex bytes (default)
pub fn parse_delimiter_input(s: &str) -> Result<Vec<u8>, String> {
    let trimmed = s.trim();
    // Quoted ASCII literal: "$" → [0x24]
    if trimmed.starts_with('"') && trimmed.ends_with('"') && trimmed.len() >= 2 {
        let inner = &trimmed[1..trimmed.len() - 1];
        if inner.is_empty() {
            return Err("Empty delimiter".to_string());
        }
        return Ok(inner.as_bytes().to_vec());
    }
    // Escape sequences: \n, \r\n, \t
    if trimmed.contains('\\') {
        let mut bytes = Vec::new();
        let mut chars = trimmed.chars();
        while let Some(c) = chars.next() {
            if c == '\\' {
                match chars.next() {
                    Some('n') => bytes.push(0x0A),
                    Some('r') => bytes.push(0x0D),
                    Some('t') => bytes.push(0x09),
                    Some('0') => bytes.push(0x00),
                    Some('\\') => bytes.push(b'\\'),
                    Some(other) => return Err(format!("Unknown escape: \\{}", other)),
                    None => return Err("Trailing backslash".to_string()),
                }
            } else if !c.is_whitespace() {
                bytes.push(c as u8);
            }
        }
        if bytes.is_empty() {
            Err("Empty delimiter".to_string())
        } else {
            Ok(bytes)
        }
    } else {
        parse_hex_string(trimmed)
    }
}

/// Parse a hex string like "B5 62" or "B562" into bytes
pub fn parse_hex_string(s: &str) -> Result<Vec<u8>, String> {
    let clean: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();

    if clean.len() % 2 != 0 {
        return Err("Odd number of hex digits".to_string());
    }

    let mut bytes = Vec::new();
    for i in (0..clean.len()).step_by(2) {
        let byte = u8::from_str_radix(&clean[i..i + 2], 16).map_err(|e| e.to_string())?;
        bytes.push(byte);
    }
    Ok(bytes)
}
