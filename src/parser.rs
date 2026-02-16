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
}

impl Packet {
    pub fn hex_string(&self) -> String {
        self.data
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ")
    }

    pub fn direction_arrow(&self) -> &str {
        match self.direction {
            Direction::Rx => "←",
            Direction::Tx => "→",
        }
    }

    pub fn ascii_string(&self) -> String {
        self.data
            .iter()
            .map(|&b| {
                if b.is_ascii_graphic() || b == b' ' {
                    b as char
                } else {
                    '.'
                }
            })
            .collect()
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

/// Stream parser that splits incoming bytes by a configurable delimiter.
pub struct StreamParser {
    delimiter: Vec<u8>,
    buffer: Vec<u8>,
    start_time: std::time::Instant,
}

impl StreamParser {
    pub fn new(delimiter: Vec<u8>) -> Self {
        Self {
            delimiter,
            buffer: Vec::with_capacity(1024),
            start_time: std::time::Instant::now(),
        }
    }

    pub fn set_delimiter(&mut self, delimiter: Vec<u8>) {
        self.delimiter = delimiter;
    }

    pub fn delimiter(&self) -> &[u8] {
        &self.delimiter
    }

    /// Feed bytes into the parser, returns completed packets.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Packet> {
        let mut packets = Vec::new();

        for &byte in data {
            self.buffer.push(byte);

            // Check if the buffer ends with the delimiter
            if self.buffer.len() >= self.delimiter.len() && !self.delimiter.is_empty() {
                let buf_len = self.buffer.len();
                let delim_len = self.delimiter.len();

                if self.buffer[buf_len - delim_len..] == self.delimiter[..] {
                    // Everything before the delimiter is a complete packet
                    let packet_data: Vec<u8> = self.buffer[..buf_len - delim_len].to_vec();

                    if !packet_data.is_empty() {
                        let label = ubx_label(&packet_data, &self.delimiter);
                        packets.push(Packet {
                            timestamp: self.start_time.elapsed().as_secs_f64(),
                            direction: Direction::Rx,
                            data: packet_data,
                            label,
                            source: None,
                        });
                    }

                    // Keep the delimiter as the start of the next packet
                    self.buffer = self.delimiter.clone();
                }
            }
        }

        packets
    }

    /// Flush any remaining data as a packet
    pub fn flush(&mut self) -> Option<Packet> {
        if self.buffer.is_empty() {
            return None;
        }
        let data = std::mem::take(&mut self.buffer);
        let label = ubx_label(&data, &self.delimiter);
        Some(Packet {
            timestamp: self.start_time.elapsed().as_secs_f64(),
            direction: Direction::Rx,
            data,
            label,
            source: None,
        })
    }

    pub fn elapsed(&self) -> f64 {
        self.start_time.elapsed().as_secs_f64()
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
