/// Decoder/framing type
#[derive(Clone, Copy, PartialEq)]
pub enum DecoderType {
    Raw,
    Delimiter,
    Slip,
    Cobs,
}

impl DecoderType {
    pub fn label(&self) -> &str {
        match self {
            Self::Raw => "Raw",
            Self::Delimiter => "Delimiter",
            Self::Slip => "SLIP",
            Self::Cobs => "COBS",
        }
    }

    pub const ALL: [DecoderType; 4] = [Self::Raw, Self::Delimiter, Self::Slip, Self::Cobs];
}

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
    pub noise: bool,
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
        let noise = direction == Direction::Rx && is_noise(&data);
        Self {
            timestamp,
            direction,
            data,
            label,
            source,
            hex,
            ascii,
            noise,
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

/// Maximum buffer size before forced flush (protection against OOM).
/// Kept small so noise from floating lines produces filterable-size packets.
const MAX_BUFFER_SIZE: usize = 4096;

/// Stream parser that splits incoming bytes using a configurable decoder.
pub struct StreamParser {
    delimiter: Vec<u8>,
    buffer: Vec<u8>,
    start_time: std::time::Instant,
    last_data_time: Option<std::time::Instant>,
    pub decoder_type: DecoderType,
    slip_escape: bool,
    /// True if current buffer accumulated without any delimiter/frame boundary match.
    /// Packets flushed by gap or overflow with this flag set are likely noise.
    unframed: bool,
}

impl StreamParser {
    pub fn new(delimiter: Vec<u8>) -> Self {
        Self {
            delimiter,
            buffer: Vec::with_capacity(1024),
            start_time: std::time::Instant::now(),
            last_data_time: None,
            decoder_type: DecoderType::Delimiter,
            slip_escape: false,
            unframed: true,
        }
    }

    pub fn set_decoder_type(&mut self, dt: DecoderType) {
        self.decoder_type = dt;
        self.buffer.clear();
        self.slip_escape = false;
        self.unframed = true;
        self.last_data_time = None;
    }

    /// Discard any buffered data without emitting a packet.
    pub fn clear(&mut self) {
        self.buffer.clear();
        self.slip_escape = false;
        self.unframed = true;
        self.last_data_time = None;
    }

    pub fn set_delimiter(&mut self, delimiter: Vec<u8>) {
        self.delimiter = delimiter;
    }

    /// Feed bytes into the parser, returns completed packets.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Packet> {
        self.last_data_time = Some(std::time::Instant::now());
        match self.decoder_type {
            DecoderType::Raw => self.feed_raw(data),
            DecoderType::Delimiter => self.feed_delimiter(data),
            DecoderType::Slip => self.feed_slip(data),
            DecoderType::Cobs => self.feed_cobs(data),
        }
    }

    fn feed_raw(&mut self, data: &[u8]) -> Vec<Packet> {
        if data.is_empty() {
            return Vec::new();
        }
        vec![Packet::new(
            self.start_time.elapsed().as_secs_f64(),
            Direction::Rx,
            data.to_vec(),
            None,
            None,
        )]
    }

    fn feed_delimiter(&mut self, data: &[u8]) -> Vec<Packet> {
        let mut packets = Vec::new();

        for &byte in data {
            self.buffer.push(byte);

            // Flush buffer as partial packet if it exceeds max size (OOM protection)
            if self.buffer.len() >= MAX_BUFFER_SIZE {
                let unframed = self.unframed;
                let data = std::mem::take(&mut self.buffer);
                let mut pkt = Packet::new(
                    self.start_time.elapsed().as_secs_f64(),
                    Direction::Rx,
                    data,
                    None,
                    None,
                );
                if unframed {
                    pkt.noise = true;
                }
                packets.push(pkt);
                continue;
            }

            // Check if the buffer ends with the delimiter
            if self.buffer.len() >= self.delimiter.len() && !self.delimiter.is_empty() {
                let buf_len = self.buffer.len();
                let delim_len = self.delimiter.len();

                if self.buffer[buf_len - delim_len..] == self.delimiter[..] {
                    self.unframed = false;
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

    /// SLIP decoder (RFC 1055): END=0xC0, ESC=0xDB, ESC_END=0xDC, ESC_ESC=0xDD
    fn feed_slip(&mut self, data: &[u8]) -> Vec<Packet> {
        const END: u8 = 0xC0;
        const ESC: u8 = 0xDB;
        const ESC_END: u8 = 0xDC;
        const ESC_ESC: u8 = 0xDD;

        let mut packets = Vec::new();

        for &byte in data {
            if self.slip_escape {
                self.slip_escape = false;
                match byte {
                    ESC_END => self.buffer.push(END),
                    ESC_ESC => self.buffer.push(ESC),
                    _ => self.buffer.push(byte), // tolerate malformed
                }
            } else {
                match byte {
                    END => {
                        if !self.buffer.is_empty() {
                            let frame = std::mem::take(&mut self.buffer);
                            packets.push(Packet::new(
                                self.start_time.elapsed().as_secs_f64(),
                                Direction::Rx,
                                frame,
                                None,
                                None,
                            ));
                            self.last_data_time = None;
                        }
                    }
                    ESC => {
                        self.slip_escape = true;
                    }
                    _ => {
                        self.buffer.push(byte);
                    }
                }
            }

            // OOM protection
            if self.buffer.len() >= MAX_BUFFER_SIZE {
                let frame = std::mem::take(&mut self.buffer);
                self.slip_escape = false;
                packets.push(Packet::new(
                    self.start_time.elapsed().as_secs_f64(),
                    Direction::Rx,
                    frame,
                    None,
                    None,
                ));
            }
        }

        packets
    }

    /// COBS decoder: accumulate until 0x00 separator, then decode frame
    fn feed_cobs(&mut self, data: &[u8]) -> Vec<Packet> {
        let mut packets = Vec::new();

        for &byte in data {
            if byte == 0x00 {
                // Frame boundary
                if !self.buffer.is_empty() {
                    if let Some(decoded) = cobs_decode(&self.buffer) {
                        packets.push(Packet::new(
                            self.start_time.elapsed().as_secs_f64(),
                            Direction::Rx,
                            decoded,
                            None,
                            None,
                        ));
                        self.last_data_time = None;
                    }
                    self.buffer.clear();
                }
            } else {
                self.buffer.push(byte);
                // OOM protection
                if self.buffer.len() >= MAX_BUFFER_SIZE {
                    self.buffer.clear();
                }
            }
        }

        packets
    }

    /// Flush buffer on gap (pause in data stream). Works for all decoder modes.
    pub fn gap_flush(&mut self) -> Option<Packet> {
        if self.buffer.is_empty() {
            return None;
        }
        if self.decoder_type == DecoderType::Delimiter && self.buffer == self.delimiter {
            return None; // keep delimiter prefix for next packet
        }
        let unframed = self.unframed;
        self.last_data_time = None;
        self.unframed = true; // reset for next accumulation
        let data = std::mem::take(&mut self.buffer);
        self.slip_escape = false;
        let label = if self.decoder_type == DecoderType::Delimiter {
            ubx_label(&data, &self.delimiter)
        } else {
            None
        };
        let mut pkt = Packet::new(
            self.start_time.elapsed().as_secs_f64(),
            Direction::Rx,
            data,
            label,
            None,
        );
        // In Delimiter mode, gap-flushed data that never matched a delimiter is noise
        if self.decoder_type == DecoderType::Delimiter && unframed {
            pkt.noise = true;
        }
        Some(pkt)
    }

    /// Flush buffered data if no new data arrived within the timeout.
    pub fn flush_stale(&mut self, timeout: std::time::Duration) -> Option<Packet> {
        if let Some(last) = self.last_data_time {
            let is_delimiter_only = self.decoder_type == DecoderType::Delimiter
                && self.buffer == self.delimiter;
            if last.elapsed() >= timeout && !self.buffer.is_empty() && !is_delimiter_only {
                self.last_data_time = None;
                return self.flush();
            }
        }
        None
    }

    /// Flush any remaining data as a packet.
    /// For Delimiter mode, skips if buffer contains only the delimiter prefix.
    pub fn flush(&mut self) -> Option<Packet> {
        if self.buffer.is_empty() {
            return None;
        }
        if self.decoder_type == DecoderType::Delimiter && self.buffer == self.delimiter {
            self.buffer.clear();
            return None;
        }
        let unframed = self.unframed;
        self.unframed = true;
        let data = std::mem::take(&mut self.buffer);
        self.slip_escape = false;
        let label = if self.decoder_type == DecoderType::Delimiter {
            ubx_label(&data, &self.delimiter)
        } else {
            None
        };
        let mut pkt = Packet::new(
            self.start_time.elapsed().as_secs_f64(),
            Direction::Rx,
            data,
            label,
            None,
        );
        if self.decoder_type == DecoderType::Delimiter && unframed {
            pkt.noise = true;
        }
        Some(pkt)
    }

    pub fn elapsed(&self) -> f64 {
        self.start_time.elapsed().as_secs_f64()
    }
}

/// COBS (Consistent Overhead Byte Stuffing) decoder.
/// Returns None for invalid encoded data.
fn cobs_decode(encoded: &[u8]) -> Option<Vec<u8>> {
    let mut decoded = Vec::with_capacity(encoded.len());
    let mut i = 0;
    while i < encoded.len() {
        let code = encoded[i];
        if code == 0 {
            return None; // zero byte not allowed in COBS-encoded data
        }
        i += 1;
        let run = (code as usize) - 1;
        if i + run > encoded.len() {
            return None; // not enough data
        }
        for j in 0..run {
            decoded.push(encoded[i + j]);
        }
        i += run;
        // If code < 0xFF and there's more data, a zero was removed here
        if code < 0xFF && i < encoded.len() {
            decoded.push(0x00);
        }
    }
    // Remove trailing zero if it was added by the last group
    if decoded.last() == Some(&0x00) {
        decoded.pop();
    }
    Some(decoded)
}

/// Detect noise packets (floating/unpowered UART lines).
/// Returns true if data looks like line noise:
/// - Packet entirely zeros (any length)
/// - Packet >4 bytes with <=2 unique byte values
/// - Packet exactly MAX_BUFFER_SIZE (no delimiter found in entire buffer — strong noise signal)
/// - Packet >32 bytes with average popcount < 3.0 (floating line, mostly LOW)
pub fn is_noise(data: &[u8]) -> bool {
    if data.is_empty() {
        return false;
    }
    if data.iter().all(|&b| b == 0) {
        return true;
    }
    if data.len() <= 4 {
        return false;
    }
    // Packet hit MAX_BUFFER_SIZE = delimiter never matched = almost certainly noise
    if data.len() >= MAX_BUFFER_SIZE {
        return true;
    }
    // Check unique byte values
    let mut seen = [false; 256];
    let mut unique = 0u16;
    let mut total_bits = 0u32;
    for &b in data {
        if !seen[b as usize] {
            seen[b as usize] = true;
            unique += 1;
        }
        total_bits += b.count_ones();
    }
    if unique <= 2 {
        return true;
    }
    // Average popcount check for longer packets.
    // Floating UART lines produce bytes with avg popcount ~2.5.
    // Real data (protocols, ASCII) has avg popcount ~3.5-4.5.
    if data.len() > 32 {
        let avg_popcount = total_bits as f32 / data.len() as f32;
        if avg_popcount < 3.0 {
            return true;
        }
    }
    false
}

/// Count occurrences of `delimiter` in `data` using sliding window.
pub fn count_delimiter(data: &[u8], delimiter: &[u8]) -> usize {
    if delimiter.is_empty() || data.len() < delimiter.len() {
        return 0;
    }
    let mut count = 0;
    for i in 0..=data.len() - delimiter.len() {
        if &data[i..i + delimiter.len()] == delimiter {
            count += 1;
        }
    }
    count
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
