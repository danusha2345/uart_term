/// Decoder/framing type
#[derive(Clone, Copy, PartialEq)]
pub enum DecoderType {
    Raw,
    Delimiter,
    Slip,
    Cobs,
    Nmea,
}

impl DecoderType {
    pub fn label(&self) -> &str {
        match self {
            Self::Raw => "Raw",
            Self::Delimiter => "Delimiter",
            Self::Slip => "SLIP",
            Self::Cobs => "COBS",
            Self::Nmea => "NMEA",
        }
    }

    pub const ALL: [DecoderType; 5] = [
        Self::Raw,
        Self::Delimiter,
        Self::Slip,
        Self::Cobs,
        Self::Nmea,
    ];
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
    /// Source index for color coding (0 = UART1, 1 = UART2)
    pub source_idx: Option<u8>,
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
            source_idx: None,
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

/// Verify UBX Fletcher-8 checksum.
/// Returns Some(true) if CRC matches, Some(false) if it doesn't,
/// None if the packet is too short or malformed to check.
pub fn ubx_crc_valid(data: &[u8]) -> Option<bool> {
    // Minimum UBX frame: sync(2) + class(1) + id(1) + length(2) + checksum(2) = 8
    if data.len() < 8 || data[0] != 0xB5 || data[1] != 0x62 {
        return None;
    }
    let payload_len = u16::from_le_bytes([data[4], data[5]]) as usize;
    let expected_len = 8 + payload_len; // sync(2) + class(1) + id(1) + len(2) + payload + ck(2)
    if data.len() != expected_len {
        return Some(false); // length mismatch
    }
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in &data[2..data.len() - 2] {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    Some(ck_a == data[data.len() - 2] && ck_b == data[data.len() - 1])
}

/// Try to generate a UBX label for the packet.
/// Only applies when delimiter is [0xB5, 0x62].
/// Appends [CRC!] if checksum validation fails (indicates data corruption).
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
    let mut label = if name.is_empty() {
        format!("[0x{:02X}-0x{:02X}]", cls, id)
    } else {
        format!("[{}-0x{:02X}]", name, id)
    };
    if let Some(false) = ubx_crc_valid(data) {
        label.push_str(" [CRC!]");
    }
    Some(label)
}

/// Verify NMEA 0183 checksum (XOR of bytes between `$` and `*`).
/// Returns Some(true) on match, Some(false) on mismatch, None if the frame
/// is malformed or has no valid `*XX` tail.
pub fn nmea_crc_valid(data: &[u8]) -> Option<bool> {
    if data.len() < 4 || data[0] != b'$' {
        return None;
    }
    let star = data.iter().rposition(|&b| b == b'*')?;
    // Need at least 2 hex chars after `*` (trailing CRLF already stripped upstream)
    if star + 3 > data.len() {
        return None;
    }
    let hex = std::str::from_utf8(&data[star + 1..star + 3]).ok()?;
    let expected = u8::from_str_radix(hex, 16).ok()?;
    let mut xor: u8 = 0;
    for &b in &data[1..star] {
        xor ^= b;
    }
    Some(xor == expected)
}

/// Build label `[talker-sentence]` for an NMEA sentence.
/// Standard: `$ttsss,...` (2-char talker + 3-char sentence).
/// Proprietary: `$Pmmm,...` (single `P` prefix + manufacturer code).
/// Appends `[CRC!]` on checksum mismatch.
pub fn nmea_label(data: &[u8]) -> Option<String> {
    if data.len() < 2 || data[0] != b'$' {
        return None;
    }
    // Find end of header (first `,` or `*`, or end of data)
    let end = data[1..]
        .iter()
        .position(|&b| b == b',' || b == b'*')
        .map(|p| p + 1)
        .unwrap_or(data.len());
    let head = &data[1..end];
    if head.is_empty() || !head.iter().all(|b| b.is_ascii_alphanumeric()) {
        return None;
    }
    let mut label = if head[0] == b'P' && head.len() > 1 {
        // Proprietary: $Pxxx... — after 'P' the rest is the manufacturer/command code
        let rest = std::str::from_utf8(&head[1..]).ok()?;
        format!("[P-{}]", rest)
    } else if head.len() >= 5 {
        let talker = std::str::from_utf8(&head[..2]).ok()?;
        let sentence = std::str::from_utf8(&head[2..]).ok()?;
        format!("[{}-{}]", talker, sentence)
    } else {
        let tag = std::str::from_utf8(head).ok()?;
        format!("[{}]", tag)
    };
    if let Some(false) = nmea_crc_valid(data) {
        label.push_str(" [CRC!]");
    }
    Some(label)
}

/// Maximum buffer size before forced flush (protection against OOM).
/// Sized for the UBX worst case (64 KiB payload + framing) plus headroom.
const MAX_BUFFER_SIZE: usize = 72 * 1024;

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
    pub fn new(delimiter: Vec<u8>, start_time: std::time::Instant) -> Self {
        Self {
            delimiter,
            buffer: Vec::with_capacity(1024),
            start_time,
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

    pub fn set_delimiter(&mut self, delimiter: Vec<u8>) {
        self.delimiter = delimiter;
        // Old buffer was framed against the previous delimiter — discard it
        // so the next packet is not silently merged with stale bytes.
        self.buffer.clear();
        self.slip_escape = false;
        self.unframed = true;
        self.last_data_time = None;
    }

    /// Feed bytes into the parser, returns completed packets.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Packet> {
        self.last_data_time = Some(std::time::Instant::now());
        match self.decoder_type {
            DecoderType::Raw => self.feed_raw(data),
            DecoderType::Delimiter => self.feed_delimiter(data),
            DecoderType::Slip => self.feed_slip(data),
            DecoderType::Cobs => self.feed_cobs(data),
            DecoderType::Nmea => self.feed_nmea(data),
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
            // Flush buffer BEFORE pushing if it exceeds max size (OOM protection).
            // Don't skip delimiter check — the new byte may complete a delimiter.
            if self.buffer.len() >= MAX_BUFFER_SIZE {
                let unframed = self.unframed;
                let buf = std::mem::take(&mut self.buffer);
                let mut pkt = Packet::new(
                    self.start_time.elapsed().as_secs_f64(),
                    Direction::Rx,
                    buf,
                    None,
                    None,
                );
                if unframed {
                    pkt.noise = true;
                }
                packets.push(pkt);
                self.unframed = true;
            }

            self.buffer.push(byte);

            // Check if the buffer ends with the delimiter
            if self.buffer.len() >= self.delimiter.len() && !self.delimiter.is_empty() {
                let buf_len = self.buffer.len();
                let delim_len = self.delimiter.len();

                if self.buffer[buf_len - delim_len..] == self.delimiter[..] {
                    let was_unframed = self.unframed;
                    self.unframed = false;
                    // Everything before the delimiter is a complete packet
                    let packet_data: Vec<u8> = self.buffer[..buf_len - delim_len].to_vec();

                    if !packet_data.is_empty() && packet_data != self.delimiter {
                        let label = ubx_label(&packet_data, &self.delimiter);
                        let mut pkt = Packet::new(
                            self.start_time.elapsed().as_secs_f64(),
                            Direction::Rx,
                            packet_data,
                            label,
                            None,
                        );
                        // Data captured before the very first delimiter is startup garbage
                        if was_unframed {
                            pkt.noise = true;
                        }
                        packets.push(pkt);
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

    /// NMEA 0183 decoder: frame = `$`..`\n`, trailing CR/LF stripped from body.
    /// Two states, keyed on `self.unframed`:
    /// - `unframed=true` — pre-sync: accumulate bytes until `$`, then emit
    ///   the preamble (if any) as a noise packet and start a new frame with `$`.
    /// - `unframed=false` — in-frame: accumulate until `\n`, then emit body
    ///   with CR/LF stripped and `[talker-sentence]` label.
    fn feed_nmea(&mut self, data: &[u8]) -> Vec<Packet> {
        let mut packets = Vec::new();

        for &byte in data {
            if self.unframed {
                // Pre-sync: wait for `$`.
                if byte == b'$' {
                    // Flush any accumulated pre-preamble bytes as noise.
                    if !self.buffer.is_empty() {
                        let pre = std::mem::take(&mut self.buffer);
                        let mut pkt = Packet::new(
                            self.start_time.elapsed().as_secs_f64(),
                            Direction::Rx,
                            pre,
                            None,
                            None,
                        );
                        pkt.noise = true;
                        packets.push(pkt);
                    }
                    self.buffer.push(byte);
                    self.unframed = false;
                } else {
                    self.buffer.push(byte);
                    if self.buffer.len() >= MAX_BUFFER_SIZE {
                        let overflow = std::mem::take(&mut self.buffer);
                        let mut pkt = Packet::new(
                            self.start_time.elapsed().as_secs_f64(),
                            Direction::Rx,
                            overflow,
                            None,
                            None,
                        );
                        pkt.noise = true;
                        packets.push(pkt);
                    }
                }
            } else {
                // In-frame: collect until `\n`.
                self.buffer.push(byte);
                if byte == b'\n' {
                    let mut body = std::mem::take(&mut self.buffer);
                    while let Some(&last) = body.last() {
                        if last == b'\n' || last == b'\r' {
                            body.pop();
                        } else {
                            break;
                        }
                    }
                    if !body.is_empty() {
                        let label = nmea_label(&body);
                        packets.push(Packet::new(
                            self.start_time.elapsed().as_secs_f64(),
                            Direction::Rx,
                            body,
                            label,
                            None,
                        ));
                        self.last_data_time = None;
                    }
                    // Back to pre-sync; next frame must start with `$`.
                    self.unframed = true;
                } else if self.buffer.len() >= MAX_BUFFER_SIZE {
                    // Runaway frame with no LF — dump as noise and resync.
                    let overflow = std::mem::take(&mut self.buffer);
                    let mut pkt = Packet::new(
                        self.start_time.elapsed().as_secs_f64(),
                        Direction::Rx,
                        overflow,
                        None,
                        None,
                    );
                    pkt.noise = true;
                    packets.push(pkt);
                    self.unframed = true;
                }
            }
        }

        packets
    }

    /// True when the current buffer holds an in-progress framed packet that
    /// a short pause should NOT flush prematurely. flush() (disconnect) and
    /// flush_stale() (long timeout) bypass this check on purpose.
    fn is_mid_frame(&self) -> bool {
        match self.decoder_type {
            DecoderType::Raw => false,
            DecoderType::Delimiter => {
                !self.unframed
                    && self.buffer.starts_with(&self.delimiter)
                    && self.buffer.len() > self.delimiter.len()
            }
            DecoderType::Nmea => !self.unframed && self.buffer.first() == Some(&b'$'),
            // SLIP and COBS clear the buffer at every frame boundary
            // (END / 0x00). Any pending bytes mean we are mid-frame.
            DecoderType::Slip | DecoderType::Cobs => !self.buffer.is_empty(),
        }
    }

    /// Common path: take buffer, build the right label, build the Packet,
    /// flag startup garbage as noise. Used by both flush() and gap_flush().
    fn finalize_buffer(&mut self) -> Option<Packet> {
        if self.buffer.is_empty() {
            return None;
        }
        let unframed = self.unframed;
        self.unframed = true;
        self.slip_escape = false;
        self.last_data_time = None;
        let data = std::mem::take(&mut self.buffer);
        let label = match self.decoder_type {
            DecoderType::Delimiter => ubx_label(&data, &self.delimiter),
            DecoderType::Nmea => nmea_label(&data),
            _ => None,
        };
        let mut pkt = Packet::new(
            self.start_time.elapsed().as_secs_f64(),
            Direction::Rx,
            data,
            label,
            None,
        );
        if matches!(
            self.decoder_type,
            DecoderType::Delimiter | DecoderType::Nmea
        ) && unframed
        {
            pkt.noise = true;
        }
        Some(pkt)
    }

    /// Flush buffer on gap (pause in data stream). Works for all decoder modes.
    /// Skips in-progress frames — those wait for flush_stale() or flush().
    pub fn gap_flush(&mut self) -> Option<Packet> {
        if self.buffer.is_empty() {
            return None;
        }
        if self.decoder_type == DecoderType::Delimiter && self.buffer == self.delimiter {
            return None; // keep delimiter prefix for next packet
        }
        if self.is_mid_frame() {
            return None;
        }
        self.finalize_buffer()
    }

    /// Flush buffered data if no new data arrived within the timeout.
    /// Bypasses mid-frame guard — long pause means the frame is lost anyway.
    pub fn flush_stale(&mut self, timeout: std::time::Duration) -> Option<Packet> {
        if let Some(last) = self.last_data_time {
            let is_delimiter_only =
                self.decoder_type == DecoderType::Delimiter && self.buffer == self.delimiter;
            if last.elapsed() >= timeout && !self.buffer.is_empty() && !is_delimiter_only {
                return self.flush();
            }
        }
        None
    }

    /// Flush any remaining data as a packet (disconnect path).
    /// For Delimiter mode, skips if buffer contains only the delimiter prefix.
    pub fn flush(&mut self) -> Option<Packet> {
        if self.decoder_type == DecoderType::Delimiter && self.buffer == self.delimiter {
            self.buffer.clear();
            return None;
        }
        self.finalize_buffer()
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
    Some(decoded)
}

/// Detect noise packets (floating/unpowered UART lines).
/// Returns true if data looks like line noise:
/// - Packet entirely zeros (any length)
/// - Packet >4 bytes with <=2 unique byte values
/// - Packet >32 bytes with average popcount < 3.0 (floating line, mostly LOW)
///
/// Note: hitting MAX_BUFFER_SIZE is handled separately — the parser
/// marks such packets as noise only when they never contained a delimiter
/// (`unframed == true`), so large legitimate frames are not misclassified.
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

    if !clean.len().is_multiple_of(2) {
        return Err("Odd number of hex digits".to_string());
    }

    let mut bytes = Vec::new();
    for i in (0..clean.len()).step_by(2) {
        let byte = u8::from_str_radix(&clean[i..i + 2], 16).map_err(|e| e.to_string())?;
        bytes.push(byte);
    }
    Ok(bytes)
}

#[cfg(test)]
mod parser_tests {
    use super::*;
    use std::time::Instant;

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

    #[test]
    fn cobs_decode_roundtrip_cases() {
        let cases: &[(&[u8], &[u8])] = &[
            (&[0x01], &[]),
            (&[0x01, 0x01], &[0x00]),
            (&[0x03, 0x11, 0x22, 0x01], &[0x11, 0x22, 0x00]),
            (&[0x02, 0x11, 0x02, 0x22], &[0x11, 0x00, 0x22]),
        ];

        for (encoded, decoded) in cases {
            assert_eq!(cobs_decode(encoded).as_deref(), Some(*decoded));
        }
    }

    #[test]
    fn feed_cobs_emits_decoded_frames() {
        let mut parser = StreamParser::new(vec![], Instant::now());
        parser.set_decoder_type(DecoderType::Cobs);

        let packets = parser.feed(&[
            0x01, 0x00, // []
            0x03, 0x11, 0x22, 0x01, 0x00, // [0x11, 0x22, 0x00]
            0x02, 0x11, 0x02, 0x22, 0x00, // [0x11, 0x00, 0x22]
        ]);

        assert_eq!(packets.len(), 3);
        assert_eq!(packets[0].data, Vec::<u8>::new());
        assert_eq!(packets[1].data, vec![0x11, 0x22, 0x00]);
        assert_eq!(packets[2].data, vec![0x11, 0x00, 0x22]);
    }

    #[test]
    fn ubx_crc_known_good_and_corrupted() {
        let frame = ubx_frame(0x06, 0x01, &[0x01, 0x02, 0x03]);
        assert_eq!(ubx_crc_valid(&frame), Some(true));
        assert_eq!(
            ubx_label(&frame, &[0xB5, 0x62]).as_deref(),
            Some("[CFG-0x01]")
        );

        let mut corrupted = frame;
        corrupted[6] ^= 0xFF;
        assert_eq!(ubx_crc_valid(&corrupted), Some(false));
        assert_eq!(
            ubx_label(&corrupted, &[0xB5, 0x62]).as_deref(),
            Some("[CFG-0x01] [CRC!]")
        );
    }

    #[test]
    fn delimiter_back_to_back_frames_emit_first_and_flush_tail() {
        let frame1 = ubx_frame(0x01, 0x07, &[0xAA, 0xBB]);
        let frame2 = ubx_frame(0x06, 0x41, &[0x01]);
        let mut stream = frame1.clone();
        stream.extend_from_slice(&frame2);

        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let packets = parser.feed(&stream);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].data, frame1);

        let tail = parser.flush().expect("tail frame should flush");
        assert_eq!(tail.data, frame2);
    }

    #[test]
    fn delimiter_frame_split_across_feed_calls() {
        let frame = ubx_frame(0x01, 0x07, &[0xAA, 0xBB]);
        let next_delimiter = [0xB5, 0x62];
        let split = 5;

        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        assert!(parser.feed(&frame[..split]).is_empty());

        let mut rest = frame[split..].to_vec();
        rest.extend_from_slice(&next_delimiter);
        let packets = parser.feed(&rest);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].data, frame);
        assert!(!packets[0].noise);
    }

    #[test]
    fn delimiter_startup_preamble_is_noise() {
        let frame = ubx_frame(0x01, 0x07, &[0xAA, 0xBB]);
        let mut stream = b"junk before sync".to_vec();
        stream.extend_from_slice(&frame);
        stream.extend_from_slice(&[0xB5, 0x62]);

        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let packets = parser.feed(&stream);
        assert_eq!(packets.len(), 2);
        assert!(packets[0].noise);
        assert_eq!(packets[0].data, b"junk before sync");
        assert_eq!(packets[1].data, frame);
    }

    #[test]
    fn delimiter_large_frame_below_limit_passes() {
        let payload = vec![0xAA; 70 * 1024];
        let mut stream = vec![0xB5, 0x62];
        stream.extend_from_slice(&payload);
        stream.extend_from_slice(&[0xB5, 0x62]);

        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let packets = parser.feed(&stream);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].data.len(), 2 + payload.len());
        assert!(!packets[0].noise);
    }

    #[test]
    fn delimiter_over_max_forced_flush_marks_noise() {
        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let packets = parser.feed(&vec![0x55; MAX_BUFFER_SIZE + 1]);

        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].data.len(), MAX_BUFFER_SIZE);
        assert!(packets[0].noise);
    }

    #[test]
    fn slip_gap_flush_mid_frame_returns_none() {
        // SLIP frames are bounded by END (0xC0). Buffer non-empty without END
        // means we're mid-frame — gap_flush must NOT emit a half-decoded packet.
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Slip);
        let _ = p.feed(&[0x11, 0x22, 0x33]);
        assert!(
            p.gap_flush().is_none(),
            "SLIP gap_flush must not emit mid-frame data"
        );
        // Disconnect-time flush() should still drain it.
        let tail = p.flush().expect("flush must drain");
        assert_eq!(tail.data, vec![0x11, 0x22, 0x33]);
    }

    #[test]
    fn cobs_gap_flush_mid_frame_returns_none() {
        // COBS clears buffer at every 0x00 boundary; non-empty buffer = mid-frame.
        // gap_flush must not emit raw (un-decoded) bytes as a "packet".
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Cobs);
        let _ = p.feed(&[0x03, 0x11, 0x22]); // partial COBS run
        assert!(p.gap_flush().is_none(), "COBS gap_flush must not flush mid-frame");
    }

    #[test]
    fn set_delimiter_clears_stale_buffer() {
        // After delimiter change, leftover bytes must NOT silently merge with
        // the next packet under the new delimiter.
        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let _ = parser.feed(&[0xB5, 0x62, 0xAA, 0xBB]); // mid-frame UBX

        parser.set_delimiter(vec![0x24]); // switch to '$'
        let pkts = parser.feed(b"$GP\n");
        // Buffer was reset to unframed; '$' starts a new frame, so '$GP\n'
        // accumulates and is split at the '$' boundary.
        // Important: previous 0xAA 0xBB must NOT appear in any emitted packet.
        for p in &pkts {
            assert!(!p.data.contains(&0xAA), "stale AA leaked: {:?}", p.data);
            assert!(!p.data.contains(&0xBB), "stale BB leaked: {:?}", p.data);
        }
        if let Some(tail) = parser.flush() {
            assert!(!tail.data.contains(&0xAA));
            assert!(!tail.data.contains(&0xBB));
        }
    }

    #[test]
    fn delimiter_mid_frame_gap_flush_returns_none() {
        // Same guarantee as NMEA mid_frame_gap_flush_returns_none, for delimiter.
        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let frame = ubx_frame(0x01, 0x07, &[0xAA, 0xBB]);
        let split = 5;
        let _ = parser.feed(&frame[..split]); // start frame
        // Note: first feed flags it as unframed=true initially because no
        // delimiter has *completed* yet. Push a completing delimiter first.
        let mut warmup = frame.clone();
        warmup.extend_from_slice(&[0xB5, 0x62]);
        let mut parser = StreamParser::new(vec![0xB5, 0x62], Instant::now());
        let _ = parser.feed(&warmup); // emits one packet, leaves delimiter prefix
        let _ = parser.feed(&[0xAA, 0xBB]); // mid-frame after sync established
        assert!(
            parser.gap_flush().is_none(),
            "Delimiter gap_flush must not split mid-frame"
        );
    }
}

#[cfg(test)]
mod nmea_tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn crc_known_good() {
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        assert_eq!(nmea_crc_valid(gga), Some(true));
        assert_eq!(nmea_crc_valid(rmc), Some(true));
    }

    #[test]
    fn crc_bad() {
        let bad = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
        assert_eq!(nmea_crc_valid(bad), Some(false));
    }

    #[test]
    fn label_standard() {
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        assert_eq!(nmea_label(gga).as_deref(), Some("[GP-GGA]"));
        let rmc = b"$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*70";
        // label produced regardless of CRC match, with [CRC!] appended on mismatch
        let l = nmea_label(rmc).unwrap();
        assert!(l.starts_with("[GN-RMC]"));
    }

    #[test]
    fn label_proprietary() {
        let pubx = b"$PUBX,00,123519.00*33";
        let l = nmea_label(pubx).unwrap();
        assert!(l.starts_with("[P-UBX]"), "got {}", l);
    }

    #[test]
    fn label_crc_mismatch_suffix() {
        let broken = b"$GPGGA,1*00";
        let l = nmea_label(broken).unwrap();
        assert!(l.ends_with("[CRC!]"), "got {}", l);
    }

    #[test]
    fn framing_splits_per_line() {
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Nmea);
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
        let mut stream = Vec::new();
        stream.extend_from_slice(gga);
        stream.extend_from_slice(b"\r\n");
        stream.extend_from_slice(rmc);
        stream.extend_from_slice(b"\r\n");
        let pkts = p.feed(&stream);
        assert_eq!(pkts.len(), 2);
        assert_eq!(&pkts[0].data[..], gga); // trailing CRLF stripped
        assert_eq!(&pkts[1].data[..], rmc);
        assert_eq!(pkts[0].label.as_deref(), Some("[GP-GGA]"));
        assert_eq!(pkts[1].label.as_deref(), Some("[GP-RMC]"));
        assert!(!pkts[0].noise);
    }

    #[test]
    fn pre_preamble_marked_noise() {
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Nmea);
        let mut stream = Vec::new();
        stream.extend_from_slice(b"junk before sync");
        stream.extend_from_slice(b"$GPGGA,1*00\r\n");
        let pkts = p.feed(&stream);
        assert_eq!(pkts.len(), 2);
        assert!(pkts[0].noise);
        assert_eq!(&pkts[0].data[..], b"junk before sync");
        assert!(!pkts[1].noise);
    }

    #[test]
    fn mid_frame_gap_flush_returns_none() {
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Nmea);
        let _ = p.feed(b"$GPGGA,123519");
        assert!(p.gap_flush().is_none(), "should not flush mid-frame");
    }

    #[test]
    fn disconnect_flush_emits_partial_frame() {
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Nmea);
        let _ = p.feed(b"$GPVTG,,T,,M,022.4,N,041.5,K,A");
        let tail = p.flush().expect("partial frame must flush on disconnect");
        assert!(!tail.noise);
        assert_eq!(&tail.data[..], b"$GPVTG,,T,,M,022.4,N,041.5,K,A");
    }

    #[test]
    fn byte_by_byte_feed_equivalent_to_bulk() {
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let mut stream = Vec::new();
        stream.extend_from_slice(gga);
        stream.extend_from_slice(rmc);

        // Bulk
        let mut bulk = StreamParser::new(vec![], Instant::now());
        bulk.set_decoder_type(DecoderType::Nmea);
        let bulk_pkts = bulk.feed(&stream);

        // Byte-by-byte (simulates slow UART, one-byte reads)
        let mut slow = StreamParser::new(vec![], Instant::now());
        slow.set_decoder_type(DecoderType::Nmea);
        let mut slow_pkts = Vec::new();
        for b in &stream {
            slow_pkts.extend(slow.feed(&[*b]));
        }

        assert_eq!(bulk_pkts.len(), slow_pkts.len());
        for (a, b) in bulk_pkts.iter().zip(slow_pkts.iter()) {
            assert_eq!(a.data, b.data);
            assert_eq!(a.label, b.label);
            assert_eq!(a.noise, b.noise);
        }
    }

    #[test]
    fn chunked_feed_across_crlf_boundary() {
        // Chunk boundary lands between \r and \n — parser must still emit clean body.
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Nmea);
        let _ = p.feed(b"$GPGGA,1*00\r");
        let pkts = p.feed(b"\n$GPRMC,1*00\r\n");
        assert_eq!(pkts.len(), 2);
        assert_eq!(&pkts[0].data[..], b"$GPGGA,1*00");
        assert_eq!(&pkts[1].data[..], b"$GPRMC,1*00");
    }

    #[test]
    fn resync_after_frame_flush() {
        let mut p = StreamParser::new(vec![], Instant::now());
        p.set_decoder_type(DecoderType::Nmea);
        let _ = p.feed(b"$GPGGA,1*00\r\n");
        // Now some garbage between frames (shouldn't normally happen but be robust):
        let pkts = p.feed(b"xxx$GPRMC,1*00\r\n");
        // First packet: noise "xxx"; second: the RMC.
        assert_eq!(pkts.len(), 2);
        assert!(pkts[0].noise);
        assert_eq!(&pkts[0].data[..], b"xxx");
        assert!(!pkts[1].noise);
        assert_eq!(&pkts[1].data[..], b"$GPRMC,1*00");
    }
}
