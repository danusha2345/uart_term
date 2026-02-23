use eframe::egui;
use uuid::Uuid;

use crate::ble::{BleCharInfo, BleCmd, BleDeviceInfo, BleHandle, BleMsg};
use crate::logger::Logger;
use crate::parser::{self, DecoderType, Direction, Packet, StreamParser};
use crate::serial::{self, SerialHandle, SerialMsg};

/// Auto-format hex input: uppercase, space between every 2 hex digits
fn format_hex_input(raw: &str) -> String {
    let hex_chars: Vec<char> = raw.chars().filter(|c| c.is_ascii_hexdigit()).collect();

    let mut out = String::with_capacity(hex_chars.len() * 3 / 2);
    for (i, ch) in hex_chars.iter().enumerate() {
        if i > 0 && i % 2 == 0 {
            out.push(' ');
        }
        out.push(ch.to_ascii_uppercase());
    }
    out
}

/// Extract short port name from full path: "/dev/ttyUSB0" -> "ttyUSB0", "COM3" -> "COM3"
fn short_port_name(port: &str) -> String {
    port.rsplit('/').next().unwrap_or(port).to_string()
}

/// Line ending appended to sent data
#[derive(Clone, Copy, PartialEq)]
pub enum LineEnding {
    None,
    Cr,
    Lf,
    CrLf,
}

impl LineEnding {
    pub fn label(&self) -> &str {
        match self {
            Self::None => "None",
            Self::Cr => "CR",
            Self::Lf => "LF",
            Self::CrLf => "CRLF",
        }
    }

    pub fn bytes(&self) -> &[u8] {
        match self {
            Self::None => &[],
            Self::Cr => &[0x0D],
            Self::Lf => &[0x0A],
            Self::CrLf => &[0x0D, 0x0A],
        }
    }

    const ALL: [LineEnding; 4] = [Self::None, Self::Cr, Self::Lf, Self::CrLf];
}

/// Common baud rates
const BAUD_RATES: &[u32] = &[
    9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000, 3000000,
];

/// Transport mode selector
#[derive(Clone, Copy, PartialEq)]
pub enum TransportMode {
    Serial,
    Ble,
}

/// Data display/log format
#[derive(Clone, Copy, PartialEq)]
pub enum DataFormat {
    Hex,
    Ascii,
    HexAscii,
}

/// What to include in log file (re-export for logger)
pub type LogFormat = DataFormat;

/// Per-connection serial state
struct SerialConn {
    label: String,
    selected_port: String,
    selected_baud: u32,
    baud_input: String,
    data_bits: serialport::DataBits,
    parity: serialport::Parity,
    stop_bits: serialport::StopBits,
    flow_control: serialport::FlowControl,
    connection: Option<SerialHandle>,
    dtr_state: bool,
    rts_state: bool,
    decoder_type: DecoderType,
    parser: StreamParser,
}

impl SerialConn {
    fn is_connected(&self) -> bool {
        self.connection.is_some()
    }

    fn new(label: &str, default_port: &str, delimiter: Vec<u8>) -> Self {
        Self {
            label: label.to_string(),
            selected_port: default_port.to_string(),
            selected_baud: 921600,
            baud_input: "921600".to_string(),
            data_bits: serialport::DataBits::Eight,
            parity: serialport::Parity::None,
            stop_bits: serialport::StopBits::One,
            flow_control: serialport::FlowControl::None,
            connection: None,
            dtr_state: false,
            rts_state: false,
            decoder_type: DecoderType::Delimiter,
            parser: StreamParser::new(delimiter),
        }
    }

    fn connect(&mut self, delimiter_input: &str) -> Result<String, String> {
        match SerialHandle::connect(
            &self.selected_port,
            self.selected_baud,
            self.data_bits,
            self.parity,
            self.stop_bits,
            self.flow_control,
        ) {
            Ok(handle) => {
                self.connection = Some(handle);
                // Reset parser with current delimiter and decoder type
                if let Ok(delim) = parser::parse_delimiter_input(delimiter_input) {
                    self.parser = StreamParser::new(delim);
                } else {
                    self.parser = StreamParser::new(vec![0xB5, 0x62]);
                }
                self.parser.set_decoder_type(self.decoder_type);
                Ok(format!(
                    "{}: {} @ {}",
                    self.label, self.selected_port, self.selected_baud
                ))
            }
            Err(e) => Err(e),
        }
    }

    fn disconnect(&mut self) -> Option<Packet> {
        if let Some(handle) = self.connection.take() {
            handle.disconnect();
        }
        self.parser.flush()
    }

    /// Poll serial handle, push packets into shared vec, log them.
    /// Returns an error/status message if something happened.
    fn poll(&mut self, packets: &mut Vec<Packet>, logger: &mut Option<Logger>, noise_filter: bool) -> Option<String> {
        if self.connection.is_none() {
            return None;
        }

        let mut new_packets = Vec::new();
        let mut error_msg = None;
        let mut lost_connection = false;

        if let Some(ref handle) = self.connection {
            while let Ok(msg) = handle.rx.try_recv() {
                match msg {
                    SerialMsg::Data(data) => {
                        let pkts = self.parser.feed(&data);
                        new_packets.extend(pkts);
                    }
                    SerialMsg::Gap => {
                        if let Some(pkt) = self.parser.gap_flush() {
                            new_packets.push(pkt);
                        }
                    }
                    SerialMsg::Error(e) => {
                        error_msg = Some(format!("{}: Error: {}", self.label, e));
                    }
                    SerialMsg::Disconnected => {
                        lost_connection = true;
                        error_msg =
                            Some(format!("{}: Disconnected (device lost)", self.label));
                        break;
                    }
                }
            }
        }

        if lost_connection {
            self.connection = None;
            // Flush parser buffer as partial packet
            if let Some(pkt) = self.parser.flush() {
                new_packets.push(pkt);
            }
        }

        // Set source on all new packets
        let source = short_port_name(&self.selected_port);
        for pkt in &mut new_packets {
            pkt.source = Some(source.clone());
        }

        // Log (skip noise when filter is on)
        for pkt in &new_packets {
            if !(noise_filter && pkt.noise) {
                if let Some(ref mut l) = logger {
                    l.log_packet(pkt);
                }
            }
        }
        if !new_packets.is_empty() {
            if let Some(ref mut l) = logger {
                l.flush();
            }
        }
        packets.extend(new_packets);

        error_msg
    }
}

pub struct UartTermApp {
    // Transport
    transport_mode: TransportMode,

    // Serial connections (two simultaneous)
    serial: [SerialConn; 2],
    send_target: usize,
    available_ports: Vec<String>,
    status_msg: String,

    // BLE
    ble_handle: Option<BleHandle>,
    ble_scanning: bool,
    ble_devices: Vec<BleDeviceInfo>,
    ble_selected_device: Option<String>,
    ble_connected: bool,
    ble_chars: Vec<BleCharInfo>,
    ble_notify_char: Option<(Uuid, Uuid)>,
    ble_write_char: Option<(Uuid, Uuid)>,
    ble_notify_idx: usize,
    ble_write_idx: usize,

    // BLE parser (serial parsers are per-connection)
    parser: StreamParser,
    ble_decoder_type: DecoderType,
    delimiter_input: String,

    // Display
    display_format: DataFormat,
    packets: Vec<Packet>,
    auto_scroll: bool,
    max_packets: usize,
    filter_input: String,
    filter_bytes: Vec<u8>,
    noise_filter: bool,

    // Send
    send_input: String,
    line_ending: LineEnding,
    send_history: Vec<String>,
    send_history_idx: Option<usize>,

    // Logging
    log_enabled: bool,
    log_format: LogFormat,
    log_path: String,
    logger: Option<Logger>,
    file_dialog_rx: Option<std::sync::mpsc::Receiver<String>>,
}

impl UartTermApp {
    pub fn new() -> Self {
        let ports = serial::list_ports();
        let default_port = ports.first().cloned().unwrap_or_default();
        let default_delim = vec![0xB5, 0x62];

        Self {
            transport_mode: TransportMode::Serial,

            serial: [
                SerialConn::new("UART1", &default_port, default_delim.clone()),
                SerialConn::new("UART2", &default_port, default_delim.clone()),
            ],
            send_target: 0,
            available_ports: ports,
            status_msg: "Disconnected".to_string(),

            ble_handle: None,
            ble_scanning: false,
            ble_devices: Vec::new(),
            ble_selected_device: None,
            ble_connected: false,
            ble_chars: Vec::new(),
            ble_notify_char: None,
            ble_write_char: None,
            ble_notify_idx: 0,
            ble_write_idx: 0,

            parser: StreamParser::new(default_delim),
            ble_decoder_type: DecoderType::Delimiter,
            delimiter_input: "B5 62".to_string(),

            display_format: DataFormat::HexAscii,
            packets: Vec::new(),
            auto_scroll: true,
            max_packets: 10000,
            filter_input: String::new(),
            filter_bytes: Vec::new(),
            noise_filter: true,

            send_input: String::new(),
            line_ending: LineEnding::None,
            send_history: Vec::new(),
            send_history_idx: None,

            log_enabled: true,
            log_format: LogFormat::Hex,
            log_path: Self::make_log_path(),
            logger: None,
            file_dialog_rx: None,
        }
    }

    fn make_log_path() -> String {
        chrono::Local::now()
            .format("log_%Y-%m-%d_%H-%M-%S.txt")
            .to_string()
    }

    fn start_logger(&mut self) {
        if self.log_enabled {
            self.log_path = Self::make_log_path();
            match Logger::new(&self.log_path, self.log_format) {
                Ok(l) => self.logger = Some(l),
                Err(e) => self.status_msg = format!("Log error: {}", e),
            }
        }
    }

    fn is_connected(&self) -> bool {
        match self.transport_mode {
            TransportMode::Serial => self.serial[0].is_connected() || self.serial[1].is_connected(),
            TransportMode::Ble => self.ble_connected,
        }
    }

    // --- Serial ---

    fn serial_connect(&mut self, idx: usize) {
        let delimiter_input = self.delimiter_input.clone();
        match self.serial[idx].connect(&delimiter_input) {
            Ok(msg) => {
                self.status_msg = msg;
                if self.logger.is_none() {
                    self.start_logger();
                }
            }
            Err(e) => {
                self.status_msg = e;
            }
        }
    }

    fn serial_disconnect(&mut self, idx: usize) {
        if let Some(mut pkt) = self.serial[idx].disconnect() {
            pkt.source = Some(short_port_name(&self.serial[idx].selected_port));
            if !(self.noise_filter && pkt.noise) {
                if let Some(ref mut logger) = self.logger {
                    logger.log_packet(&pkt);
                    logger.flush();
                }
            }
            self.packets.push(pkt);
        }
        // Update status
        let any_connected = self.serial[0].is_connected() || self.serial[1].is_connected();
        if !any_connected {
            self.status_msg = "Disconnected".to_string();
        } else {
            for conn in &self.serial {
                if conn.is_connected() {
                    self.status_msg = format!(
                        "{}: {} @ {}",
                        conn.label, conn.selected_port, conn.selected_baud
                    );
                    break;
                }
            }
        }
    }

    fn scan_ports(&mut self) {
        self.available_ports = serial::list_ports();
        let ports = self.available_ports.clone();
        for conn in &mut self.serial {
            if !ports.contains(&conn.selected_port) {
                conn.selected_port = ports.first().cloned().unwrap_or_default();
            }
        }
    }

    fn poll_serial(&mut self) {
        let nf = self.noise_filter;
        for i in 0..2 {
            if let Some(err) = self.serial[i].poll(&mut self.packets, &mut self.logger, nf) {
                self.status_msg = err;
            }
        }
    }

    // --- BLE ---

    fn ensure_ble_handle(&mut self) {
        if self.ble_handle.is_none() {
            match BleHandle::new() {
                Ok(h) => self.ble_handle = Some(h),
                Err(e) => self.status_msg = format!("BLE init error: {}", e),
            }
        }
    }

    fn ble_start_scan(&mut self) {
        self.ensure_ble_handle();
        if let Some(ref handle) = self.ble_handle {
            self.ble_devices.clear();
            self.ble_selected_device = None;
            if let Err(e) = handle.send_cmd(BleCmd::StartScan) {
                self.status_msg = format!("BLE scan error: {}", e);
            } else {
                self.ble_scanning = true;
                self.status_msg = "Scanning...".to_string();
            }
        }
    }

    fn ble_stop_scan(&mut self) {
        if let Some(ref handle) = self.ble_handle {
            let _ = handle.send_cmd(BleCmd::StopScan);
        }
        self.ble_scanning = false;
    }

    fn ble_connect(&mut self) {
        if let Some(ref address) = self.ble_selected_device {
            if let Some(ref handle) = self.ble_handle {
                self.status_msg = "Connecting...".to_string();
                // Reset parser with current decoder type
                if let Ok(delim) = parser::parse_delimiter_input(&self.delimiter_input) {
                    self.parser = StreamParser::new(delim);
                } else {
                    self.parser = StreamParser::new(vec![0xB5, 0x62]);
                }
                self.parser.set_decoder_type(self.ble_decoder_type);
                let addr = address.clone();
                if let Err(e) = handle.send_cmd(BleCmd::Connect(addr)) {
                    self.status_msg = format!("BLE connect error: {}", e);
                }
                self.start_logger();
            }
        }
    }

    fn ble_disconnect(&mut self) {
        if let Some(ref handle) = self.ble_handle {
            let _ = handle.send_cmd(BleCmd::Disconnect);
        }
        self.ble_connected = false;
        self.ble_chars.clear();
        self.ble_notify_char = None;
        self.ble_write_char = None;
        self.ble_notify_idx = 0;
        self.ble_write_idx = 0;
        self.status_msg = "Disconnected".to_string();
        // Flush remaining parser data
        if let Some(mut pkt) = self.parser.flush() {
            pkt.source = Some("BLE".to_string());
            if let Some(ref mut logger) = self.logger {
                logger.log_packet(&pkt);
                logger.flush();
            }
            self.packets.push(pkt);
        }
    }

    fn poll_ble_events(&mut self) {
        if let Some(ref handle) = self.ble_handle {
            while let Ok(msg) = handle.rx.try_recv() {
                match msg {
                    BleMsg::ScanResult(mut devices) => {
                        // Sort: named devices first, then by RSSI descending
                        devices.sort_by(|a, b| {
                            let a_named = !a.name.is_empty();
                            let b_named = !b.name.is_empty();
                            b_named
                                .cmp(&a_named)
                                .then_with(|| {
                                    let a_rssi = a.rssi.unwrap_or(i16::MIN);
                                    let b_rssi = b.rssi.unwrap_or(i16::MIN);
                                    b_rssi.cmp(&a_rssi)
                                })
                        });
                        self.ble_devices = devices;
                    }
                    BleMsg::Connected(name) => {
                        self.ble_connected = true;
                        self.ble_scanning = false;
                        self.status_msg = format!("BLE Connected: {}", name);
                    }
                    BleMsg::Disconnected => {
                        // Flush parser buffer as partial packet
                        if let Some(mut pkt) = self.parser.flush() {
                            pkt.source = Some("BLE".to_string());
                            if let Some(ref mut logger) = self.logger {
                                logger.log_packet(&pkt);
                                logger.flush();
                            }
                            self.packets.push(pkt);
                        }
                        self.ble_connected = false;
                        self.ble_chars.clear();
                        self.ble_notify_char = None;
                        self.ble_write_char = None;
                        self.ble_notify_idx = 0;
                        self.ble_write_idx = 0;
                        self.status_msg = "BLE Disconnected".to_string();
                    }
                    BleMsg::Services(chars) => {
                        // Auto-detect NUS
                        let mut auto_notify = None;
                        let mut auto_write = None;
                        for (i, c) in chars.iter().enumerate() {
                            if c.is_nus_tx && c.supports_notify() {
                                auto_notify = Some((i, c.service_uuid, c.char_uuid));
                            }
                            if c.is_nus_rx && c.supports_write() {
                                auto_write = Some((i, c.service_uuid, c.char_uuid));
                            }
                        }
                        self.ble_chars = chars;

                        // Auto-subscribe to NUS TX (notify)
                        if let Some((idx, svc, chr)) = auto_notify {
                            self.ble_notify_char = Some((svc, chr));
                            self.ble_notify_idx = idx;
                            if let Some(ref h) = self.ble_handle {
                                let _ = h.send_cmd(BleCmd::Subscribe(svc, chr));
                            }
                        }
                        if let Some((idx, svc, chr)) = auto_write {
                            self.ble_write_char = Some((svc, chr));
                            self.ble_write_idx = idx;
                        }

                        if auto_notify.is_some() || auto_write.is_some() {
                            self.status_msg =
                                format!("{} | NUS detected", self.status_msg);
                        }
                    }
                    BleMsg::Data(data) => {
                        let mut pkts = self.parser.feed(&data);
                        for pkt in &mut pkts {
                            pkt.source = Some("BLE".to_string());
                        }
                        for pkt in &pkts {
                            if let Some(ref mut logger) = self.logger {
                                logger.log_packet(pkt);
                            }
                        }
                        if !pkts.is_empty() {
                            if let Some(ref mut logger) = self.logger {
                                logger.flush();
                            }
                        }
                        self.packets.extend(pkts);
                    }
                    BleMsg::Error(e) => {
                        self.status_msg = format!("BLE: {}", e);
                    }
                }
            }
        }
    }

    fn poll_transport(&mut self) {
        self.poll_ble_events();
        self.poll_serial(); // Always poll serial, even in BLE mode

        // Flush stale parser buffers (BLE only — serial uses Gap detection)
        let stale_timeout = std::time::Duration::from_millis(500);
        if self.ble_connected {
            if let Some(mut pkt) = self.parser.flush_stale(stale_timeout) {
                pkt.source = Some("BLE".to_string());
                if let Some(ref mut logger) = self.logger {
                    logger.log_packet(&pkt);
                    logger.flush();
                }
                self.packets.push(pkt);
            }
        }

        // Check logger errors
        if let Some(ref mut logger) = self.logger {
            if let Some(err) = logger.last_error.take() {
                self.status_msg = err;
            }
        }

        // Cap packets
        if self.packets.len() > self.max_packets {
            let drain = self.packets.len() - self.max_packets;
            self.packets.drain(..drain);
        }
    }

    // --- Send ---

    fn send_hex(&mut self) {
        let input = self.send_input.trim().to_string();
        if input.is_empty() {
            return;
        }

        // Save to history (avoid duplicating last entry)
        if self.send_history.last().map_or(true, |last| last != &input) {
            self.send_history.push(input.clone());
        }

        match parser::parse_hex_string(&input) {
            Ok(mut bytes) => {
                // Append line ending
                bytes.extend_from_slice(self.line_ending.bytes());

                match self.transport_mode {
                    TransportMode::Serial => {
                        let target = self.send_target;
                        if let Some(ref mut handle) = self.serial[target].connection {
                            match handle.send(&bytes) {
                                Ok(()) => {
                                    let source = short_port_name(
                                        &self.serial[target].selected_port,
                                    );
                                    let pkt = Packet::new(
                                        self.serial[target].parser.elapsed(),
                                        Direction::Tx,
                                        bytes,
                                        None,
                                        Some(source),
                                    );
                                    if let Some(ref mut logger) = self.logger {
                                        logger.log_packet(&pkt);
                                        logger.flush();
                                    }
                                    self.packets.push(pkt);
                                }
                                Err(e) => {
                                    self.status_msg = format!("Send error: {}", e);
                                }
                            }
                        }
                    }
                    TransportMode::Ble => {
                        if let (Some(ref handle), Some((svc, chr))) =
                            (&self.ble_handle, self.ble_write_char)
                        {
                            match handle.send(svc, chr, &bytes) {
                                Ok(()) => {
                                    let pkt = Packet::new(
                                        self.parser.elapsed(),
                                        Direction::Tx,
                                        bytes,
                                        None,
                                        Some("BLE".to_string()),
                                    );
                                    if let Some(ref mut logger) = self.logger {
                                        logger.log_packet(&pkt);
                                        logger.flush();
                                    }
                                    self.packets.push(pkt);
                                }
                                Err(e) => {
                                    self.status_msg = format!("BLE send error: {}", e);
                                }
                            }
                        } else {
                            self.status_msg =
                                "No write characteristic selected".to_string();
                        }
                    }
                }
            }
            Err(e) => {
                self.status_msg = format!("Invalid hex: {}", e);
            }
        }
    }

    // --- UI ---

    fn draw_toolbar(&mut self, ui: &mut egui::Ui) {
        // Row 1: Transport mode + Delimiter + Filter + Log
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut self.transport_mode, TransportMode::Serial, "Serial");
            ui.selectable_value(&mut self.transport_mode, TransportMode::Ble, "BLE");
            ui.separator();

            ui.label("Delimiter:");
            let resp = ui.add(
                egui::TextEdit::singleline(&mut self.delimiter_input)
                    .desired_width(80.0)
                    .hint_text("B5 62 / \\n / \"$\""),
            );
            if resp.changed() {
                if !self.delimiter_input.contains('\\') && !self.delimiter_input.contains('"') {
                    self.delimiter_input = format_hex_input(&self.delimiter_input);
                }
                if let Ok(delim) = parser::parse_delimiter_input(&self.delimiter_input) {
                    self.parser.set_delimiter(delim.clone());
                    for conn in &mut self.serial {
                        conn.parser.set_delimiter(delim.clone());
                    }
                }
            }

            ui.separator();

            ui.label("Filter:");
            let filter_resp = ui.add(
                egui::TextEdit::singleline(&mut self.filter_input)
                    .desired_width(120.0)
                    .hint_text("e.g. B5 62 06 41"),
            );
            if filter_resp.changed() {
                self.filter_bytes =
                    parser::parse_hex_string(&self.filter_input).unwrap_or_default();
            }
            ui.checkbox(&mut self.noise_filter, "Noise")
                .on_hover_text("Filter out noise packets (repetitive data from floating lines)");

            ui.separator();

            let was_enabled = self.log_enabled;
            ui.checkbox(&mut self.log_enabled, "Log");

            let fmt_label = match self.log_format {
                LogFormat::Hex => "HEX",
                LogFormat::Ascii => "ASCII",
                LogFormat::HexAscii => "HEX+ASCII",
            };
            egui::ComboBox::from_id_salt("log_fmt_combo")
                .selected_text(fmt_label)
                .width(70.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.log_format, LogFormat::Hex, "HEX");
                    ui.selectable_value(&mut self.log_format, LogFormat::Ascii, "ASCII");
                    ui.selectable_value(&mut self.log_format, LogFormat::HexAscii, "HEX+ASCII");
                });

            if self.log_enabled && !was_enabled && self.is_connected() {
                self.start_logger();
            }
            if !self.log_enabled && was_enabled {
                self.logger = None;
            }

            ui.add(
                egui::TextEdit::singleline(&mut self.log_path)
                    .desired_width(150.0)
                    .hint_text("uart_log.txt"),
            );

            if ui.button("...").on_hover_text("Choose log file").clicked()
                && self.file_dialog_rx.is_none()
            {
                let (tx, rx) = std::sync::mpsc::channel();
                self.file_dialog_rx = Some(rx);
                std::thread::spawn(move || {
                    let file = rfd::FileDialog::new()
                        .set_title("Save log file")
                        .set_file_name("uart_log.txt")
                        .add_filter("Log files", &["txt", "log"])
                        .save_file();
                    if let Some(path) = file {
                        let _ = tx.send(path.to_string_lossy().to_string());
                    }
                });
            }
        });

        // Row 2: UART1 or BLE toolbar
        ui.horizontal_wrapped(|ui| {
            match self.transport_mode {
                TransportMode::Serial => self.draw_serial_toolbar(ui, 0),
                TransportMode::Ble => self.draw_ble_toolbar(ui),
            }
        });

        // Row 3: UART2 (Serial mode only)
        if self.transport_mode == TransportMode::Serial {
            ui.horizontal_wrapped(|ui| {
                self.draw_serial_toolbar(ui, 1);
            });
        }
    }

    fn draw_serial_toolbar(&mut self, ui: &mut egui::Ui, idx: usize) {
        let salt = format!("s{}", idx);

        // Label
        ui.label(egui::RichText::new(&self.serial[idx].label).strong());

        // Port combo
        let port_text = if self.serial[idx].selected_port.is_empty() {
            "---".to_string()
        } else {
            self.serial[idx].selected_port.clone()
        };

        let ports = self.available_ports.clone();
        egui::ComboBox::from_id_salt(format!("{}_port", salt))
            .selected_text(&port_text)
            .show_ui(ui, |ui| {
                for port in &ports {
                    ui.selectable_value(
                        &mut self.serial[idx].selected_port,
                        port.clone(),
                        port,
                    );
                }
            });

        if ui.button("Scan").on_hover_text("Scan ports").clicked() {
            self.scan_ports();
        }

        ui.separator();
        ui.label("Baud:");

        egui::ComboBox::from_id_salt(format!("{}_baud", salt))
            .selected_text(self.serial[idx].selected_baud.to_string())
            .show_ui(ui, |ui| {
                for &rate in BAUD_RATES {
                    if ui
                        .selectable_value(
                            &mut self.serial[idx].selected_baud,
                            rate,
                            rate.to_string(),
                        )
                        .clicked()
                    {
                        self.serial[idx].baud_input = rate.to_string();
                    }
                }
            });

        let baud_resp = ui.add(
            egui::TextEdit::singleline(&mut self.serial[idx].baud_input)
                .desired_width(70.0)
                .hint_text("custom"),
        );
        if baud_resp.changed() {
            if let Ok(val) = self.serial[idx].baud_input.trim().parse::<u32>() {
                self.serial[idx].selected_baud = val;
            }
        }

        ui.separator();

        // Data bits
        let db_label = match self.serial[idx].data_bits {
            serialport::DataBits::Five => "5",
            serialport::DataBits::Six => "6",
            serialport::DataBits::Seven => "7",
            serialport::DataBits::Eight => "8",
        };
        egui::ComboBox::from_id_salt(format!("{}_db", salt))
            .selected_text(db_label)
            .width(32.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.serial[idx].data_bits,
                    serialport::DataBits::Five,
                    "5",
                );
                ui.selectable_value(
                    &mut self.serial[idx].data_bits,
                    serialport::DataBits::Six,
                    "6",
                );
                ui.selectable_value(
                    &mut self.serial[idx].data_bits,
                    serialport::DataBits::Seven,
                    "7",
                );
                ui.selectable_value(
                    &mut self.serial[idx].data_bits,
                    serialport::DataBits::Eight,
                    "8",
                );
            })
            .response
            .on_hover_text("Data bits");

        // Parity
        let par_label = match self.serial[idx].parity {
            serialport::Parity::None => "N",
            serialport::Parity::Odd => "O",
            serialport::Parity::Even => "E",
        };
        egui::ComboBox::from_id_salt(format!("{}_par", salt))
            .selected_text(par_label)
            .width(32.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.serial[idx].parity,
                    serialport::Parity::None,
                    "None",
                );
                ui.selectable_value(
                    &mut self.serial[idx].parity,
                    serialport::Parity::Odd,
                    "Odd",
                );
                ui.selectable_value(
                    &mut self.serial[idx].parity,
                    serialport::Parity::Even,
                    "Even",
                );
            })
            .response
            .on_hover_text("Parity");

        // Stop bits
        let sb_label = match self.serial[idx].stop_bits {
            serialport::StopBits::One => "1",
            serialport::StopBits::Two => "2",
        };
        egui::ComboBox::from_id_salt(format!("{}_sb", salt))
            .selected_text(sb_label)
            .width(32.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.serial[idx].stop_bits,
                    serialport::StopBits::One,
                    "1",
                );
                ui.selectable_value(
                    &mut self.serial[idx].stop_bits,
                    serialport::StopBits::Two,
                    "2",
                );
            })
            .response
            .on_hover_text("Stop bits");

        // Flow control
        let fc_label = match self.serial[idx].flow_control {
            serialport::FlowControl::None => "Off",
            serialport::FlowControl::Software => "XON",
            serialport::FlowControl::Hardware => "RTS",
        };
        egui::ComboBox::from_id_salt(format!("{}_fc", salt))
            .selected_text(fc_label)
            .width(40.0)
            .show_ui(ui, |ui| {
                ui.selectable_value(
                    &mut self.serial[idx].flow_control,
                    serialport::FlowControl::None,
                    "Off",
                );
                ui.selectable_value(
                    &mut self.serial[idx].flow_control,
                    serialport::FlowControl::Software,
                    "XON/XOFF",
                );
                ui.selectable_value(
                    &mut self.serial[idx].flow_control,
                    serialport::FlowControl::Hardware,
                    "RTS/CTS",
                );
            })
            .response
            .on_hover_text("Flow control");

        ui.separator();

        // Connect / Disconnect
        if self.serial[idx].is_connected() {
            if ui
                .button(
                    egui::RichText::new("Disconnect")
                        .color(egui::Color32::from_rgb(255, 100, 100)),
                )
                .clicked()
            {
                self.serial_disconnect(idx);
            }
        } else {
            let can_connect = !self.serial[idx].selected_port.is_empty();
            if ui
                .add_enabled(
                    can_connect,
                    egui::Button::new(
                        egui::RichText::new("Connect")
                            .color(egui::Color32::from_rgb(100, 255, 100)),
                    ),
                )
                .clicked()
            {
                self.serial_connect(idx);
            }
        }

        ui.separator();

        // DTR / RTS toggles
        if ui
            .add_enabled(
                self.serial[idx].is_connected(),
                egui::Button::new("DTR").selected(self.serial[idx].dtr_state),
            )
            .on_hover_text("Data Terminal Ready")
            .clicked()
        {
            self.serial[idx].dtr_state = !self.serial[idx].dtr_state;
            if let Some(ref mut handle) = self.serial[idx].connection {
                if let Err(e) = handle.set_dtr(self.serial[idx].dtr_state) {
                    self.status_msg = e;
                }
            }
        }
        if ui
            .add_enabled(
                self.serial[idx].is_connected(),
                egui::Button::new("RTS").selected(self.serial[idx].rts_state),
            )
            .on_hover_text("Request To Send")
            .clicked()
        {
            self.serial[idx].rts_state = !self.serial[idx].rts_state;
            if let Some(ref mut handle) = self.serial[idx].connection {
                if let Err(e) = handle.set_rts(self.serial[idx].rts_state) {
                    self.status_msg = e;
                }
            }
        }

        ui.separator();

        // Decoder type
        let mut new_dt = self.serial[idx].decoder_type;
        egui::ComboBox::from_id_salt(format!("{}_dec", salt))
            .selected_text(self.serial[idx].decoder_type.label())
            .width(70.0)
            .show_ui(ui, |ui| {
                for dt in DecoderType::ALL {
                    ui.selectable_value(&mut new_dt, dt, dt.label());
                }
            });
        if new_dt != self.serial[idx].decoder_type {
            self.serial[idx].decoder_type = new_dt;
            self.serial[idx].parser.set_decoder_type(new_dt);
        }
    }

    fn draw_ble_toolbar(&mut self, ui: &mut egui::Ui) {
        // Scan button
        if self.ble_scanning {
            if ui
                .button(
                    egui::RichText::new("Stop scan")
                        .color(egui::Color32::from_rgb(255, 200, 60)),
                )
                .clicked()
            {
                self.ble_stop_scan();
            }
        } else if ui
            .button(
                egui::RichText::new("Scan").color(egui::Color32::from_rgb(100, 200, 255)),
            )
            .clicked()
        {
            self.ble_start_scan();
        }

        ui.separator();

        // Device combo
        let device_text = if let Some(ref addr) = self.ble_selected_device {
            self.ble_devices
                .iter()
                .find(|d| d.address == *addr)
                .map(|dev| {
                    let rssi_str = dev
                        .rssi
                        .map(|r| format!(" ({}dBm)", r))
                        .unwrap_or_default();
                    if dev.name.is_empty() {
                        format!("{}{}", dev.address, rssi_str)
                    } else {
                        format!("{}{}", dev.name, rssi_str)
                    }
                })
                .unwrap_or_else(|| "---".to_string())
        } else {
            "---".to_string()
        };

        egui::ComboBox::from_id_salt("ble_dev_combo")
            .selected_text(&device_text)
            .width(280.0)
            .height(400.0)
            .show_ui(ui, |ui| {
                let devices = self.ble_devices.clone();
                for dev in &devices {
                    let rssi_str = dev
                        .rssi
                        .map(|r| format!(" ({}dBm)", r))
                        .unwrap_or_default();
                    let label = if dev.name.is_empty() {
                        format!("{}{}", dev.address, rssi_str)
                    } else {
                        format!("{} [{}]{}", dev.name, dev.address, rssi_str)
                    };
                    ui.selectable_value(
                        &mut self.ble_selected_device,
                        Some(dev.address.clone()),
                        &label,
                    );
                }
            });

        ui.separator();

        // Connect/Disconnect
        if self.ble_connected {
            if ui
                .button(
                    egui::RichText::new("Disconnect")
                        .color(egui::Color32::from_rgb(255, 100, 100)),
                )
                .clicked()
            {
                self.ble_disconnect();
            }
        } else {
            let can_connect = self.ble_selected_device.is_some() && !self.ble_connected;
            if ui
                .add_enabled(
                    can_connect,
                    egui::Button::new(
                        egui::RichText::new("Connect")
                            .color(egui::Color32::from_rgb(100, 255, 100)),
                    ),
                )
                .clicked()
            {
                self.ble_connect();
            }
        }

        ui.separator();

        // BLE Decoder type
        let mut new_ble_dt = self.ble_decoder_type;
        egui::ComboBox::from_id_salt("ble_dec_combo")
            .selected_text(self.ble_decoder_type.label())
            .width(70.0)
            .show_ui(ui, |ui| {
                for dt in DecoderType::ALL {
                    ui.selectable_value(&mut new_ble_dt, dt, dt.label());
                }
            });
        if new_ble_dt != self.ble_decoder_type {
            self.ble_decoder_type = new_ble_dt;
            self.parser.set_decoder_type(new_ble_dt);
        }

        // Characteristic selectors (only when connected)
        if self.ble_connected && !self.ble_chars.is_empty() {
            ui.separator();

            // RX (notify) characteristic
            let notify_chars: Vec<(usize, String)> = self
                .ble_chars
                .iter()
                .enumerate()
                .filter(|(_, c)| c.supports_notify())
                .map(|(i, c)| (i, c.short_label()))
                .collect();

            // TX (write) characteristic
            let write_chars: Vec<(usize, String)> = self
                .ble_chars
                .iter()
                .enumerate()
                .filter(|(_, c)| c.supports_write())
                .map(|(i, c)| (i, c.short_label()))
                .collect();

            // If NUS auto-detected and only one choice each — just show label
            let nus_auto = self.ble_notify_char.is_some()
                && self.ble_write_char.is_some()
                && notify_chars.len() == 1
                && write_chars.len() == 1;

            if nus_auto {
                ui.label(
                    egui::RichText::new("NUS")
                        .color(egui::Color32::from_rgb(100, 200, 100)),
                );
            } else {
                // Show comboboxes when characteristics are available
                if !notify_chars.is_empty() {
                    ui.label("RX:");
                    let current_label = notify_chars
                        .iter()
                        .find(|(i, _)| *i == self.ble_notify_idx)
                        .map(|(_, l)| l.clone())
                        .unwrap_or_else(|| "---".to_string());

                    let mut new_notify_idx = self.ble_notify_idx;
                    egui::ComboBox::from_id_salt("ble_rx_combo")
                        .selected_text(&current_label)
                        .width(100.0)
                        .show_ui(ui, |ui| {
                            for (i, label) in &notify_chars {
                                ui.selectable_value(&mut new_notify_idx, *i, label);
                            }
                        });

                    if new_notify_idx != self.ble_notify_idx {
                        self.ble_notify_idx = new_notify_idx;
                        if let Some(c) = self.ble_chars.get(new_notify_idx) {
                            self.ble_notify_char = Some((c.service_uuid, c.char_uuid));
                            if let Some(ref h) = self.ble_handle {
                                let _ =
                                    h.send_cmd(BleCmd::Subscribe(c.service_uuid, c.char_uuid));
                            }
                        }
                    }
                }

                if !write_chars.is_empty() {
                    ui.label("TX:");
                    let current_label = write_chars
                        .iter()
                        .find(|(i, _)| *i == self.ble_write_idx)
                        .map(|(_, l)| l.clone())
                        .unwrap_or_else(|| "---".to_string());

                    let mut new_write_idx = self.ble_write_idx;
                    egui::ComboBox::from_id_salt("ble_tx_combo")
                        .selected_text(&current_label)
                        .width(100.0)
                        .show_ui(ui, |ui| {
                            for (i, label) in &write_chars {
                                ui.selectable_value(&mut new_write_idx, *i, label);
                            }
                        });

                    if new_write_idx != self.ble_write_idx {
                        self.ble_write_idx = new_write_idx;
                        if let Some(c) = self.ble_chars.get(new_write_idx) {
                            self.ble_write_char = Some((c.service_uuid, c.char_uuid));
                        }
                    }
                }
            }
        }
    }

    fn draw_packet_view(&self, ui: &mut egui::Ui) {
        let rx_color = egui::Color32::from_rgb(80, 220, 120); // green
        let tx_color = egui::Color32::from_rgb(255, 200, 60); // yellow
        let ts_color = egui::Color32::from_rgb(140, 140, 140); // grey
        let label_color = egui::Color32::from_rgb(120, 180, 255); // light blue
        let ascii_color = egui::Color32::from_rgb(180, 160, 200); // muted purple
        let source_color = egui::Color32::from_rgb(100, 200, 200); // teal

        let mono = egui::FontId::monospace(13.0);

        // Pre-filter packets
        let filtered: Vec<&Packet> = self
            .packets
            .iter()
            .filter(|pkt| {
                if self.noise_filter && pkt.noise {
                    return false;
                }
                self.filter_bytes.is_empty()
                    || pkt.direction == Direction::Tx
                    || pkt.data.starts_with(&self.filter_bytes)
            })
            .collect();

        let row_height = 18.0;
        let output = egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .show_rows(ui, row_height, filtered.len(), |ui, range| {
                for pkt in &filtered[range] {
                    let (dir_color, arrow) = match pkt.direction {
                        Direction::Rx => (rx_color, "< "),
                        Direction::Tx => (tx_color, "> "),
                    };

                    let mut job = egui::text::LayoutJob::default();

                    let fmt = |color: egui::Color32| egui::TextFormat {
                        color,
                        font_id: mono.clone(),
                        ..Default::default()
                    };

                    // Source (port name)
                    if let Some(ref source) = pkt.source {
                        job.append(&format!("{} ", source), 0.0, fmt(source_color));
                    }

                    // Arrow
                    job.append(arrow, 0.0, fmt(dir_color));

                    // Timestamp
                    let ts = format!("{:10.3}s  ", pkt.timestamp);
                    job.append(&ts, 0.0, fmt(ts_color));

                    // Data columns based on display format
                    match self.display_format {
                        DataFormat::Hex => {
                            job.append(pkt.hex_string(), 0.0, fmt(dir_color));
                        }
                        DataFormat::Ascii => {
                            job.append(pkt.ascii_string(), 0.0, fmt(dir_color));
                        }
                        DataFormat::HexAscii => {
                            job.append(pkt.hex_string(), 0.0, fmt(dir_color));
                            job.append(
                                &format!("  |{}|", pkt.ascii_string()),
                                0.0,
                                fmt(ascii_color),
                            );
                        }
                    }

                    // Label
                    if let Some(ref label) = pkt.label {
                        job.append(&format!("  {}", label), 0.0, fmt(label_color));
                    }

                    let response = ui.label(job);
                    response.context_menu(|ui| {
                        if ui.button("Copy HEX").clicked() {
                            ui.ctx().copy_text(pkt.hex_string().to_string());
                            ui.close();
                        }
                        if ui.button("Copy ASCII").clicked() {
                            ui.ctx().copy_text(pkt.ascii_string().to_string());
                            ui.close();
                        }
                    });
                }
            });

        // Smooth animated auto-scroll instead of instant stick_to_bottom
        if self.auto_scroll && !filtered.is_empty() {
            let max_offset = (output.content_size.y - output.inner_rect.height()).max(0.0);
            let smoothed = ui.ctx().animate_value_with_time(
                output.id.with("auto_scroll_y"),
                max_offset,
                0.15,
            );
            // Clamp: never exceed max_offset (prevents blank space on window resize)
            let mut state = output.state;
            state.offset.y = smoothed.min(max_offset);
            state.store(ui.ctx(), output.id);
        }
    }

    fn draw_send_bar(&mut self, ui: &mut egui::Ui) {
        // Right-side buttons width: Target(~90) + LineEnding(~75) + Send(~45) + Clear(~50) + Auto-scroll(~85) + Display(~110) + gaps(~65)
        let target_width = if self.transport_mode == TransportMode::Serial {
            90.0
        } else {
            0.0
        };
        let right_width = 430.0 + target_width;

        ui.horizontal(|ui| {
            ui.label("HEX:");

            let input_width = (ui.available_width() - right_width).max(80.0);
            let resp = ui.add(
                egui::TextEdit::singleline(&mut self.send_input)
                    .desired_width(input_width)
                    .hint_text("B5 62 06 01 ...")
                    .font(egui::TextStyle::Monospace),
            );

            // History navigation (Up/Down)
            if resp.has_focus() {
                if ui.input(|i| i.key_pressed(egui::Key::ArrowUp)) && !self.send_history.is_empty()
                {
                    let idx = match self.send_history_idx {
                        Some(i) if i > 0 => i - 1,
                        Some(i) => i,
                        None => self.send_history.len() - 1,
                    };
                    self.send_history_idx = Some(idx);
                    self.send_input = self.send_history[idx].clone();
                }
                if ui.input(|i| i.key_pressed(egui::Key::ArrowDown)) {
                    if let Some(idx) = self.send_history_idx {
                        if idx + 1 < self.send_history.len() {
                            self.send_history_idx = Some(idx + 1);
                            self.send_input = self.send_history[idx + 1].clone();
                        } else {
                            self.send_history_idx = None;
                            self.send_input.clear();
                        }
                    }
                }
            }

            // Send on Enter
            if resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                self.send_hex();
                self.send_input.clear();
                self.send_history_idx = None;
            }

            // Target port selector (Serial mode only)
            if self.transport_mode == TransportMode::Serial {
                let target_label = &self.serial[self.send_target].label;
                egui::ComboBox::from_id_salt("send_target_combo")
                    .selected_text(target_label.as_str())
                    .width(55.0)
                    .show_ui(ui, |ui| {
                        for i in 0..2 {
                            let lbl = self.serial[i].label.clone();
                            ui.selectable_value(&mut self.send_target, i, &lbl);
                        }
                    });
            }

            // Line ending
            egui::ComboBox::from_id_salt("le_combo")
                .selected_text(self.line_ending.label())
                .width(45.0)
                .show_ui(ui, |ui| {
                    for le in LineEnding::ALL {
                        ui.selectable_value(&mut self.line_ending, le, le.label());
                    }
                });

            let can_send = self.is_connected();
            if ui
                .add_enabled(
                    can_send,
                    egui::Button::new(
                        egui::RichText::new("Send").color(egui::Color32::from_rgb(255, 200, 60)),
                    ),
                )
                .clicked()
            {
                self.send_hex();
                self.send_input.clear();
                self.send_history_idx = None;
            }

            if ui.button("Clear").clicked() {
                self.packets.clear();
            }

            if ui
                .selectable_label(self.auto_scroll, "Auto-scroll")
                .clicked()
            {
                self.auto_scroll = !self.auto_scroll;
            }

            ui.separator();

            // Display format
            let disp_label = match self.display_format {
                DataFormat::Hex => "HEX",
                DataFormat::Ascii => "ASCII",
                DataFormat::HexAscii => "HEX+ASCII",
            };
            egui::ComboBox::from_id_salt("disp_fmt_combo")
                .selected_text(disp_label)
                .width(70.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.display_format, DataFormat::Hex, "HEX");
                    ui.selectable_value(&mut self.display_format, DataFormat::Ascii, "ASCII");
                    ui.selectable_value(
                        &mut self.display_format,
                        DataFormat::HexAscii,
                        "HEX+ASCII",
                    );
                });
        });
    }
}

impl eframe::App for UartTermApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Poll transport data
        self.poll_transport();

        // Poll file dialog result
        if let Some(ref rx) = self.file_dialog_rx {
            match rx.try_recv() {
                Ok(path) => {
                    self.log_path = path;
                    self.file_dialog_rx = None;
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.file_dialog_rx = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
            }
        }

        // Request repaint while connected, scanning, connecting, or dialog pending
        let any_serial = self.serial[0].is_connected() || self.serial[1].is_connected();
        let ble_active = self.ble_connected || self.ble_scanning || self.ble_handle.is_some();
        if any_serial || ble_active || self.file_dialog_rx.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            self.draw_toolbar(ui);
            ui.separator();
            // Status bar
            ui.horizontal(|ui| {
                let color = if self.is_connected() {
                    egui::Color32::from_rgb(80, 220, 120)
                } else {
                    egui::Color32::from_rgb(180, 180, 180)
                };
                ui.label(egui::RichText::new(&self.status_msg).color(color).small());
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        egui::RichText::new(format!("{} packets", self.packets.len()))
                            .small()
                            .color(egui::Color32::from_rgb(140, 140, 140)),
                    );
                });
            });
        });

        egui::TopBottomPanel::bottom("send_bar").show(ctx, |ui| {
            ui.add_space(4.0);
            self.draw_send_bar(ui);
            ui.add_space(4.0);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.draw_packet_view(ui);
        });
    }
}
