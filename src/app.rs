use eframe::egui;

use crate::logger::Logger;
use crate::parser::{self, Direction, Packet, StreamParser};
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

pub struct UartTermApp {
    // Connection
    available_ports: Vec<String>,
    selected_port: String,
    selected_baud: u32,
    baud_input: String,
    data_bits: serialport::DataBits,
    parity: serialport::Parity,
    stop_bits: serialport::StopBits,
    flow_control: serialport::FlowControl,
    connection: Option<SerialHandle>,
    connected: bool,
    status_msg: String,

    // Parser
    parser: StreamParser,
    delimiter_input: String,

    // Display
    packets: Vec<Packet>,
    auto_scroll: bool,
    max_packets: usize,
    filter_input: String,
    filter_bytes: Vec<u8>,

    // Send
    send_input: String,
    line_ending: LineEnding,
    send_history: Vec<String>,
    send_history_idx: Option<usize>,

    // Control lines
    dtr_state: bool,
    rts_state: bool,

    // Logging
    log_enabled: bool,
    log_path: String,
    logger: Option<Logger>,
    file_dialog_rx: Option<std::sync::mpsc::Receiver<String>>,
}

impl UartTermApp {
    pub fn new() -> Self {
        let ports = serial::list_ports();
        let selected = ports.first().cloned().unwrap_or_default();

        Self {
            available_ports: ports,
            selected_port: selected,
            selected_baud: 921600,
            baud_input: "921600".to_string(),
            data_bits: serialport::DataBits::Eight,
            parity: serialport::Parity::None,
            stop_bits: serialport::StopBits::One,
            flow_control: serialport::FlowControl::None,
            connection: None,
            connected: false,
            status_msg: "Disconnected".to_string(),

            parser: StreamParser::new(vec![0xB5, 0x62]),
            delimiter_input: "B5 62".to_string(),

            packets: Vec::new(),
            auto_scroll: true,
            max_packets: 10000,
            filter_input: String::new(),
            filter_bytes: Vec::new(),

            send_input: String::new(),
            line_ending: LineEnding::None,
            send_history: Vec::new(),
            send_history_idx: None,

            dtr_state: false,
            rts_state: false,

            log_enabled: false,
            log_path: "uart_log.txt".to_string(),
            logger: None,
            file_dialog_rx: None,
        }
    }

    fn connect(&mut self) {
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
                self.connected = true;
                self.status_msg =
                    format!("Connected: {} @ {}", self.selected_port, self.selected_baud);
                // Reset parser
                if let Ok(delim) = parser::parse_hex_string(&self.delimiter_input) {
                    self.parser = StreamParser::new(delim);
                } else {
                    self.parser = StreamParser::new(vec![0xB5, 0x62]);
                }
                // Start logger if enabled
                if self.log_enabled && self.logger.is_none() {
                    match Logger::new(&self.log_path) {
                        Ok(l) => self.logger = Some(l),
                        Err(e) => self.status_msg = format!("Log error: {}", e),
                    }
                }
            }
            Err(e) => {
                self.status_msg = e;
            }
        }
    }

    fn disconnect(&mut self) {
        if let Some(handle) = self.connection.take() {
            handle.disconnect();
        }
        self.connected = false;
        self.status_msg = "Disconnected".to_string();
        // Flush remaining parser data
        if let Some(pkt) = self.parser.flush() {
            if let Some(ref mut logger) = self.logger {
                logger.log_packet(&pkt);
                logger.flush();
            }
            self.packets.push(pkt);
        }
    }

    fn scan_ports(&mut self) {
        self.available_ports = serial::list_ports();
        if !self.available_ports.contains(&self.selected_port) {
            self.selected_port = self.available_ports.first().cloned().unwrap_or_default();
        }
    }

    fn poll_serial(&mut self) {
        if let Some(ref handle) = self.connection {
            let mut new_packets = Vec::new();
            // Drain all available messages
            while let Ok(msg) = handle.rx.try_recv() {
                match msg {
                    SerialMsg::Data(data) => {
                        let pkts = self.parser.feed(&data);
                        new_packets.extend(pkts);
                    }
                    SerialMsg::Error(e) => {
                        self.status_msg = format!("Error: {}", e);
                    }
                    SerialMsg::Disconnected => {
                        self.connected = false;
                        self.status_msg = "Disconnected (device lost)".to_string();
                        self.connection = None;
                        break;
                    }
                }
            }

            // Log and store
            for pkt in &new_packets {
                if let Some(ref mut logger) = self.logger {
                    logger.log_packet(pkt);
                }
            }
            if !new_packets.is_empty() {
                if let Some(ref mut logger) = self.logger {
                    logger.flush();
                }
            }
            self.packets.extend(new_packets);

            // Cap packets
            if self.packets.len() > self.max_packets {
                let drain = self.packets.len() - self.max_packets;
                self.packets.drain(..drain);
            }
        }
    }

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

                if let Some(ref mut handle) = self.connection {
                    match handle.send(&bytes) {
                        Ok(()) => {
                            // Record as TX packet
                            let pkt = Packet {
                                timestamp: self.parser.elapsed(),
                                direction: Direction::Tx,
                                data: bytes,
                                label: None,
                            };
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
            Err(e) => {
                self.status_msg = format!("Invalid hex: {}", e);
            }
        }
    }

    fn draw_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Port:");

            let port_text = if self.selected_port.is_empty() {
                "—".to_string()
            } else {
                self.selected_port.clone()
            };

            egui::ComboBox::from_id_salt("port_combo")
                .selected_text(&port_text)
                .show_ui(ui, |ui| {
                    for port in &self.available_ports.clone() {
                        ui.selectable_value(&mut self.selected_port, port.clone(), port);
                    }
                });

            if ui.button("🔄").on_hover_text("Scan ports").clicked() {
                self.scan_ports();
            }

            ui.separator();
            ui.label("Baud:");

            egui::ComboBox::from_id_salt("baud_combo")
                .selected_text(self.selected_baud.to_string())
                .show_ui(ui, |ui| {
                    for &rate in BAUD_RATES {
                        if ui
                            .selectable_value(&mut self.selected_baud, rate, rate.to_string())
                            .clicked()
                        {
                            self.baud_input = rate.to_string();
                        }
                    }
                });

            let baud_resp = ui.add(
                egui::TextEdit::singleline(&mut self.baud_input)
                    .desired_width(70.0)
                    .hint_text("custom"),
            );
            if baud_resp.changed() {
                if let Ok(val) = self.baud_input.trim().parse::<u32>() {
                    self.selected_baud = val;
                }
            }

            ui.separator();

            // Data bits
            let db_label = match self.data_bits {
                serialport::DataBits::Five => "5",
                serialport::DataBits::Six => "6",
                serialport::DataBits::Seven => "7",
                serialport::DataBits::Eight => "8",
            };
            egui::ComboBox::from_id_salt("db_combo")
                .selected_text(db_label)
                .width(32.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.data_bits, serialport::DataBits::Five, "5");
                    ui.selectable_value(&mut self.data_bits, serialport::DataBits::Six, "6");
                    ui.selectable_value(&mut self.data_bits, serialport::DataBits::Seven, "7");
                    ui.selectable_value(&mut self.data_bits, serialport::DataBits::Eight, "8");
                });

            // Parity
            let par_label = match self.parity {
                serialport::Parity::None => "N",
                serialport::Parity::Odd => "O",
                serialport::Parity::Even => "E",
            };
            egui::ComboBox::from_id_salt("par_combo")
                .selected_text(par_label)
                .width(32.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.parity, serialport::Parity::None, "None");
                    ui.selectable_value(&mut self.parity, serialport::Parity::Odd, "Odd");
                    ui.selectable_value(&mut self.parity, serialport::Parity::Even, "Even");
                });

            // Stop bits
            let sb_label = match self.stop_bits {
                serialport::StopBits::One => "1",
                serialport::StopBits::Two => "2",
            };
            egui::ComboBox::from_id_salt("sb_combo")
                .selected_text(sb_label)
                .width(32.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.stop_bits, serialport::StopBits::One, "1");
                    ui.selectable_value(&mut self.stop_bits, serialport::StopBits::Two, "2");
                });

            // Flow control
            let fc_label = match self.flow_control {
                serialport::FlowControl::None => "Off",
                serialport::FlowControl::Software => "XON",
                serialport::FlowControl::Hardware => "RTS",
            };
            egui::ComboBox::from_id_salt("fc_combo")
                .selected_text(fc_label)
                .width(40.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(
                        &mut self.flow_control,
                        serialport::FlowControl::None,
                        "Off",
                    );
                    ui.selectable_value(
                        &mut self.flow_control,
                        serialport::FlowControl::Software,
                        "XON/XOFF",
                    );
                    ui.selectable_value(
                        &mut self.flow_control,
                        serialport::FlowControl::Hardware,
                        "RTS/CTS",
                    );
                });

            ui.separator();

            if self.connected {
                if ui
                    .button(
                        egui::RichText::new("⏹ Disconnect")
                            .color(egui::Color32::from_rgb(255, 100, 100)),
                    )
                    .clicked()
                {
                    self.disconnect();
                }
            } else if ui
                .button(
                    egui::RichText::new("▶ Connect").color(egui::Color32::from_rgb(100, 255, 100)),
                )
                .clicked()
            {
                self.connect();
            }

            ui.separator();

            // DTR / RTS toggles
            if ui
                .add_enabled(
                    self.connected,
                    egui::Button::new("DTR").selected(self.dtr_state),
                )
                .clicked()
            {
                self.dtr_state = !self.dtr_state;
                if let Some(ref mut handle) = self.connection {
                    if let Err(e) = handle.set_dtr(self.dtr_state) {
                        self.status_msg = e;
                    }
                }
            }
            if ui
                .add_enabled(
                    self.connected,
                    egui::Button::new("RTS").selected(self.rts_state),
                )
                .clicked()
            {
                self.rts_state = !self.rts_state;
                if let Some(ref mut handle) = self.connection {
                    if let Err(e) = handle.set_rts(self.rts_state) {
                        self.status_msg = e;
                    }
                }
            }
        });

        ui.horizontal(|ui| {
            ui.label("Delimiter:");
            let resp = ui.add(
                egui::TextEdit::singleline(&mut self.delimiter_input)
                    .desired_width(80.0)
                    .hint_text("B5 62"),
            );
            if resp.changed() {
                self.delimiter_input = format_hex_input(&self.delimiter_input);
                if let Ok(delim) = parser::parse_hex_string(&self.delimiter_input) {
                    self.parser.set_delimiter(delim);
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
                self.filter_input = format_hex_input(&self.filter_input);
                self.filter_bytes =
                    parser::parse_hex_string(&self.filter_input).unwrap_or_default();
            }

            ui.separator();

            let was_enabled = self.log_enabled;
            ui.checkbox(&mut self.log_enabled, "📝 Log");
            if self.log_enabled && !was_enabled && self.connected {
                match Logger::new(&self.log_path) {
                    Ok(l) => self.logger = Some(l),
                    Err(e) => {
                        self.status_msg = format!("Log error: {}", e);
                        self.log_enabled = false;
                    }
                }
            }
            if !self.log_enabled && was_enabled {
                self.logger = None;
            }

            ui.add(
                egui::TextEdit::singleline(&mut self.log_path)
                    .desired_width(150.0)
                    .hint_text("uart_log.txt"),
            );

            if ui.button("📂").on_hover_text("Choose log file").clicked()
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
    }

    fn draw_packet_view(&self, ui: &mut egui::Ui) {
        let rx_color = egui::Color32::from_rgb(80, 220, 120); // green
        let tx_color = egui::Color32::from_rgb(255, 200, 60); // yellow
        let ts_color = egui::Color32::from_rgb(140, 140, 140); // grey
        let label_color = egui::Color32::from_rgb(120, 180, 255); // light blue
        let ascii_color = egui::Color32::from_rgb(180, 160, 200); // muted purple

        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .stick_to_bottom(self.auto_scroll)
            .show(ui, |ui| {
                ui.style_mut().override_text_style = Some(egui::TextStyle::Monospace);

                for pkt in &self.packets {
                    // Apply display filter (TX always visible)
                    if !self.filter_bytes.is_empty()
                        && pkt.direction == Direction::Rx
                        && !pkt.data.starts_with(&self.filter_bytes)
                    {
                        continue;
                    }

                    let (dir_color, arrow) = match pkt.direction {
                        Direction::Rx => (rx_color, "← "),
                        Direction::Tx => (tx_color, "→ "),
                    };

                    let hex = pkt.hex_string();

                    let mut job = egui::text::LayoutJob::default();

                    // Arrow
                    job.append(
                        arrow,
                        0.0,
                        egui::TextFormat {
                            color: dir_color,
                            font_id: egui::FontId::monospace(13.0),
                            ..Default::default()
                        },
                    );

                    // Timestamp
                    let ts = format!("{:10.3}s  ", pkt.timestamp);
                    job.append(
                        &ts,
                        0.0,
                        egui::TextFormat {
                            color: ts_color,
                            font_id: egui::FontId::monospace(13.0),
                            ..Default::default()
                        },
                    );

                    // Hex data
                    job.append(
                        &hex,
                        0.0,
                        egui::TextFormat {
                            color: dir_color,
                            font_id: egui::FontId::monospace(13.0),
                            ..Default::default()
                        },
                    );

                    // ASCII column
                    job.append(
                        &format!("  |{}|", pkt.ascii_string()),
                        0.0,
                        egui::TextFormat {
                            color: ascii_color,
                            font_id: egui::FontId::monospace(13.0),
                            ..Default::default()
                        },
                    );

                    // Label
                    if let Some(ref label) = pkt.label {
                        job.append(
                            &format!("  {}", label),
                            0.0,
                            egui::TextFormat {
                                color: label_color,
                                font_id: egui::FontId::monospace(13.0),
                                ..Default::default()
                            },
                        );
                    }

                    let response = ui.label(job);
                    response.context_menu(|ui| {
                        if ui.button("Copy HEX").clicked() {
                            ui.ctx().copy_text(pkt.hex_string());
                            ui.close();
                        }
                        if ui.button("Copy ASCII").clicked() {
                            ui.ctx().copy_text(pkt.ascii_string());
                            ui.close();
                        }
                    });
                }
            });
    }

    fn draw_send_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("HEX:");

            let resp = ui.add(
                egui::TextEdit::singleline(&mut self.send_input)
                    .desired_width(ui.available_width() - 340.0)
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

            if ui
                .add_enabled(
                    self.connected,
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
                .selectable_label(self.auto_scroll, "⏬ Auto-scroll")
                .clicked()
            {
                self.auto_scroll = !self.auto_scroll;
            }

            ui.separator();

            egui::ComboBox::from_id_salt("le_combo")
                .selected_text(self.line_ending.label())
                .width(55.0)
                .show_ui(ui, |ui| {
                    for le in LineEnding::ALL {
                        ui.selectable_value(&mut self.line_ending, le, le.label());
                    }
                });
        });
    }
}

impl eframe::App for UartTermApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Poll serial data
        self.poll_serial();

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

        // Request repaint while connected or dialog pending
        if self.connected || self.file_dialog_rx.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            self.draw_toolbar(ui);
            ui.separator();
            // Status bar
            ui.horizontal(|ui| {
                let color = if self.connected {
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
