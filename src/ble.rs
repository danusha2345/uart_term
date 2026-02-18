use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::{FutureExt, StreamExt};
use std::sync::mpsc;
use uuid::Uuid;

/// Nordic UART Service UUIDs
pub const NUS_RX_CHAR: Uuid = Uuid::from_u128(0x6E400002_B5A3_F393_E0A9_E50E24DCCA9E);
pub const NUS_TX_CHAR: Uuid = Uuid::from_u128(0x6E400003_B5A3_F393_E0A9_E50E24DCCA9E);

/// Discovered BLE device info for the UI
#[derive(Clone, Debug)]
pub struct BleDeviceInfo {
    pub name: String,
    pub address: String,
    pub rssi: Option<i16>,
}

/// Characteristic info for the UI
#[derive(Clone, Debug)]
pub struct BleCharInfo {
    pub service_uuid: Uuid,
    pub char_uuid: Uuid,
    pub properties: CharPropFlags,
    pub is_nus_tx: bool, // notify — we read from this
    pub is_nus_rx: bool, // write — we write to this
}

impl BleCharInfo {
    pub fn short_label(&self) -> String {
        let uuid_str = self.char_uuid.to_string();
        let short = &uuid_str[4..8];
        let mut flags = String::new();
        if self.properties.contains(CharPropFlags::NOTIFY) {
            flags.push('N');
        }
        if self.properties.contains(CharPropFlags::WRITE) {
            flags.push('W');
        }
        if self.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE) {
            flags.push('w');
        }
        if self.properties.contains(CharPropFlags::READ) {
            flags.push('R');
        }
        format!("{}({})", short, flags)
    }

    pub fn supports_notify(&self) -> bool {
        self.properties.contains(CharPropFlags::NOTIFY)
    }

    pub fn supports_write(&self) -> bool {
        self.properties.contains(CharPropFlags::WRITE)
            || self.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE)
    }

}

/// Commands from UI to BLE thread
pub enum BleCmd {
    StartScan,
    StopScan,
    Connect(usize),
    Disconnect,
    Subscribe(Uuid, Uuid),
    Write(Uuid, Uuid, Vec<u8>),
}

/// Messages from BLE thread to UI
pub enum BleMsg {
    ScanResult(Vec<BleDeviceInfo>),
    Connected(String),
    Disconnected,
    Services(Vec<BleCharInfo>),
    Data(Vec<u8>),
    Error(String),
}

/// Handle for BLE communication (analogous to SerialHandle)
pub struct BleHandle {
    pub rx: mpsc::Receiver<BleMsg>,
    cmd_tx: mpsc::Sender<BleCmd>,
    _thread: std::thread::JoinHandle<()>,
}

impl BleHandle {
    pub fn new() -> Result<Self, String> {
        let (msg_tx, msg_rx) = mpsc::channel();
        let (cmd_tx, cmd_rx) = mpsc::channel();

        let thread = std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("Failed to create tokio runtime");

            rt.block_on(ble_event_loop(msg_tx, cmd_rx));
        });

        Ok(Self {
            rx: msg_rx,
            cmd_tx,
            _thread: thread,
        })
    }

    pub fn send_cmd(&self, cmd: BleCmd) -> Result<(), String> {
        self.cmd_tx
            .send(cmd)
            .map_err(|_| "BLE thread not running".to_string())
    }

    pub fn send(&self, svc: Uuid, chr: Uuid, data: &[u8]) -> Result<(), String> {
        self.send_cmd(BleCmd::Write(svc, chr, data.to_vec()))
    }
}

async fn get_adapter() -> Result<Adapter, String> {
    let manager = Manager::new()
        .await
        .map_err(|e| format!("BLE Manager error: {}", e))?;
    let adapters = manager
        .adapters()
        .await
        .map_err(|e| format!("No BLE adapters: {}", e))?;
    adapters
        .into_iter()
        .next()
        .ok_or_else(|| "No BLE adapter found".to_string())
}

async fn ble_event_loop(tx: mpsc::Sender<BleMsg>, cmd_rx: mpsc::Receiver<BleCmd>) {
    let adapter = match get_adapter().await {
        Ok(a) => a,
        Err(e) => {
            let _ = tx.send(BleMsg::Error(e));
            return;
        }
    };

    let mut peripherals: Vec<Peripheral> = Vec::new();
    let mut connected_peripheral: Option<Peripheral> = None;
    let mut notification_stream: Option<
        std::pin::Pin<Box<dyn futures::Stream<Item = btleplug::api::ValueNotification> + Send>>,
    > = None;
    let mut scanning = false;

    loop {
        // Process commands (non-blocking)
        match cmd_rx.try_recv() {
            Ok(cmd) => match cmd {
                BleCmd::StartScan => {
                    peripherals.clear();
                    if let Err(e) = adapter.start_scan(ScanFilter::default()).await {
                        let _ = tx.send(BleMsg::Error(format!("Scan error: {}", e)));
                    } else {
                        scanning = true;
                    }
                }
                BleCmd::StopScan => {
                    let _ = adapter.stop_scan().await;
                    scanning = false;
                }
                BleCmd::Connect(idx) => {
                    // Stop scan first
                    if scanning {
                        let _ = adapter.stop_scan().await;
                        scanning = false;
                    }

                    if idx >= peripherals.len() {
                        let _ = tx.send(BleMsg::Error("Invalid device index".to_string()));
                        continue;
                    }

                    let peripheral = peripherals[idx].clone();
                    match peripheral.connect().await {
                        Ok(()) => {
                            // Discover services
                            if let Err(e) = peripheral.discover_services().await {
                                let _ = tx.send(BleMsg::Error(format!(
                                    "Service discovery error: {}",
                                    e
                                )));
                                continue;
                            }

                            let name = peripheral
                                .properties()
                                .await
                                .ok()
                                .flatten()
                                .and_then(|p| p.local_name)
                                .unwrap_or_else(|| "Unknown".to_string());

                            let _ = tx.send(BleMsg::Connected(name));

                            // Build characteristic list
                            let chars: Vec<BleCharInfo> = peripheral
                                .services()
                                .iter()
                                .flat_map(|svc| {
                                    svc.characteristics.iter().map(move |chr| BleCharInfo {
                                        service_uuid: svc.uuid,
                                        char_uuid: chr.uuid,
                                        properties: chr.properties,
                                        is_nus_tx: chr.uuid == NUS_TX_CHAR,
                                        is_nus_rx: chr.uuid == NUS_RX_CHAR,
                                    })
                                })
                                .collect();

                            let _ = tx.send(BleMsg::Services(chars));
                            connected_peripheral = Some(peripheral);
                        }
                        Err(e) => {
                            let _ = tx.send(BleMsg::Error(format!("Connect error: {}", e)));
                        }
                    }
                }
                BleCmd::Disconnect => {
                    notification_stream = None;
                    if let Some(ref p) = connected_peripheral {
                        let _ = p.disconnect().await;
                    }
                    connected_peripheral = None;
                    let _ = tx.send(BleMsg::Disconnected);
                }
                BleCmd::Subscribe(svc_uuid, char_uuid) => {
                    if let Some(ref p) = connected_peripheral {
                        let chr = find_characteristic(p, svc_uuid, char_uuid);
                        if let Some(c) = chr {
                            match p.subscribe(&c).await {
                                Ok(()) => {
                                    match p.notifications().await {
                                        Ok(stream) => {
                                            notification_stream = Some(Box::pin(stream));
                                        }
                                        Err(e) => {
                                            let _ = tx.send(BleMsg::Error(format!(
                                                "Notification stream error: {}",
                                                e
                                            )));
                                        }
                                    }
                                }
                                Err(e) => {
                                    let _ =
                                        tx.send(BleMsg::Error(format!("Subscribe error: {}", e)));
                                }
                            }
                        } else {
                            let _ =
                                tx.send(BleMsg::Error("Characteristic not found".to_string()));
                        }
                    }
                }
                BleCmd::Write(svc_uuid, char_uuid, data) => {
                    if let Some(ref p) = connected_peripheral {
                        let chr = find_characteristic(p, svc_uuid, char_uuid);
                        if let Some(c) = chr {
                            let write_type =
                                if c.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE) {
                                    WriteType::WithoutResponse
                                } else {
                                    WriteType::WithResponse
                                };
                            if let Err(e) = p.write(&c, &data, write_type).await {
                                let _ = tx.send(BleMsg::Error(format!("Write error: {}", e)));
                            }
                        } else {
                            let _ =
                                tx.send(BleMsg::Error("Write characteristic not found".to_string()));
                        }
                    }
                }
            },
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => {
                // UI dropped, clean up
                if let Some(ref p) = connected_peripheral {
                    let _ = p.disconnect().await;
                }
                return;
            }
        }

        // Poll scan results
        if scanning {
            if let Ok(discovered) = adapter.peripherals().await {
                let mut devices = Vec::new();
                for p in discovered.iter() {
                    let props = p.properties().await.ok().flatten();
                    let name = props
                        .as_ref()
                        .and_then(|p| p.local_name.clone())
                        .unwrap_or_default();
                    let address = props
                        .as_ref()
                        .map(|p| p.address.to_string())
                        .unwrap_or_default();
                    let rssi = props.as_ref().and_then(|p| p.rssi);
                    devices.push(BleDeviceInfo {
                        name,
                        address,
                        rssi,
                    });
                }
                peripherals = discovered;
                let _ = tx.send(BleMsg::ScanResult(devices));
            }
        }

        // Poll notifications
        if let Some(ref mut stream) = notification_stream {
            loop {
                match stream.next().now_or_never() {
                    Some(Some(notification)) => {
                        let _ = tx.send(BleMsg::Data(notification.value));
                    }
                    Some(None) => {
                        // Stream ended — device probably disconnected
                        notification_stream = None;
                        connected_peripheral = None;
                        let _ = tx.send(BleMsg::Disconnected);
                        break;
                    }
                    None => break, // No data available
                }
            }
        }

        // Check if connected peripheral is still connected
        if let Some(ref p) = connected_peripheral {
            if let Ok(false) = p.is_connected().await {
                notification_stream = None;
                connected_peripheral = None;
                let _ = tx.send(BleMsg::Disconnected);
            }
        }

        tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
    }
}

fn find_characteristic(
    peripheral: &Peripheral,
    svc_uuid: Uuid,
    char_uuid: Uuid,
) -> Option<Characteristic> {
    peripheral
        .services()
        .iter()
        .find(|s| s.uuid == svc_uuid)
        .and_then(|s| s.characteristics.iter().find(|c| c.uuid == char_uuid).cloned())
}
