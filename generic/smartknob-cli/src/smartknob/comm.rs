use cobs::DecodeError;
use cobs::max_encoding_length;
use futures::SinkExt;
use futures::stream::SplitSink;
use futures::stream::SplitStream;
use futures::stream::StreamExt;
use log::warn;
use log::{error, info};
use postcard::{experimental::max_size::MaxSize, from_bytes, to_stdvec_cobs};
use smartknob_core::comm;
use static_cell::StaticCell;
use std::time::Duration;
use thiserror::Error;
use tokio::sync::watch;
use tokio::task;
use tokio::time::error::Elapsed;
use tokio::{
    sync::mpsc::{self, Receiver},
    time::{sleep, timeout},
};
use tokio_serial::{
    SerialPortBuilderExt, SerialPortInfo, SerialPortType, SerialStream, UsbPortInfo,
    available_ports,
};
use tokio_util::bytes::BufMut;
use tokio_util::codec::LinesCodec;
use tokio_util::{
    bytes::BytesMut,
    codec::{Decoder, Encoder, Framed},
};

const JTAG_PID: u16 = 0x1001;
const JTAG_VID: u16 = 0x303a;
const JTAG_MANUFACTURER: &str = "Espressif";
const JTAG_PRODUCT: &str = "USB JTAG/serial debug unit";
const SMARTKNOB_TRANSTION_MAGIC_BYTE: &[u8] = &[35];

/// How many times maximum should a single message be repeated if the smartknob requests a repeat
const NUM_MAX_REPEAT: usize = 3;

#[derive(Debug, Error)]
pub enum CommError {
    #[error("Error during data transfer: {0}")]
    IOError(#[from] std::io::Error),
    #[error("Cobs failed to decode: {0}")]
    CobsDecodeError(#[from] DecodeError),
    #[error("Serialization of message fialed: {0}")]
    SerializationError(#[from] postcard::Error),
    #[error("Failed to get a response message")]
    GettingResponse,
    #[error("Smartknob asked too many times for a re-transmission of the current message")]
    ExceededRetries,
    #[error("The smartknob took too long to respond to the request")]
    ExceededTimeout(Elapsed),
    #[error("Enabling a channel watcher failed: {0}")]
    EnableChannelWatcher(#[from] watch::error::SendError<ActiveCommChannels>),
    #[error("Event channel was already consumed")]
    EventChannelGone,
}

#[derive(Debug, Error)]
pub enum ConnectionError {
    #[error("Failed to list serial ports: {0}")]
    ListingPorts(tokio_serial::Error),
    #[error("Failed to connect to serial port: {0}")]
    Connecting(tokio_serial::Error),
    #[error("Failed to send data to smartknob: {0}")]
    SendingData(#[from] std::io::Error),
    #[error("Failed to transition smartknob to ACM mode")]
    Transitionioning,
    #[error("Failed to discover any connected smartknob device")]
    Discovering,
    #[error("Log channel was already taken perviously")]
    LogChannelGone,
    #[error("Comm cannels were already taken perviously")]
    ComChannelGone,
    #[error("Communication channel is already connected to from elsewhere")]
    AlreadyConnected,
    #[error("Timed out while waiting for connection to smartknob")]
    Timeout(#[from] Elapsed),
}

pub struct PostcardCobsCodec<'a>(cobs::CobsDecoder<'a>);

impl<'a> PostcardCobsCodec<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self(cobs::CobsDecoder::new(buf))
    }
}

impl<'a> Decoder for PostcardCobsCodec<'a> {
    type Item = comm::Comm;
    type Error = CommError;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        // Pre-allocate buffer size for maximum received packet length
        src.reserve(cobs::max_encoding_length(comm::Comm::POSTCARD_MAX_SIZE));
        match self.0.push(src) {
            Ok(None) => {
                // All bytes were consumed
                src.clear();
                Ok(None)
            }
            Ok(Some(report)) => {
                let _ = src.split_to(report.parsed_size());
                Ok(Some(from_bytes(self.0.dest())?))
            }
            Err(DecodeError::EmptyFrame) => Ok(None),
            Err(DecodeError::InvalidFrame { decoded_bytes }) => {
                // clear the input buffer to get rid of any remaining wong bytes
                // TODO: Not sure if this is the best way to handle things
                src.clear();
                Err(CommError::CobsDecodeError(DecodeError::InvalidFrame {
                    decoded_bytes,
                }))
            }
            Err(e) => Err(e.into()),
        }
    }
}

impl<'a> Encoder<&comm::Command> for PostcardCobsCodec<'a> {
    type Error = CommError;

    fn encode(&mut self, item: &comm::Command, dst: &mut BytesMut) -> Result<(), Self::Error> {
        let ser = to_stdvec_cobs(item)?;
        dst.reserve(ser.len());
        dst.put(ser.as_slice());
        Ok(())
    }
}

fn find_in_acm_mode(ports: Vec<SerialPortInfo>) -> Option<(SerialPortInfo, SerialPortInfo)> {
    let mut sk_ports: Vec<_> = ports
        .into_iter()
        .filter(|p| {
            matches!(&p.port_type, SerialPortType::UsbPort(UsbPortInfo {
                vid: comm::VID,
                pid: comm::PID,
                manufacturer: Some(man),
                product: Some(prod),
                ..
                })
            if man.as_str() == comm::MANUFACTURER && prod.as_str() == comm::PRODUCT)
        })
        .collect();
    if sk_ports.len() == 2 {
        let commands_port = sk_ports.swap_remove(0);
        let log_port = sk_ports.swap_remove(0);
        Some((commands_port, log_port))
    } else {
        None
    }
}

fn find_in_jtag_mode(ports: Vec<SerialPortInfo>) -> Option<SerialPortInfo> {
    let mut sk_ports: Vec<_> = ports
        .into_iter()
        .filter(|p| {
            matches!(&p.port_type, SerialPortType::UsbPort(UsbPortInfo {
                vid: JTAG_VID,
                pid: JTAG_PID,
                manufacturer: Some(man),
                product: Some(prod),
                ..
            }) if man.as_str() == JTAG_MANUFACTURER && prod.as_str() == JTAG_PRODUCT)
        })
        .collect();
    if sk_ports.len() == 1 {
        let port = sk_ports.remove(0);
        Some(port)
    } else {
        None
    }
}

async fn connect_to_sk() -> Result<(SerialPortInfo, SerialPortInfo), ConnectionError> {
    let all_ports = available_ports().map_err(ConnectionError::ListingPorts)?;
    // Shortest/easiest path so we try that first
    if let Some(ports) = find_in_acm_mode(all_ports.clone()) {
        return Ok(ports);
    }
    if let Some(port) = find_in_jtag_mode(all_ports) {
        let mut port = tokio_serial::new(port.port_name, 0)
            .open()
            .map_err(ConnectionError::Connecting)?;
        // write the special character to get the embedded device to switch to ACM mode
        // This may need to have some retry logic in case the sending fails
        port.write_all(SMARTKNOB_TRANSTION_MAGIC_BYTE)?;
        // Ignore errors here in case the port is already gone
        let _ = port.flush();
        drop(port);
        async fn try_find() -> (SerialPortInfo, SerialPortInfo) {
            loop {
                if let Ok(all_ports) = available_ports() {
                    match find_in_acm_mode(all_ports) {
                        Some(ports) => return ports,
                        None => sleep(Duration::from_millis(50)).await,
                    }
                } else {
                    sleep(Duration::from_millis(50)).await
                }
            }
        }
        if let Ok(ports) = timeout(Duration::from_secs(3), try_find()).await {
            Ok(ports)
        } else {
            Err(ConnectionError::Transitionioning)
        }
    } else {
        Err(ConnectionError::Discovering)
    }
}

async fn connect_with_retry(port: SerialPortInfo) -> Result<SerialStream, ConnectionError> {
    async fn try_connect(port: &SerialPortInfo) -> Result<SerialStream, ConnectionError> {
        loop {
            match tokio_serial::new(&port.port_name, 0).open_native_async() {
                Ok(port) => return Ok(port),
                Err(tokio_serial::Error {
                    kind: tokio_serial::ErrorKind::NoDevice,
                    ..
                }) => {
                    return Err(ConnectionError::AlreadyConnected);
                }
                Err(_) => sleep(Duration::from_millis(50)).await,
            }
        }
    }
    timeout(Duration::from_secs(3), try_connect(&port)).await?
}

/// Currently active receiver channels
#[derive(Default, Clone, Copy)]
pub struct ActiveCommChannels {
    /// Communication channel for receiving events from the smartknob
    events: bool,
    /// Communication channel for receiving responses from the smartknob
    responses: bool,
}

async fn serial_listener(
    mut stream: SplitStream<Framed<SerialStream, PostcardCobsCodec<'_>>>,
    response_sender: mpsc::Sender<comm::Response>,
    event_sender: mpsc::Sender<comm::Event>,
    mut channel_receiver: watch::Receiver<ActiveCommChannels>,
) {
    let mut active_channels = channel_receiver.borrow_and_update().clone();
    loop {
        if let Ok(true) = channel_receiver.has_changed() {
            active_channels = channel_receiver.borrow_and_update().clone();
        }

        match stream.next().await {
            Some(Ok(comm::Comm::Response(r))) => {
                if active_channels.responses && response_sender.send(r).await.is_err() {
                    error!("Could not notify of new response");
                    return;
                }
            }
            Some(Ok(comm::Comm::Event(r))) if active_channels.events => {
                if event_sender.send(r).await.is_err() {
                    error!("Could not notify of new event");
                    return;
                }
            }
            Some(Err(e)) => {
                error!("Error listener: {}", e)
            }
            _ => {}
        }
    }
}

pub async fn log_printer(mut stream: Framed<SerialStream, LinesCodec>) {
    loop {
        match stream.next().await {
            Some(Ok(line)) => {
                info!(target: "embedded", "{}", line);
            }
            Some(Err(e)) => {
                error!("Error log_printer: {}", e);
            }
            _ => {}
        }
    }
}

pub async fn event_printer(mut event_receiver: Receiver<comm::Event>) {
    loop {
        match event_receiver.recv().await {
            Some(e) => {
                info!("Event: {e:#?}");
            }
            None => {
                info!("Channel was closed");
                return;
            }
        }
    }
}

pub struct CommChannels<'a> {
    requests: SplitSink<Framed<SerialStream, PostcardCobsCodec<'static>>, &'a comm::Command>,
    responses: Receiver<comm::Response>,
    events: Option<Receiver<comm::Event>>,
    active_channels: ActiveCommChannels,
    active_channel_tx: watch::Sender<ActiveCommChannels>,
}

impl<'a> CommChannels<'a> {
    /// Send a command to the smartknob and wait for a response which is returned.
    /// In case of an error automatically re-tries to get a response from the smartknob
    pub async fn send_command(
        &mut self,
        msg: &'a comm::Command,
    ) -> Result<comm::Response, CommError> {
        self.active_channels.responses = true;
        self.active_channel_tx.send(self.active_channels)?;
        let mut num_retries = NUM_MAX_REPEAT;
        loop {
            self.requests.send(msg).await?;
            let resp = timeout(Duration::from_secs(3), self.responses.recv())
                .await
                .map_err(CommError::ExceededTimeout)?
                .ok_or(CommError::GettingResponse)?;
            if let comm::Response::Repeat = resp {
                if num_retries == 0 {
                    return Err(CommError::ExceededRetries);
                } else {
                    num_retries -= 1;
                    warn!("Retrying sending response. {num_retries} retries left");
                }
            } else {
                return Ok(resp);
            }
        }
    }

    /// Note that calling this will enable generation of events on the event channel
    /// which will block reception of command responses if not consumed
    pub fn take_event_receiver(&mut self) -> Result<Receiver<comm::Event>, CommError> {
        self.active_channels.events = true;
        self.active_channel_tx.send(self.active_channels)?;
        Ok(self.events.take().ok_or(CommError::EventChannelGone)?)
    }
}

pub struct Communication {
    commands_port: Option<SerialPortInfo>,
    log_port: Option<SerialPortInfo>,
}

impl Communication {
    pub async fn init() -> Result<Self, ConnectionError> {
        let (commands_port, log_port) = connect_to_sk().await?;
        Ok(Self {
            commands_port: Some(commands_port),
            log_port: Some(log_port),
        })
    }

    pub async fn take_log(&mut self) -> Result<Framed<SerialStream, LinesCodec>, ConnectionError> {
        let port = connect_with_retry(
            self.log_port
                .take()
                .ok_or(ConnectionError::LogChannelGone)?,
        )
        .await?;
        Ok(Framed::new(port, LinesCodec::new()))
    }

    pub async fn take_comms<'a>(&mut self) -> Result<CommChannels<'a>, ConnectionError> {
        let port = connect_with_retry(
            self.commands_port
                .take()
                .ok_or(ConnectionError::ComChannelGone)?,
        )
        .await?;
        static BUFFER: StaticCell<[u8; max_encoding_length(comm::Comm::POSTCARD_MAX_SIZE)]> =
            StaticCell::new();
        let buffer = BUFFER.init([0u8; _]);
        let (tx, rx) = Framed::new(port, PostcardCobsCodec::new(buffer.as_mut_slice())).split();

        let (response_tx, response_rx) = mpsc::channel(1);
        let (events_tx, events_rx) = mpsc::channel(20);

        let (active_channel_tx, active_channel_rx) = watch::channel(ActiveCommChannels::default());
        task::spawn(serial_listener(
            rx,
            response_tx,
            events_tx,
            active_channel_rx,
        ));
        Ok(CommChannels {
            events: Some(events_rx),
            requests: tx,
            responses: response_rx,
            active_channels: ActiveCommChannels::default(),
            active_channel_tx,
        })
    }
}
