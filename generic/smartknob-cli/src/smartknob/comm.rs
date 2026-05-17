use anyhow::{Context, Error, Result, anyhow};
use cobs::max_encoding_length;
use futures::stream::SplitSink;
use futures::stream::SplitStream;
use futures::stream::StreamExt;
use idc::Command;
use idc::Event;
use idc::Response;
use log::{error, info};
use postcard::{experimental::max_size::MaxSize, from_bytes, to_stdvec_cobs};
use static_cell::StaticCell;
use std::time::Duration;
use tokio::task;
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

pub struct PostcardCobsCodec<'a>(cobs::CobsDecoder<'a>);

impl<'a> PostcardCobsCodec<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self(cobs::CobsDecoder::new(buf))
    }
}

impl<'a> Decoder for PostcardCobsCodec<'a> {
    type Item = idc::Comm;
    type Error = Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        match self.0.push(src)? {
            None => {
                // All bytes were consumed
                src.clear();
                Ok(None)
            }
            Some(report) => {
                let _ = src.split_to(report.parsed_size());
                Ok(Some(from_bytes(self.0.dest())?))
            }
        }
    }
}

impl<'a> Encoder<&idc::Command> for PostcardCobsCodec<'a> {
    type Error = Error;

    fn encode(&mut self, item: &idc::Command, dst: &mut BytesMut) -> Result<(), Self::Error> {
        let ser = to_stdvec_cobs(item)?;
        dst.reserve(ser.len());
        dst.put(ser.as_slice());
        dbg!(ser.len());
        Ok(())
    }
}

fn find_in_acm_mode(ports: Vec<SerialPortInfo>) -> Option<(SerialPortInfo, SerialPortInfo)> {
    let mut sk_ports: Vec<_> = ports
        .into_iter()
        .filter(|p| match &p.port_type {
            SerialPortType::UsbPort(UsbPortInfo {
                vid: idc::VID,
                pid: idc::PID,
                manufacturer: Some(man),
                product: Some(prod),
                serial_number: Some(ser),
            }) if man.as_str() == idc::MANUFACTURER
                && prod.as_str() == idc::PRODUCT
                && ser.as_str() == idc::SERIAL =>
            {
                true
            }
            _ => false,
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
        .filter(|p| match &p.port_type {
            SerialPortType::UsbPort(UsbPortInfo {
                vid: JTAG_VID,
                pid: JTAG_PID,
                manufacturer: Some(man),
                product: Some(prod),
                ..
            }) if man.as_str() == JTAG_MANUFACTURER && prod.as_str() == JTAG_PRODUCT => true,
            _ => false,
        })
        .collect();
    if sk_ports.len() == 1 {
        let port = sk_ports.remove(0);
        Some(port)
    } else {
        None
    }
}

async fn connect_to_sk() -> Result<(SerialPortInfo, SerialPortInfo)> {
    let all_ports = available_ports().context("Could not list serial ports")?;
    // Shortest/easiest path so we try that first
    if let Some(ports) = find_in_acm_mode(all_ports.clone()) {
        return Ok(ports);
    }
    if let Some(port) = find_in_jtag_mode(all_ports) {
        let mut port = tokio_serial::new(port.port_name, 0)
            .open()
            .context("Could not open JTAG communications serial prot")?;
        // write the special character to get the embedded device to switch to ACM mode
        port.write_all(&[35])
            .context("Could not send magic byte to smartknob")?;
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
            Err(anyhow!("Could not transition smartknob device to ACM mode"))
        }
    } else {
        Err(anyhow!("Could not find a matching smartknob device"))
    }
}

async fn connect_with_retry(port: SerialPortInfo) -> Option<SerialStream> {
    async fn try_connect(port: &SerialPortInfo) -> SerialStream {
        loop {
            match tokio_serial::new(&port.port_name, 0).open_native_async() {
                Ok(port) => return port,
                Err(_) => sleep(Duration::from_millis(50)).await,
            }
        }
    }
    timeout(Duration::from_secs(3), try_connect(&port))
        .await
        .ok()
}

async fn serial_listener(
    mut stream: SplitStream<Framed<SerialStream, PostcardCobsCodec<'_>>>,
    response_sender: mpsc::Sender<idc::Response>,
    event_sender: mpsc::Sender<idc::Event>,
) {
    loop {
        match stream.next().await {
            Some(Ok(idc::Comm::Response(r))) => {
                if let Err(_) = response_sender.send(r).await {
                    error!("Could not notify of new response");
                    return;
                }
            }
            Some(Ok(idc::Comm::Event(r))) => {
                if let Err(_) = event_sender.send(r).await {
                    error!("Could not notify of new event");
                    return;
                }
            }

            Some(Err(e)) => {
                error!("Error: {}", e)
            }
            _ => {}
        }
    }
}

async fn log_listener(mut stream: Framed<SerialStream, LinesCodec>) {
    loop {
        match stream.next().await {
            Some(Ok(line)) => {
                info!(target: "embedded", "{}", line);
            }

            Some(Err(e)) => {
                error!("Error: {}", e)
            }
            _ => {}
        }
    }
}

pub struct CommChannels<'a> {
    pub requests: SplitSink<Framed<SerialStream, PostcardCobsCodec<'static>>, &'a Command>,
    pub responses: Receiver<Response>,
    pub events: Receiver<Event>,
}

pub async fn init_comms<'a>() -> Result<CommChannels<'a>> {
    let (commands_port, log_port) = connect_to_sk().await?;
    let port = connect_with_retry(log_port)
        .await
        .context("Could not open logging serial prot")?;

    let log_stream = Framed::new(port, LinesCodec::new());
    task::spawn(log_listener(log_stream));

    let port = connect_with_retry(commands_port)
        .await
        .context("Could not open communications serial prot")?;

    static BUFFER: StaticCell<[u8; max_encoding_length(idc::Comm::POSTCARD_MAX_SIZE)]> =
        StaticCell::new();
    let buffer = BUFFER.init([0u8; _]);
    let (tx, rx) = Framed::new(port, PostcardCobsCodec::new(buffer.as_mut_slice())).split();

    let (response_tx, response_rx) = mpsc::channel(1);
    let (events_tx, events_rx) = mpsc::channel(20);
    task::spawn(serial_listener(rx, response_tx, events_tx));
    Ok(CommChannels {
        events: events_rx,
        requests: tx,
        responses: response_rx,
    })
}
