use crate::{comm, knob_tilt::KnobTiltEvent};
use crate::{
    haptics::get_encoder_position,
    knob_tilt::{get_tilt_angle, get_tilt_magnitude},
};
use cobs::{CobsDecoder, max_encoding_length};
use embassy_futures::select::select;
use embassy_sync::channel;
use embassy_sync::pubsub::DynSubscriber;
use embassy_time::{Duration, Ticker};
use embassy_usb::driver::Driver;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, CdcAcmError, Receiver, Sender},
    driver::EndpointError,
};
use embedded_io_async::Write;
use log::{error, info};
use postcard::{experimental::max_size::MaxSize, from_bytes, to_slice_cobs};

// static COMM_CHANNEL: Channel<CriticalSectionRawMutex, comm::Comm, 10> = Channel::new();

//TODO: Remove this const and make the functions generic instead
const MAX_PACKET_SIZE: u16 = 64;

pub async fn run_logger<D: Driver<'static>>(class: CdcAcmClass<'static, D>) {
    const RESET: &str = "\u{001B}[0m";
    const RED: &str = "\u{001B}[31m";
    const GREEN: &str = "\u{001B}[32m";
    const YELLOW: &str = "\u{001B}[33m";
    const BLUE: &str = "\u{001B}[34m";
    const CYAN: &str = "\u{001B}[35m";

    let log_fut = embassy_usb_logger::with_custom_style!(
        1024,
        log::LevelFilter::Info,
        class,
        |record, writer| {
            use core::fmt::Write;
            let level = record.level().as_str();
            let color = match record.level() {
                log::Level::Error => RED,
                log::Level::Warn => YELLOW,
                log::Level::Info => GREEN,
                log::Level::Debug => BLUE,
                log::Level::Trace => CYAN,
            };
            write!(writer, "{}[{level}] {}{}\r\n", color, record.args(), RESET).unwrap();
        }
    );
    log_fut.await; // never returns
}

pub async fn run_comm_sender<D: Driver<'static>>(
    mut sender: Sender<'static, D>,
    comm_rx: channel::DynamicReceiver<'static, comm::Comm>,
) {
    loop {
        sender.wait_connection().await;
        info!("Sender connected");
        let _ = stream_events(&mut sender, comm_rx).await;
        info!("Sender disconnected");
    }
}

async fn stream_events<'d, D: Driver<'static>>(
    sender: &mut Sender<'static, D>,
    comm_rx: channel::DynamicReceiver<'static, comm::Comm>,
) -> Result<(), Disconnected> {
    let mut buf = [0u8; max_encoding_length(comm::Comm::POSTCARD_MAX_SIZE)];
    loop {
        let data = comm_rx.receive().await;
        let encoded = to_slice_cobs(&data, &mut buf).unwrap();
        sender.write_all(encoded).await?;
        if let comm::Comm::Response(x) = data {
            info!("Response was sent: {x:?}");
        }
    }
}

pub async fn run_event_sender(
    mut events: DynSubscriber<'static, KnobTiltEvent>,
    poll_rate: Duration,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
) {
    let mut ticker = Ticker::every(poll_rate);
    let mut last_encoder_pos = get_encoder_position();
    let mut is_tilted = false;
    let mut last_tilt = comm::Event::Tilt {
        angle: get_tilt_angle(),
        magnitude: get_tilt_magnitude(),
    };
    loop {
        match select(ticker.next(), events.next_message()).await {
            embassy_futures::select::Either::First(_) => {
                let current_pos = get_encoder_position();
                // TODO: Consider making this approximateley equal
                if current_pos != last_encoder_pos {
                    last_encoder_pos = current_pos;
                    comm_tx
                        .send(comm::Comm::Event(comm::Event::EncoderAngle(
                            get_encoder_position(),
                        )))
                        .await;
                }
                if is_tilted {
                    let new = comm::Event::Tilt {
                        angle: get_tilt_angle(),
                        magnitude: get_tilt_magnitude(),
                    };
                    if new != last_tilt {
                        last_tilt = new;
                        comm_tx
                            .send(comm::Comm::Event(comm::Event::Tilt {
                                angle: get_tilt_angle(),
                                magnitude: get_tilt_magnitude(),
                            }))
                            .await;
                    }
                }
            }
            embassy_futures::select::Either::Second(msg) => match msg {
                embassy_sync::pubsub::WaitResult::Lagged(num_missed) => {
                    comm_tx
                        .send(comm::Comm::Event(comm::Event::MissedEvents(num_missed)))
                        .await;
                }
                embassy_sync::pubsub::WaitResult::Message(msg) => {
                    match msg {
                        KnobTiltEvent::TiltStart(_) => {
                            is_tilted = true;
                        }
                        KnobTiltEvent::TiltEnd => {
                            is_tilted = false;
                        }
                        _ => {}
                    }
                    comm_tx
                        .send(comm::Comm::Event(comm::Event::Button(msg)))
                        .await;
                }
            },
        }
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

impl From<CdcAcmError> for Disconnected {
    fn from(_: CdcAcmError) -> Self {
        Disconnected {}
    }
}

pub async fn run_request_handler<D: Driver<'static>>(
    mut receiver: Receiver<'static, D>,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
) {
    loop {
        receiver.wait_connection().await;
        info!("Connected");
        read_requests(&mut receiver, comm_tx).await.ok();
        info!("Disconnected");
    }
}

async fn read_requests<'d, D: Driver<'static>>(
    receiver: &mut Receiver<'static, D>,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
) -> Result<(), Disconnected> {
    let mut buf = [0u8; MAX_PACKET_SIZE as usize];
    // The buffer must be at least as long as a single packet can be to prevent some decoding errors
    const MAX_SIZE: usize =
        max_encoding_length(comm::Command::POSTCARD_MAX_SIZE) + MAX_PACKET_SIZE as usize;

    let mut msg_buf = [0u8; MAX_SIZE];
    let mut decoder = CobsDecoder::new(&mut msg_buf);
    loop {
        let size = receiver.read_packet(&mut buf).await?;
        match decoder.push(&buf[..size]) {
            Ok(None) => {
                // Dont' need to do anything. Waiting for more bytes to be sent
            }
            Ok(Some(msg)) => {
                let frame = &decoder.dest()[..msg.frame_size()];
                match from_bytes::<comm::Command>(frame) {
                    Ok(cmd) => {
                        handle_request(cmd, comm_tx).await;
                    }
                    Err(e) => {
                        error!("Postcard parsing error: {e}");
                        comm_tx
                            .send(comm::Comm::Response(comm::Response::Error(e.into())))
                            .await;
                    }
                }
            }
            Err(e) => {
                info!("Incoming bytes: {size}; buffer size: {MAX_SIZE}");
                error!("COBS decoding error: {e}");
                // Clear the internal state of the decoder
                decoder = CobsDecoder::new(&mut msg_buf);
                comm_tx
                    .send(comm::Comm::Response(comm::Response::Repeat))
                    .await;
            }
        }
    }
}

async fn handle_request(
    request: comm::Command,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
) {
    info!("Got command of {request:?}");
    match request {
        _ => {
            comm_tx
                .send(comm::Comm::Response(comm::Response::Ack))
                .await
        }
    }
}
