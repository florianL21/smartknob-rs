use crate::comm::LogChannel;
use crate::display::DISPLAY_BRIGHTNESS_SIGNAL;
use crate::flash::{FlashHandling, FlashKeys};
use crate::haptics::{MOTOR_COMMAND_SIGNAL, MotorCommand};
use crate::shutdown::REQUEST_POWER_DOWN;
use crate::system_settings::log_toggles::{LogChannelToggles, LogToggleSender};
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
use embedded_storage::nor_flash::NorFlash;
use fixed::types::I16F16;
use log::{error, info};
use postcard::{experimental::max_size::MaxSize, from_bytes, to_slice_cobs};

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

pub async fn run_request_handler<D: Driver<'static>, Flash: NorFlash, F: FlashHandling<Flash>>(
    mut receiver: Receiver<'static, D>,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
    log_toggles_initial: LogChannelToggles,
    log_toggle_sender: LogToggleSender,
    flash: &'static F,
) {
    let mut current_toggle_state = log_toggles_initial;
    loop {
        receiver.wait_connection().await;
        info!("Connected");
        read_requests(
            &mut receiver,
            comm_tx,
            &mut current_toggle_state,
            &log_toggle_sender,
            flash,
        )
        .await
        .ok();
        info!("Disconnected");
    }
}

async fn read_requests<'d, D: Driver<'static>, Flash: NorFlash, F: FlashHandling<Flash>>(
    receiver: &mut Receiver<'static, D>,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
    current_toggle_state: &mut LogChannelToggles,
    log_toggle_sender: &LogToggleSender,
    flash: &'static F,
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
                        handle_request(
                            cmd,
                            comm_tx,
                            current_toggle_state,
                            &log_toggle_sender,
                            flash,
                        )
                        .await;
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

async fn set_log<Flash: NorFlash, F: FlashHandling<Flash>>(
    current_toggle_state: &mut LogChannelToggles,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
    log_toggle_sender: &LogToggleSender,
    flash: &'static F,
    channel: LogChannel,
    state: bool,
) {
    current_toggle_state.set(&channel, state);
    info!("Enabled logging for channel {channel:?}");
    log_toggle_sender.send(current_toggle_state.clone());
    match flash
        .store::<_, { LogChannelToggles::POSTCARD_MAX_SIZE }>(
            &FlashKeys::LogChannels,
            current_toggle_state,
        )
        .await
    {
        Ok(_) => {
            comm_tx
                .send(comm::Comm::Response(comm::Response::Ack))
                .await;
        }
        Err(e) => {
            error!("Failed to store log toggle update: {e}");
            comm_tx
                .send(comm::Comm::Response(comm::Response::Error(
                    comm::EmbeddedError::FlashError,
                )))
                .await
        }
    }
}

async fn handle_request<Flash: NorFlash, F: FlashHandling<Flash>>(
    request: comm::Command,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
    current_toggle_state: &mut LogChannelToggles,
    log_toggle_sender: &LogToggleSender,
    flash: &'static F,
) {
    async fn ack(comm_tx: channel::DynamicSender<'static, comm::Comm>) {
        comm_tx
            .send(comm::Comm::Response(comm::Response::Ack))
            .await
    }
    info!("Got command of {request:?}");
    match request {
        comm::Command::MotorCalibrate => {
            info!("Starting motor alignment");
            MOTOR_COMMAND_SIGNAL.signal(MotorCommand::StartAlignment);
            ack(comm_tx).await;
        }
        comm::Command::EncoderValidate => {
            info!("Validating encoder linearity. Do not touch the motor!");
            MOTOR_COMMAND_SIGNAL.signal(MotorCommand::VerifyEncoder);
            ack(comm_tx).await;
        }
        comm::Command::MotorTune(value) => {
            MOTOR_COMMAND_SIGNAL.signal(MotorCommand::TuneAlignment(I16F16::from_num(value)));
            ack(comm_tx).await;
        }
        comm::Command::MotorTuneStore => {
            MOTOR_COMMAND_SIGNAL.signal(MotorCommand::TuneStore);
            ack(comm_tx).await;
        }
        comm::Command::Beep {
            freq,
            duration,
            volume,
            note_offset,
        } => {
            MOTOR_COMMAND_SIGNAL.signal(MotorCommand::Beep {
                freq: I16F16::from_num(freq),
                duration: Duration::from_millis(duration),
                volume: I16F16::from_num(volume),
                note_offset: note_offset,
            });
            ack(comm_tx).await;
        }
        comm::Command::Brightness { percent } => {
            let percent = (percent).clamp(0, 100);
            info!("Setting display brightness to {percent}%");
            DISPLAY_BRIGHTNESS_SIGNAL.signal(percent);
            ack(comm_tx).await;
        }
        comm::Command::Shutdown => {
            info!("System shutdown requested");
            ack(comm_tx).await;
            REQUEST_POWER_DOWN.signal(true);
        }
        comm::Command::FlashErase => {
            info!("Formatting flash");
            match flash.format().await {
                Ok(_) => {
                    ack(comm_tx).await;
                }
                Err(e) => {
                    error!("Failed to format flash: {e}");
                    comm_tx
                        .send(comm::Comm::Response(comm::Response::Error(
                            comm::EmbeddedError::FlashError,
                        )))
                        .await
                }
            }
        }
        comm::Command::LogDisable(channel) => {
            set_log(
                current_toggle_state,
                comm_tx,
                log_toggle_sender,
                flash,
                channel,
                false,
            )
            .await
        }
        comm::Command::LogEnable(channel) => {
            set_log(
                current_toggle_state,
                comm_tx,
                log_toggle_sender,
                flash,
                channel,
                true,
            )
            .await
        }
        comm::Command::Ping => {
            ack(comm_tx).await;
        }
    }
}
