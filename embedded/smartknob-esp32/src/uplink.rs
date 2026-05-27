use cobs::{CobsDecoder, max_encoding_length};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::Timer;
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
    driver::EndpointError,
};
use embedded_io_async::Read;
use esp_hal::{
    Async,
    otg_fs::{
        Usb, UsbDm, UsbDp,
        asynch::{Config, Driver},
    },
    peripherals::{USB_DEVICE, USB0},
    usb_serial_jtag::UsbSerialJtag,
};
use log::{error, info};
use postcard::{experimental::max_size::MaxSize, from_bytes, to_slice_cobs};
use smartknob_core::idc;
use static_cell::StaticCell;

const MAX_PACKET_SIZE: u16 = 64;
static USB_CDC_TRIGGER: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static COMM_CHANNEL: Channel<CriticalSectionRawMutex, idc::Comm, 10> = Channel::new();

pub fn initialize_uplink(
    spawner: Spawner,
    usb: USB0<'static>,
    usb_device: USB_DEVICE<'static>,
    dp: impl UsbDp + 'static,
    dm: impl UsbDm + 'static,
) {
    let usb = Usb::new(usb, dp, dm);

    // Create the driver, from the HAL.
    static SP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let ep_out_buffer = SP_OUT_BUFFER.init([0u8; 1024]);
    let config = Config::default();

    let driver = Driver::new(usb, ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(idc::VID, idc::PID);
    config.manufacturer = Some(idc::MANUFACTURER);
    config.product = Some(idc::PRODUCT);
    config.serial_number = Some(idc::SERIAL);

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]);
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; 64]);

    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create classes on the builder.
    static EVENT_STATE: StaticCell<State> = StaticCell::new();
    let state = EVENT_STATE.init(State::new());
    let events_class = CdcAcmClass::new(&mut builder, state, MAX_PACKET_SIZE);
    let (tx, rx) = events_class.split();
    static LOGGING_STATE: StaticCell<State> = StaticCell::new();
    let state = LOGGING_STATE.init(State::new());
    let logging_class = CdcAcmClass::new(&mut builder, state, MAX_PACKET_SIZE);

    let serial = UsbSerialJtag::new(usb_device).into_async();

    spawner.spawn(serial_watcher(serial).unwrap());

    spawner.spawn(usb_task(builder).unwrap());
    spawner.spawn(logger_task(logging_class).unwrap());
    spawner.spawn(comm_sender(tx).unwrap());
    spawner.spawn(event_sender().unwrap());
    spawner.spawn(request_handler(rx).unwrap());
}

#[embassy_executor::task]
async fn logger_task(class: CdcAcmClass<'static, Driver<'static>>) {
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

#[embassy_executor::task]
async fn usb_task(builder: Builder<'static, Driver<'static>>) {
    USB_CDC_TRIGGER.wait().await;
    let mut usb = builder.build();
    usb.run().await;
}

#[embassy_executor::task]
async fn comm_sender(mut sender: Sender<'static, Driver<'static>>) {
    loop {
        sender.wait_connection().await;
        info!("Sender connected");
        let _ = stream_events(&mut sender).await;
        info!("Sender disconnected");
    }
}

async fn stream_events<'d>(
    sender: &mut Sender<'static, Driver<'static>>,
) -> Result<(), Disconnected> {
    let mut buf = [0u8; max_encoding_length(idc::Comm::POSTCARD_MAX_SIZE)];

    loop {
        let data = COMM_CHANNEL.receive().await;
        let encoded = to_slice_cobs(&data, &mut buf).unwrap();
        sender.write_packet(encoded).await?;
    }
}

#[embassy_executor::task]
async fn event_sender() {
    let data = idc::Comm::Event(idc::Event::Button(idc::ButtonEvent::PressDown));
    loop {
        Timer::after_secs(1).await;
        COMM_CHANNEL.send(data.clone()).await;
        info!("Sent Button press event");
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

#[embassy_executor::task]
async fn request_handler(mut receiver: Receiver<'static, Driver<'static>>) {
    loop {
        receiver.wait_connection().await;
        info!("Connected");
        read_requests(&mut receiver).await.ok();
        info!("Disconnected");
    }
}

async fn read_requests<'d>(
    receiver: &mut Receiver<'static, Driver<'static>>,
) -> Result<(), Disconnected> {
    let mut buf = [0u8; MAX_PACKET_SIZE as usize];
    // The buffer must be at least as long as a single packet can be to prevent some decoding errors
    const MAX_SIZE: usize =
        max_encoding_length(idc::Command::POSTCARD_MAX_SIZE) + MAX_PACKET_SIZE as usize;

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
                match from_bytes::<idc::Command>(frame) {
                    Ok(cmd) => {
                        handle_request(cmd).await;
                    }
                    Err(e) => {
                        error!("Postcard parsing error: {e}");
                        COMM_CHANNEL
                            .send(idc::Comm::Response(idc::Response::Error(e.into())))
                            .await;
                    }
                }
            }
            Err(e) => {
                error!("COBS decoding error: {e}");
                COMM_CHANNEL
                    .send(idc::Comm::Response(idc::Response::Repeat))
                    .await;
            }
        }
    }
}

async fn handle_request(request: idc::Command) {
    info!("Got command of {request:?}");
    match request {
        _ => {
            COMM_CHANNEL
                .send(idc::Comm::Response(idc::Response::Ack))
                .await
        }
    }
}

#[embassy_executor::task]
pub async fn serial_watcher(serial: UsbSerialJtag<'static, Async>) {
    let (mut rx, _) = serial.split();

    let mut buf: [u8; 1] = [0; 1];
    // read byte from somewhere (for example, uart)
    loop {
        let _ = rx.read(&mut buf).await;
        if buf[0] == 35 {
            USB_CDC_TRIGGER.signal(());
            break;
        } else {
            info!("Got: {}", buf[0]);
        }
    }
}
