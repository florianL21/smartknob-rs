use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel, pubsub::DynSubscriber, signal::Signal,
};
use embassy_time::Duration;
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
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
use log::info;
use smartknob_core::{
    comm,
    knob_tilt::KnobTiltEvent,
    system_settings::log_toggles::{LogChannelToggles, LogToggleSender},
    uplink::{run_comm_sender, run_event_sender, run_logger, run_request_handler},
};
use static_cell::StaticCell;

use crate::flash::FlashHandler;

const MAX_PACKET_SIZE: u16 = 64;
static USB_CDC_TRIGGER: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub fn initialize_uplink(
    spawner: Spawner,
    usb: USB0<'static>,
    usb_device: USB_DEVICE<'static>,
    dp: impl UsbDp + 'static,
    dm: impl UsbDm + 'static,
    knob_events: DynSubscriber<'static, KnobTiltEvent>,
    comm_channel_sender: channel::DynamicSender<'static, comm::Comm>,
    comm_channel_receiver: channel::DynamicReceiver<'static, comm::Comm>,
    initial_log_toggles: LogChannelToggles,
    log_toggle_sender: LogToggleSender,
    flash: &'static FlashHandler,
) {
    let usb = Usb::new(usb, dp, dm);

    // Create the driver, from the HAL.
    static SP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let ep_out_buffer = SP_OUT_BUFFER.init([0u8; 1024]);
    let config = Config::default();

    let driver = Driver::new(usb, ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(comm::VID, comm::PID);
    config.manufacturer = Some(comm::MANUFACTURER);
    config.product = Some(comm::PRODUCT);
    config.serial_number = Some(comm::SERIAL);

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
    spawner.spawn(comm_sender(tx, comm_channel_receiver).unwrap());
    spawner.spawn(
        event_sender(
            knob_events,
            Duration::from_hz(20),
            comm_channel_sender.clone(),
        )
        .unwrap(),
    );
    spawner.spawn(
        request_handler(
            rx,
            comm_channel_sender.clone(),
            initial_log_toggles,
            log_toggle_sender,
            flash,
        )
        .unwrap(),
    );
}

#[embassy_executor::task]
async fn logger_task(class: CdcAcmClass<'static, Driver<'static>>) {
    run_logger(class).await;
}

#[embassy_executor::task]
async fn usb_task(builder: Builder<'static, Driver<'static>>) {
    USB_CDC_TRIGGER.wait().await;
    let mut usb = builder.build();
    usb.run().await;
}

#[embassy_executor::task]
async fn comm_sender(
    sender: Sender<'static, Driver<'static>>,
    comm_rx: channel::DynamicReceiver<'static, comm::Comm>,
) {
    run_comm_sender(sender, comm_rx).await;
}

#[embassy_executor::task]
async fn event_sender(
    events: DynSubscriber<'static, KnobTiltEvent>,
    poll_rate: Duration,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
) {
    run_event_sender(events, poll_rate, comm_tx).await;
}

#[embassy_executor::task]
async fn request_handler(
    receiver: Receiver<'static, Driver<'static>>,
    comm_tx: channel::DynamicSender<'static, comm::Comm>,
    log_toggles_initial: LogChannelToggles,
    log_toggle_sender: LogToggleSender,
    flash: &'static FlashHandler,
) {
    run_request_handler(
        receiver,
        comm_tx,
        log_toggles_initial,
        log_toggle_sender,
        flash,
    )
    .await;
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
