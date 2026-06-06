pub mod button;
pub mod display;
pub mod foc;
pub mod led;
pub mod shutdown;

use crate::flash::{flash_task, init_flash};
use crate::uplink::initialize_uplink;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::channel;
use embassy_sync::pubsub::{DynPublisher, DynSubscriber, PubSubChannel};
use embassy_time::Timer;
use esp_hal::interrupt::software::SoftwareInterrupt;
use esp_hal::otg_fs::{UsbDm, UsbDp};
use esp_hal::peripherals;
use esp_hal::system::Stack;
use esp_rtos::embassy::Executor;
use log::info;
use smartknob_core::comm;
use smartknob_core::haptics::CalibrationData;
use smartknob_core::knob_tilt::KnobTiltEvent;
use smartknob_core::system_settings::{HapticSystemStoreSignal, StoreSignals};
use smartknob_core::{
    haptics::get_encoder_position,
    system_settings::log_toggles::{LogChannel, LogToggleReceiver, LogToggleWatcher, may_log},
};
use static_cell::StaticCell;

pub trait FOCBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        restored_state: Option<CalibrationData>,
        settings_store_signals: &'static HapticSystemStoreSignal<CriticalSectionRawMutex>,
    );
}
pub trait LEDBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        tilt_receiver: DynSubscriber<'static, KnobTiltEvent>,
    );
}
pub trait ButtonBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        event_sender: DynPublisher<'static, KnobTiltEvent>,
    );
}
pub trait DisplayBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        knob_tilt: DynSubscriber<'static, KnobTiltEvent>,
    );
}
pub trait ShutdownBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
    );
}

pub struct NoButtonBlock {}
impl ButtonBlock for NoButtonBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        _: Spawner,
        _: &'static LogToggleWatcher<M, N>,
        _: DynPublisher<'static, KnobTiltEvent>,
    ) {
    }
}
pub struct NoLEDBlock {}
impl LEDBlock for NoLEDBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        _: Spawner,
        _: &'static LogToggleWatcher<M, N>,
        _: DynSubscriber<'static, KnobTiltEvent>,
    ) {
    }
}
pub struct NoDisplayBlock {}
impl DisplayBlock for NoDisplayBlock {
    fn init<M: RawMutex, const N: usize>(
        self,
        _: Spawner,
        _: &'static LogToggleWatcher<M, N>,
        _: DynSubscriber<'static, KnobTiltEvent>,
    ) {
    }
}
pub struct NoShutdownBlock {}
impl ShutdownBlock for NoShutdownBlock {
    fn init<M: RawMutex, const N: usize>(self, _: Spawner, _: &'static LogToggleWatcher<M, N>) {}
}

pub struct Peripherals {
    pub flash: peripherals::FLASH<'static>,
    pub cpu_ctrl: peripherals::CPU_CTRL<'static>,
    pub usb: peripherals::USB0<'static>,
    pub usb_device: peripherals::USB_DEVICE<'static>,
}

pub struct SystemPins<Up: UsbDp + 'static, Um: UsbDm + 'static> {
    pub usb_plus: Up,
    pub usb_minus: Um,
}

pub struct SmartknobSystemConfig<Up: UsbDp + 'static, Um: UsbDm + 'static> {
    pub peripherals: Peripherals,
    pub pins: SystemPins<Up, Um>,
    pub sw_interrupt: SoftwareInterrupt<'static, 1>,
}

#[embassy_executor::task]
async fn log_rotations(mut log_receiver: LogToggleReceiver) {
    loop {
        may_log(&mut log_receiver, LogChannel::Encoder, || {
            let pos = get_encoder_position();
            info!("Position: {pos}")
        })
        .await;
        Timer::after_millis(200).await;
    }
}

pub async fn smartknob_main<Up: UsbDp + 'static, Um: UsbDm + 'static>(
    spawner: Spawner,
    foc: impl FOCBlock + Send + Sync + 'static,
    system_config: SmartknobSystemConfig<Up, Um>,
    buttons: impl ButtonBlock,
    leds: impl LEDBlock,
    display: impl DisplayBlock,
    shutdown: impl ShutdownBlock,
) {
    // TODO: Consider exposing these to the caller
    const NUM_LOG_WATCHERS: usize = 8;
    const NUM_BUTTON_BUFFER: usize = 10;
    const NUM_BUTTON_SUBS: usize = 4;
    const NUM_COMM_CHANNEL: usize = 10;

    type LogWatcher = LogToggleWatcher<CriticalSectionRawMutex, NUM_LOG_WATCHERS>;
    type KnobTiltChannel =
        PubSubChannel<NoopRawMutex, KnobTiltEvent, NUM_BUTTON_BUFFER, NUM_BUTTON_SUBS, 1>;

    let peripherals = system_config.peripherals;
    // Make all neccessary signals
    static SETTING_SIGNALS: StaticCell<StoreSignals<CriticalSectionRawMutex>> = StaticCell::new();
    let setting_signals = SETTING_SIGNALS.init(StoreSignals::new());
    static LOG_TOGGLES: StaticCell<LogWatcher> = StaticCell::new();
    let log_toggles = LOG_TOGGLES.init(LogToggleWatcher::new());
    static KNOB_TILT_PUB_SUB: StaticCell<KnobTiltChannel> = StaticCell::new();
    let knob_tilt = KNOB_TILT_PUB_SUB.init(PubSubChannel::new());
    static HAPTIC_SETTING_SIGNAL: StaticCell<&HapticSystemStoreSignal<CriticalSectionRawMutex>> =
        StaticCell::new();
    let haptic_setting_signal = HAPTIC_SETTING_SIGNAL.init(&setting_signals.haptic_core);
    static COMM_CHANNEL: StaticCell<
        channel::Channel<CriticalSectionRawMutex, comm::Comm, NUM_COMM_CHANNEL>,
    > = StaticCell::new();
    let comm_channel = COMM_CHANNEL.init(channel::Channel::new());

    // Call this early in case the system would shut down if the pin is not in a proper state
    shutdown.init(spawner, log_toggles);

    // First initialize the flash so that we can load persistent data from it like the stored calibration data
    let (flash, restored_state) = init_flash(peripherals.flash).await;
    spawner.spawn(flash_task(flash, setting_signals).unwrap());

    // Inform everyone about the log toggle states loaded from flash
    let log_toggles_initial = restored_state.log_toggles.unwrap_or_default();
    log_toggles.dyn_sender().send(log_toggles_initial.clone());

    // Prepare everything to send it off to the second core which will run the FOC control loop
    static APP_CORE_STACK: StaticCell<Stack<32768>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    static LOG_TOGGLE_REF: StaticCell<&LogWatcher> = StaticCell::new();
    let log_toggles_foc = LOG_TOGGLE_REF.init(log_toggles);
    let motor_calibration = restored_state.haptic_core;

    esp_rtos::start_second_core(
        peripherals.cpu_ctrl,
        system_config.sw_interrupt,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                foc.init(
                    spawner,
                    log_toggles_foc,
                    motor_calibration,
                    haptic_setting_signal,
                );
            });
        },
    );

    buttons.init(
        spawner,
        log_toggles,
        knob_tilt
            .dyn_publisher()
            .expect("Could not create knob tilt subscriber, please increase capacity"),
    );

    initialize_uplink(
        spawner,
        peripherals.usb,
        peripherals.usb_device,
        system_config.pins.usb_plus,
        system_config.pins.usb_minus,
        knob_tilt
            .dyn_subscriber()
            .expect("Could not create knob tilt subscriber for uplink, please increase capacity"),
        comm_channel.dyn_sender(),
        comm_channel.dyn_receiver(),
        log_toggles_initial,
        log_toggles.dyn_sender(),
        flash,
    );

    // log encoder values
    spawner.spawn(log_rotations(log_toggles.dyn_receiver().unwrap()).unwrap());

    leds.init(
        spawner,
        log_toggles,
        knob_tilt.dyn_subscriber().expect(
            "Could not create knob tilt subscriber for LED subsystem, please increase capacity",
        ),
    );

    display.init(
        spawner,
        log_toggles,
        knob_tilt.dyn_subscriber().expect(
            "Could not create knob tilt subscriber for display subsystem, please increase capacity",
        ),
    );

    info!("All tasks spawned");
    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);
}
