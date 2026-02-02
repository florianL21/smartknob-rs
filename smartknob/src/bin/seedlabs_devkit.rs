#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{DataPartitionSubType, PartitionType};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::ledc::Ledc;
use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq};
use esp_hal::spi;
use esp_hal::system::Stack;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::CpuClock,
    gpio::{AnyPin, Level, Output, OutputConfig},
    rmt::Rmt,
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    usb_serial_jtag::UsbSerialJtag,
};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use esp_rtos::embassy::Executor;
use log::info;
use smart_leds::{
    SmartLedsWrite, brightness,
    colors::{BLACK, BLUE, RED},
    gamma,
};
use smartknob_core::flash::FlashHandling;
use smartknob_core::haptic_core::get_encoder_position;
use smartknob_core::system_settings::log_toggles::{
    LogChannel, LogToggleReceiver, LogToggleWatcher, may_log,
};
use smartknob_core::system_settings::{HapticSystemStoreSignal, StoreSignals};
use smartknob_esp32::flash::FlashHandler;
use smartknob_esp32::motor_driver::mcpwm::Pins6PWM;
use smartknob_esp32::{
    cli::menu_handler,
    display::{
        BacklightHandles, BacklightTask, DisplayHandles, Orientation,
        slint::{spawn_display_tasks, ui_task},
    },
    knob_tilt::KnobTiltEvent,
};
use smartknob_rs::motor_control::update_foc;
use smartknob_rs::signals::{KNOB_EVENTS_CHANNEL, KNOB_TILT_ANGLE};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

type LogWatcher = LogToggleWatcher<CriticalSectionRawMutex, 6>;

#[embassy_executor::task]
async fn log_rotations(mut log_receiver: LogToggleReceiver) {
    info!("Log encoder init done!");
    loop {
        may_log(&mut log_receiver, LogChannel::Encoder, || {
            let pos = get_encoder_position();
            info!("Position: {pos}")
        })
        .await;
        Timer::after_millis(200).await;
    }
}

fn map<
    T: core::ops::Sub<Output = T>
        + core::ops::Mul<Output = T>
        + core::ops::Div<Output = T>
        + core::ops::Add<Output = T>
        + Copy,
>(
    x: T,
    in_min: T,
    in_max: T,
    out_min: T,
    out_max: T,
) -> T {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[embassy_executor::task]
async fn led_ring(
    rmt_channel: esp_hal::rmt::ChannelCreator<'static, esp_hal::Blocking, 0>,
    led_pin: AnyPin<'static>,
    mut log_receiver: LogToggleReceiver,
) {
    const NUM_LEDS: usize = 72;
    const _LED_OFFSET: usize = 1;

    let mut tilt_receiver = KNOB_EVENTS_CHANNEL.subscriber()
    .expect("No subscriber channels were left for the knob events channel. Consider increating tha number of subscribers");

    let mut rmt_buffer = smart_led_buffer!(NUM_LEDS);
    let mut led = SmartLedsAdapter::new(rmt_channel, led_pin, &mut rmt_buffer);

    let mut data = [RED; NUM_LEDS];
    let _step_size = (2.0f32 * core::f32::consts::PI) / NUM_LEDS as f32;

    info!("LED init done!");
    loop {
        // Since this is only interested in displaying the current state we can ignore lag information
        let tilt_event = tilt_receiver.next_message_pure().await;
        may_log(&mut log_receiver, LogChannel::Push, || {
            info!("Event: {:#?}", &tilt_event)
        })
        .await;
        match tilt_event {
            KnobTiltEvent::PressEnd | KnobTiltEvent::TiltEnd => {
                for item in data.iter_mut() {
                    *item = BLACK;
                }
            }
            KnobTiltEvent::PressStart => {
                for item in data.iter_mut() {
                    *item = RED;
                }
            }
            KnobTiltEvent::TiltStart(_) | KnobTiltEvent::TiltAdjust => {
                let angle = KNOB_TILT_ANGLE.load(core::sync::atomic::Ordering::Relaxed);
                let angle = if angle < 0.0 {
                    angle + 2.0 * core::f32::consts::PI
                } else {
                    angle
                };
                let led_index = map(
                    angle,
                    2.0 * core::f32::consts::PI,
                    0.0,
                    0.0,
                    NUM_LEDS as f32,
                );
                // not yet working
                for (i, item) in data.iter_mut().enumerate() {
                    if i == led_index as usize {
                        *item = BLUE;
                    } else {
                        *item = BLACK;
                    }
                }
            }
        }
        led.write(brightness(gamma(data.iter().cloned()), 10))
            .unwrap();
        Timer::after(Duration::from_millis(20)).await;
    }
}

#[embassy_executor::task]
pub async fn flash_task(
    flash_handler: &'static FlashHandler,
    store_signals: &'static StoreSignals<CriticalSectionRawMutex>,
) {
    loop {
        flash_handler.run(store_signals).await;
    }
}

#[embassy_executor::task]
async fn brightness_task(handles: BacklightHandles, log_receiver: LogToggleReceiver) {
    let mut bl = BacklightTask::new(handles, log_receiver);
    loop {
        bl.run().await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let psram_config = PsramConfig {
        flash_frequency: FlashFreq::FlashFreq80m,
        ram_frequency: SpiRamFreq::Freq80m,
        // core_clock: Some(SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m),
        ..Default::default()
    };

    let config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max())
        .with_psram(psram_config);
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_alloc::heap_allocator!(size: 72 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    esp_rtos::start(timg0.timer0);
    info!("Embassy initialized!");

    let f = FlashHandler::new(
        peripherals.FLASH,
        PartitionType::Data(DataPartitionSubType::Fat),
    )
    .await;

    static FLASH: StaticCell<FlashHandler> = StaticCell::new();
    let flash = FLASH.init(f);
    static SETTING_SIGNALS: StaticCell<StoreSignals<CriticalSectionRawMutex>> = StaticCell::new();
    let setting_signals = SETTING_SIGNALS.init(StoreSignals::new());

    let restored_state = flash.restore().await;
    spawner.must_spawn(flash_task(flash, setting_signals));

    // Pins for WS2812B LEDs
    let pin_led_data = peripherals.GPIO12;

    // pins for TMC6300
    let pin_tmc_uh = peripherals.GPIO8;
    let pin_tmc_ul = peripherals.GPIO16;
    let pin_tmc_vh = peripherals.GPIO18;
    let pin_tmc_vl = peripherals.GPIO7;
    let pin_tmc_wh = peripherals.GPIO17;
    let pin_tmc_wl = peripherals.GPIO15;

    // pins for MT6701-CT
    let pin_mag_clk = peripherals.GPIO13;
    let pin_mag_do = peripherals.GPIO14;
    let pin_mag_csn = peripherals.GPIO11;

    // pins for display
    let pin_lcd_sck = peripherals.GPIO4;
    // let pin_lcd_miso = peripherals.GPIO;
    let pin_lcd_mosi = peripherals.GPIO3;
    let pin_lcd_dc = peripherals.GPIO2;
    let pin_lcd_cs = peripherals.GPIO9;
    let pin_lcd_bl = peripherals.GPIO5;
    let pin_lcd_reset = peripherals.GPIO10;

    // various other pins
    let _brightness_sensor_pin = peripherals.GPIO1;

    static LOG_TOGGLES: StaticCell<LogWatcher> = StaticCell::new();
    let log_toggles = LOG_TOGGLES.init(LogToggleWatcher::new());

    // Encoder initialization
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    static APP_CORE_STACK: StaticCell<Stack<32768>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());

    let motor_calibration = restored_state.haptic_core;

    static HAPTIC_SETTING_SIGNAL: StaticCell<&HapticSystemStoreSignal<CriticalSectionRawMutex>> =
        StaticCell::new();
    let haptic_setting_signal = HAPTIC_SETTING_SIGNAL.init(&setting_signals.haptic_core);
    static LOG_TOGGLE_REF: StaticCell<&LogWatcher> = StaticCell::new();
    let log_toggles_foc = LOG_TOGGLE_REF.init(log_toggles);

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_int.software_interrupt0,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                let spi_bus = Spi::new(
                    peripherals.SPI2,
                    SpiConfig::default()
                        .with_frequency(Rate::from_mhz(5))
                        .with_mode(Mode::_0),
                )
                .unwrap()
                .with_sck(pin_mag_clk)
                .with_miso(pin_mag_do)
                .with_dma(peripherals.DMA_CH0);

                let mag_cs = Output::new(pin_mag_csn, Level::Low, OutputConfig::default());
                let pwm_pins = Pins6PWM::new(
                    pin_tmc_uh.into(),
                    pin_tmc_ul.into(),
                    pin_tmc_vh.into(),
                    pin_tmc_vl.into(),
                    pin_tmc_wh.into(),
                    pin_tmc_wl.into(),
                );

                spawner.must_spawn(update_foc(
                    spi_bus,
                    mag_cs,
                    peripherals.MCPWM0,
                    pwm_pins,
                    motor_calibration,
                    haptic_setting_signal,
                    log_toggles_foc.dyn_receiver().unwrap(),
                ));
            });
        },
    );

    let serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    spawner
        .spawn(menu_handler(serial, flash, log_toggles.dyn_sender()))
        .ok();

    // log encoder values
    spawner.must_spawn(log_rotations(log_toggles.dyn_receiver().unwrap()));

    // LED ring
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    spawner.must_spawn(led_ring(
        rmt.channel0,
        pin_led_data.into(),
        log_toggles.dyn_receiver().unwrap(),
    ));

    // Display
    let spi_bus: spi::master::SpiDma<'_, esp_hal::Blocking> = Spi::new(
        peripherals.SPI3,
        spi::master::Config::default()
            .with_frequency(Rate::from_mhz(80))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(pin_lcd_sck)
    .with_mosi(pin_lcd_mosi)
    .with_dma(peripherals.DMA_CH1);

    let display_handles = DisplayHandles {
        spi_bus,
        lcd_cs: Output::new(pin_lcd_cs, Level::Low, OutputConfig::default()),
        dc_output: Output::new(pin_lcd_dc, Level::Low, OutputConfig::default()),
        reset_output: Output::new(pin_lcd_reset, Level::Low, OutputConfig::default()),
        orientation: Orientation::default(),
    };

    let backlight_stuff = BacklightHandles {
        backlight_output: Output::new(pin_lcd_bl, Level::Low, OutputConfig::default()),
        brightness_sensor: None,
        ledc: Ledc::new(peripherals.LEDC),
    };
    spawn_display_tasks(spawner, display_handles, log_toggles).unwrap();
    // Spawn the taks which sets the actual UI state
    spawner.must_spawn(ui_task());
    spawner.must_spawn(brightness_task(
        backlight_stuff,
        log_toggles
            .dyn_receiver()
            .expect("Could not get log toggle receiver"),
    ));

    info!("All tasks spawned");
    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);
}
