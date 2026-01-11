#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
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
    gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
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
use smartknob_core::haptic_core::get_encoder_position;
use smartknob_esp32::flash::{FlashHandler, flash_task};
use smartknob_esp32::motor_driver::mcpwm::Pins6PWM;
use smartknob_rs::config::{LogToggleReceiver, LogToggleWatcher};
use smartknob_rs::display::BacklightHandles;
use smartknob_rs::signals::{KNOB_EVENTS_CHANNEL, KNOB_TILT_ANGLE};
use smartknob_rs::{
    cli::menu_handler,
    config::{LogChannel, may_log},
    display::{DisplayHandles, spawn_display_tasks},
    knob_tilt::{KnobTiltEvent, read_ldc_task},
    motor_control::update_foc,
    shutdown::shutdown_handler,
};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

type I2cBus1 = Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;

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
    const NUM_LEDS: usize = 24;
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

    spawner.must_spawn(flash_task(flash));

    // Pins for LDC1614
    let pins_ldc_int_pin = peripherals.GPIO40;
    let pins_i2c_scl = peripherals.GPIO42;
    let pins_i2c_sda = peripherals.GPIO41;

    // Pins for WS2812B LEDs
    let pin_led_data = peripherals.GPIO39;

    // pins for TMC6300
    let _pin_tmc_diag = peripherals.GPIO47;
    let pin_tmc_uh = peripherals.GPIO48;
    let pin_tmc_ul = peripherals.GPIO17;
    let pin_tmc_vh = peripherals.GPIO21;
    let pin_tmc_vl = peripherals.GPIO46;
    let pin_tmc_wh = peripherals.GPIO18;
    let pin_tmc_wl = peripherals.GPIO45;

    // pins for MT6701-CT
    let pin_mag_clk = peripherals.GPIO11;
    let pin_mag_do = peripherals.GPIO10;
    let pin_mag_csn = peripherals.GPIO12;
    let _pin_mag_push = peripherals.GPIO3;

    // pins for display
    let pin_lcd_sck = peripherals.GPIO13;
    // let pin_lcd_miso = peripherals.GPIO;
    let pin_lcd_mosi = peripherals.GPIO14;
    let pin_lcd_dc = peripherals.GPIO16; //pin 22
    // BEWARE!!! Schematic has mismatched pins!
    let pin_lcd_cs = peripherals.GPIO15; //pin 21 // BL pin on the base PCB; CS on the display PCB
    let pin_lcd_bl = peripherals.GPIO9; // RST pin on the base PCB; BL on the display PCB
    let pin_lcd_reset = peripherals.GPIO8; // CS pin on the base PCB; RST on the display PCB

    // various other pins
    let brightness_sensor_pin = peripherals.GPIO4;
    let power_off_pin = peripherals.GPIO38;

    static LOG_TOGGLES: StaticCell<LogToggleWatcher<NoopRawMutex, 6>> = StaticCell::new();
    let log_toggles = LOG_TOGGLES.init(LogToggleWatcher::new());

    // Encoder initialization
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    static APP_CORE_STACK: StaticCell<Stack<32768>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());

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

                spawner.must_spawn(update_foc(spi_bus, mag_cs, peripherals.MCPWM0, pwm_pins));
            });
        },
    );

    let serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    spawner
        .spawn(menu_handler(serial, flash, log_toggles.dyn_sender()))
        .ok();

    // LDC sensor
    let i2c_bus: I2c<'_, esp_hal::Async> = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(300)),
    )
    .unwrap()
    .with_scl(pins_i2c_scl)
    .with_sda(pins_i2c_sda)
    .into_async();
    static I2C_BUS: StaticCell<I2cBus1> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c_bus));

    let ldc_int_pin = Input::new(pins_ldc_int_pin, InputConfig::default());

    spawner.must_spawn(read_ldc_task(i2c_bus, ldc_int_pin));

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
    };

    let backlight_stuff = BacklightHandles {
        adc_instance: peripherals.ADC1,
        backlight_output: Output::new(pin_lcd_bl, Level::Low, OutputConfig::default()),
        brightness_sensor_pin,
        ledc: Ledc::new(peripherals.LEDC),
    };

    spawn_display_tasks(spawner, display_handles, backlight_stuff, log_toggles).unwrap();

    let power_off_pin = Output::new(power_off_pin, Level::High, OutputConfig::default());
    spawner.must_spawn(shutdown_handler(power_off_pin));

    info!("All tasks spawned");
    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);
}
