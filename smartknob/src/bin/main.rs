#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    rmt::Rmt,
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
    timer::systimer::SystemTimer,
    usb_serial_jtag::UsbSerialJtag,
};
use esp_hal_smartled::{buffer_size, SmartLedsAdapter};
use log::info;
use smart_leds::{
    brightness,
    colors::{BLACK, GREEN, RED},
    gamma, SmartLedsWrite,
};
use smartknob_rs::flash::flash_init;
use smartknob_rs::{
    cli::{may_log, menu_handler, LogChannel, LogToggles},
    knob_tilt::KnobTiltEvent,
    motor_control::{update_foc, ENCODER_POSITION},
};
use smartknob_rs::{knob_tilt::read_ldc_task, motor_control::Pins6PWM};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;
type I2cBus1 = Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;

#[embassy_executor::task]
async fn log_rotations(
    mut log_receiver: embassy_sync::watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        LogToggles,
        4,
    >,
) {
    info!("Log encoder init done!");
    loop {
        let pos = ENCODER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
        may_log(&mut log_receiver, LogChannel::Encoder, || {
            info!("Position: {}", pos)
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
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#[embassy_executor::task]
async fn led_ring(
    rmt_channel: esp_hal::rmt::ChannelCreator<'static, esp_hal::Blocking, 0>,
    led_pin: AnyPin<'static>,
    mut receiver: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, KnobTiltEvent, 2>,
    mut log_receiver: embassy_sync::watch::Receiver<
        'static,
        CriticalSectionRawMutex,
        LogToggles,
        4,
    >,
) {
    const NUM_LEDS: usize = 24;
    const LED_OFFSET: usize = 1;
    const BUFFE_SIZE: usize = buffer_size(NUM_LEDS);
    let rmt_buffer: [u32; BUFFE_SIZE] = [0; BUFFE_SIZE];
    let mut led = SmartLedsAdapter::new(rmt_channel, led_pin, rmt_buffer);

    let mut data = [RED; NUM_LEDS];
    let step_size = (2.0f32 * core::f32::consts::PI) / NUM_LEDS as f32;
    info!("LED init done!");
    loop {
        let tilt_event = receiver.changed().await;
        may_log(&mut log_receiver, LogChannel::PushEvents, || {
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
            KnobTiltEvent::TiltAdjust(tilt) | KnobTiltEvent::TiltStart(tilt) => {
                let angle = if tilt.angle < 0.0 {
                    tilt.angle + 2.0 * core::f32::consts::PI
                } else {
                    tilt.angle
                };
                let led_index = map(
                    angle,
                    0.0,
                    2.0 * core::f32::consts::PI,
                    0.0,
                    NUM_LEDS as f32,
                );
                // not yet working
                for (i, item) in data.iter_mut().enumerate() {
                    if i == led_index as usize {
                        *item = GREEN;
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

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(timer0.alarm0);
    info!("Embassy initialized!");

    // watcher for log output toggles
    static LOG_WATCH: Watch<CriticalSectionRawMutex, LogToggles, 4> = Watch::new();

    // Pins for LDC1614
    let pins_ldc_int_pin = peripherals.GPIO40;
    let pins_i2c_scl = peripherals.GPIO42;
    let pins_i2c_sda = peripherals.GPIO41;

    // Pins for WS2812B LEDs
    let pin_led_data = peripherals.GPIO39;

    // pins for TMC6300
    // let pin_tmc_diag = peripherals.GPIO47;
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
    // let pin_mag_push = peripherals.GPIO3;

    // Encoder initialization
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi_bus = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(pin_mag_clk)
    .with_miso(pin_mag_do)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    static SPI_BUS: StaticCell<SpiBus1> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi_bus));

    let mag_cs = Output::new(pin_mag_csn, Level::Low, OutputConfig::default());
    let pwm_pins = Pins6PWM {
        uh: pin_tmc_uh.into(),
        ul: pin_tmc_ul.into(),
        vh: pin_tmc_vh.into(),
        vl: pin_tmc_vl.into(),
        wh: pin_tmc_wh.into(),
        wl: pin_tmc_wl.into(),
    };

    spawner.must_spawn(update_foc(spi_bus, mag_cs, peripherals.MCPWM0, pwm_pins));

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
    static TILT: Watch<CriticalSectionRawMutex, KnobTiltEvent, 2> = Watch::new();
    let sender = TILT.sender();

    spawner.must_spawn(read_ldc_task(i2c_bus, ldc_int_pin, sender));

    // log encoder values
    let log_receiver = LOG_WATCH.receiver().unwrap();
    spawner.must_spawn(log_rotations(log_receiver));

    // LED ring
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let receiver = TILT.receiver().unwrap();
    let log_receiver = LOG_WATCH.receiver().unwrap();
    spawner.must_spawn(led_ring(
        rmt.channel0,
        pin_led_data.into(),
        receiver,
        log_receiver,
    ));
    info!("Startup done!");

    let f = flash_init(peripherals.FLASH).await;

    let serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    let sender = LOG_WATCH.sender();
    let _ = spawner.spawn(menu_handler(serial, sender, f));
}
