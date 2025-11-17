#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::GPIO4;
use esp_hal::spi;
// use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq, SpiTimingConfigCoreClock};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
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
use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
use log::{info, warn};
use mipidsi::asynchronous::{
    interface::SpiInterface,
    models::{Model, GC9A01},
    options::{ColorInversion, ColorOrder},
    Builder,
};
use smart_leds::{
    brightness,
    colors::{BLACK, GREEN, RED},
    gamma, SmartLedsWrite,
};
use smartknob_rs::config::{may_log, LogChannel, LOG_TOGGLES};
use smartknob_rs::flash::{FlashHandler, FlashType, RestoredState};
use smartknob_rs::motor_control::ENCODER_ANGLE;
use smartknob_rs::shutdown::shutdown_handler;
use smartknob_rs::{
    cli::menu_handler,
    knob_tilt::KnobTiltEvent,
    motor_control::{update_foc, ENCODER_POSITION},
};
use smartknob_rs::{knob_tilt::read_ldc_task, motor_control::Pins6PWM};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

type EncoderSpiBus = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;
type DisplaySpiBus = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;
type I2cBus1 = Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;

const ENCODER_SPI_DMA_BUFFER_SIZE: usize = 200;
const DISPLAY_SPI_DMA_BUFFER_SIZE: usize = 32000;

#[embassy_executor::task]
async fn log_rotations() {
    let mut log_receiver = LOG_TOGGLES
        .receiver()
        .expect("Log toggle receiver had not enough capacity");
    info!("Log encoder init done!");
    loop {
        let pos = ENCODER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
        let angle = ENCODER_ANGLE.load(core::sync::atomic::Ordering::Relaxed);
        may_log(&mut log_receiver, LogChannel::Encoder, || {
            info!("Position: {pos}; Angle: {angle}")
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
) {
    const NUM_LEDS: usize = 24;
    const _LED_OFFSET: usize = 1;

    let mut log_receiver = LOG_TOGGLES
        .receiver()
        .expect("Log toggle receiver had not enough capacity");

    let mut rmt_buffer = smart_led_buffer!(NUM_LEDS);
    let mut led = SmartLedsAdapter::new(rmt_channel, led_pin, &mut rmt_buffer);

    let mut data = [RED; NUM_LEDS];
    let _step_size = (2.0f32 * core::f32::consts::PI) / NUM_LEDS as f32;
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
    // let psram_config = PsramConfig {
    //     flash_frequency: FlashFreq::FlashFreq120m,
    //     ram_frequency: SpiRamFreq::Freq120m,
    //     ..Default::default()
    // };

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    // .with_psram(psram_config);
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(timer0.alarm0);
    info!("Embassy initialized!");

    let f = FlashHandler::new(peripherals.FLASH).await;
    let restored_state = match f.restore().await {
        Ok(state) => {
            info!("Restored previous state from flash: {:#?}", state);
            state
        }
        Err(e) => {
            warn!("Failed to restore state from flash: {}", e);
            warn!("Assuming first startup. No calibration data was loaded");
            RestoredState::default()
        }
    };

    static FLASH: StaticCell<FlashType<'static>> = StaticCell::new();
    let flash = FLASH.init(f.eject());

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

    // Encoder initialization
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_buffers!(ENCODER_SPI_DMA_BUFFER_SIZE);
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

    static ENCODER_SPI_BUS: StaticCell<EncoderSpiBus> = StaticCell::new();
    let spi_bus = ENCODER_SPI_BUS.init(Mutex::new(spi_bus));

    let mag_cs = Output::new(pin_mag_csn, Level::Low, OutputConfig::default());
    let pwm_pins = Pins6PWM {
        uh: pin_tmc_uh.into(),
        ul: pin_tmc_ul.into(),
        vh: pin_tmc_vh.into(),
        vl: pin_tmc_vl.into(),
        wh: pin_tmc_wh.into(),
        wl: pin_tmc_wl.into(),
    };

    spawner.must_spawn(update_foc(
        spi_bus,
        mag_cs,
        peripherals.MCPWM0,
        pwm_pins,
        flash,
        restored_state.motor_alignment,
    ));

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
    spawner.must_spawn(log_rotations());

    // LED ring
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let receiver = TILT.receiver().unwrap();
    spawner.must_spawn(led_ring(rmt.channel0, pin_led_data.into(), receiver));

    let serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    spawner
        .spawn(menu_handler(serial, flash, restored_state.log_channels))
        .ok();

    // Display
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_buffers!(DISPLAY_SPI_DMA_BUFFER_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi_bus = Spi::new(
        peripherals.SPI3,
        spi::master::Config::default()
            .with_frequency(Rate::from_mhz(80))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(pin_lcd_sck)
    .with_mosi(pin_lcd_mosi)
    .with_dma(peripherals.DMA_CH1)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let lcd_cs = Output::new(pin_lcd_cs, Level::Low, OutputConfig::default());
    let lcd_dc = Output::new(pin_lcd_dc, Level::Low, OutputConfig::default());
    let lcd_bl = Output::new(pin_lcd_bl, Level::Low, OutputConfig::default());
    let lcd_rs = Output::new(pin_lcd_reset, Level::Low, OutputConfig::default());

    static DISPLAY_SPI_BUS: StaticCell<DisplaySpiBus> = StaticCell::new();
    let spi_bus = DISPLAY_SPI_BUS.init(Mutex::new(spi_bus));
    let ledc = Ledc::new(peripherals.LEDC);

    spawner.must_spawn(display_task(spi_bus, lcd_cs, lcd_dc, lcd_bl, lcd_rs, ledc));

    spawner.must_spawn(brightness_task(peripherals.ADC1, brightness_sensor_pin));

    let power_off_pin = Output::new(power_off_pin, Level::High, OutputConfig::default());
    spawner.must_spawn(shutdown_handler(power_off_pin));

    info!("All tasks spawned");
    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);
}

#[embassy_executor::task]
async fn brightness_task(adc_per: esp_hal::peripherals::ADC1<'static>, sensor_pin: GPIO4<'static>) {
    let mut log_receiver = LOG_TOGGLES
        .receiver()
        .expect("Log toggle receiver had not enough capacity");

    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(sensor_pin, Attenuation::_11dB);
    let mut adc1 = Adc::new(adc_per, adc1_config);
    loop {
        if let Ok(val) = adc1.read_oneshot(&mut pin) {
            may_log(&mut log_receiver, LogChannel::Brightness, || {
                info!("Brightness: {val}")
            })
            .await;
        }
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
pub async fn display_task(
    spi_bus: &'static DisplaySpiBus,
    lcd_cs: Output<'static>,
    dc_output: Output<'static>,
    mut backlight_output: Output<'static>,
    reset_output: Output<'static>,
    mut ledc: Ledc<'static>,
) {
    let device = SpiDevice::new(spi_bus, lcd_cs);
    const DISPLAY_SIZE: (u16, u16) = GC9A01::FRAMEBUFFER_SIZE;

    let mut delay = Delay {};
    let mut buffer = [0u8; DISPLAY_SPI_DMA_BUFFER_SIZE];
    let di = SpiInterface::new(device, dc_output, &mut buffer);
    let mut display = Builder::new(GC9A01, di)
        .reset_pin(reset_output)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .await
        .unwrap();

    // backlight control

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty10Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(ledc::channel::Number::Channel0, backlight_output);
    channel0
        .configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    if let Err(e) = channel0.start_duty_fade(0, 50, 1000) {
        warn!("Display backlight fade failed: {e:?}");
    }
    // brightness sensor
    loop {
        let single_color = [Rgb565::RED].into_iter().cycle();
        display
            .set_pixels(0, 0, DISPLAY_SIZE.0 - 1, DISPLAY_SIZE.1 - 1, single_color)
            .await
            .ok();
        Timer::after_millis(300).await;
        let single_color = [Rgb565::BLACK].into_iter().cycle();
        display
            .set_pixels(0, 0, DISPLAY_SIZE.0 - 1, DISPLAY_SIZE.1 - 1, single_color)
            .await
            .ok();
        Timer::after_millis(300).await;
        warn!("iter");
        // Timer::after_millis(2000).await;
        // if let Err(e) = channel0.start_duty_fade(100, 0, 1000) {
        //     warn!("Display backlight fade failed: {e:?}");
        // }
        // Timer::after_millis(2000).await;
        // if let Err(e) = channel0.start_duty_fade(0, 100, 1000) {
        //     warn!("Display backlight fade failed: {e:?}");
        // }
    }
}
