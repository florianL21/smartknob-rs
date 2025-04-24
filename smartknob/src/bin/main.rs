#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{GpioPin, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    rmt::Rmt,
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
    timer::systimer::SystemTimer,
};
use mt6701::{self, AngleSensorTrait};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use esp_hal_smartled::{buffer_size, SmartLedsAdapter};
use log::info;
use smart_leds::{
    brightness,
    colors::{BLACK, RED},
    gamma, SmartLedsWrite,
};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

use static_cell::StaticCell;

use smartknob_rs::knob_tilt::read_ldc_task;

type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;
type I2cBus1 = Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;

#[derive(Clone)]
struct Encoder {
    pub position: f64,
    pub angle: f32,
}

#[embassy_executor::task]
async fn read_encoder(
    spi_bus: &'static SpiBus1,
    mag_csn: Output<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, Encoder, 2>,
) {
    let spi_device = SpiDevice::new(spi_bus, mag_csn);

    let mut encoder: mt6701::MT6701Spi<
        SpiDevice<
            '_,
            NoopRawMutex,
            esp_hal::spi::master::SpiDmaBus<'_, esp_hal::Async>,
            Output<'_>,
        >,
    > = mt6701::MT6701Spi::new(spi_device);
    let mut t = Instant::now();
    info!("encoder init done!");
    loop {
        if encoder.update(t.elapsed().into()).await.is_ok() {
            t = Instant::now();
            let pos = encoder.get_position();
            let angle = encoder.get_angle();
            sender.send(Encoder {
                position: pos,
                angle: angle,
            });
        }
        Timer::after_millis(2).await;
    }
}

#[embassy_executor::task]
async fn log_rotations(
    mut receiver: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, Encoder, 2>,
) {
    info!("Log encoder init done!");
    loop {
        let pos = receiver.get().await;
        info!("Position: {}", pos.position);
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn led_ring(
    rmt_channel: esp_hal::rmt::ChannelCreator<esp_hal::Blocking, 0>,
    led_pin: GpioPin<39>,
    mut receiver: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, Encoder, 2>,
) {
    const NUM_LEDS: usize = 24;
    const BUFFE_SIZE: usize = buffer_size(NUM_LEDS);
    let rmt_buffer: [u32; BUFFE_SIZE] = [0; BUFFE_SIZE];
    let mut led = SmartLedsAdapter::new(rmt_channel, led_pin, rmt_buffer);

    let mut data = [RED; NUM_LEDS];
    let step_size = (2.0f32 * core::f32::consts::PI) / NUM_LEDS as f32;
    info!("LED init done!");
    loop {
        let encoder = receiver.get().await;
        for (i, item) in data.iter_mut().enumerate() {
            if encoder.angle > step_size * i as f32 {
                *item = RED;
            } else {
                *item = BLACK;
            }
        }
        led.write(brightness(gamma(data.iter().cloned()), 10))
            .unwrap();
        Timer::after(Duration::from_millis(20)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    info!("Embassy initialized!");

    // Pins for LDC1614
    // let pins_ldc_int_pin = peripherals.GPIO40;
    let pins_i2c_scl = peripherals.GPIO42;
    let pins_i2c_sda = peripherals.GPIO41;

    // Pins for WS2812B LEDs
    let pin_led_data = peripherals.GPIO39;

    // pins for TMC6300
    // let pin_tmc_diag = peripherals.GPIO47;
    // let pin_tmc_uh = peripherals.GPIO48;
    // let pin_tmc_ul = peripherals.GPIO17;
    // let pin_tmc_vh = peripherals.GPIO21;
    // let pin_tmc_vl = peripherals.GPIO46;
    // let pin_tmc_wh = peripherals.GPIO18;
    // let pin_tmc_wl = peripherals.GPIO45;

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

    static WATCH: Watch<CriticalSectionRawMutex, Encoder, 2> = Watch::new();
    let sender = WATCH.sender();

    let mag_cs = Output::new(pin_mag_csn, Level::Low, OutputConfig::default());
    spawner.must_spawn(read_encoder(spi_bus, mag_cs, sender));

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

    spawner.must_spawn(read_ldc_task(i2c_bus));

    // log encoder values
    // let receiver = WATCH.receiver().unwrap();
    // spawner.must_spawn(log_rotations(receiver));

    // LED ring
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .unwrap();
    let receiver = WATCH.receiver().unwrap();
    spawner.must_spawn(led_ring(rmt.channel0, pin_led_data, receiver));
    info!("Startup done!");
}
