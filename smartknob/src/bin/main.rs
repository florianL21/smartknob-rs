#![no_std]
#![no_main]

extern crate alloc;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{GpioPin, Level, Output, OutputConfig},
    rmt::Rmt,
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::Rate,
    timer::systimer::SystemTimer,
};
use mt6701::{self, AngleSensorTrait};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use log::info;
use smart_leds::{
    brightness,
    colors::{BLACK, RED},
    gamma, SmartLedsWrite,
};

#[embassy_executor::task]
async fn log_angle(
    spi_bus: esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>,
    pin_mag_csn: GpioPin<12>,
) {
    let spi_bus_mutex: Mutex<
        NoopRawMutex,
        esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>,
    > = embassy_sync::mutex::Mutex::new(spi_bus);

    let spi_device = SpiDevice::new(
        &spi_bus_mutex,
        Output::new(pin_mag_csn, Level::Low, OutputConfig::default()),
    );

    let mut encoder: mt6701::MT6701Spi<
        SpiDevice<
            '_,
            NoopRawMutex,
            esp_hal::spi::master::SpiDmaBus<'_, esp_hal::Async>,
            Output<'_>,
        >,
    > = mt6701::MT6701Spi::new(spi_device);
    loop {
        if encoder.update(0).await.is_ok() {
            let pos = encoder.get_position();
            info!("pos: {}", pos);
        }
        Timer::after_millis(200).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);
    info!("Embassy initialized!");

    // Pins for LDC1614
    // let pins_ldc_int_pin = peripherals.GPIO40;
    // let pins_i2c_scl = peripherals.GPIO42;
    // let pins_i2c_sda = peripherals.GPIO41;

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

    // initialize various peripherals
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt_buffer = smartLedBuffer!(24);
    const NUM_LEDS: usize = 24;
    let mut led = SmartLedsAdapter::new(rmt.channel0, pin_led_data, rmt_buffer);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi_bus: esp_hal::spi::master::SpiDmaBus<'_, esp_hal::Async> = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(pin_mag_clk)
    .with_miso(pin_mag_do)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    // TODO: Spawn some tasks
    spawner.spawn(log_angle(spi_bus, pin_mag_csn)).unwrap();
    let _ = spawner;

    let mut data;

    loop {
        Timer::after(Duration::from_secs(1)).await;
        data = [RED; NUM_LEDS];
        led.write(brightness(gamma(data.iter().cloned()), 10))
            .unwrap();
        Timer::after(Duration::from_secs(1)).await;
        data = [BLACK; NUM_LEDS];
        led.write(brightness(gamma(data.iter().cloned()), 10))
            .unwrap();
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}
