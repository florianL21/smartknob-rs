#![no_std]
#![no_main]

// mod mt6701;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    rmt::Rmt,
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::Rate,
    timer::systimer::SystemTimer,
};

// use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use log::info;
// use mt6701::AngleSensorTrait;
use smart_leds::{
    brightness,
    colors::{BLACK, RED},
    gamma, SmartLedsWrite,
};

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

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

    let mut spi_bus = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(pin_mag_clk)
    .with_miso(pin_mag_do)
    .with_cs(pin_mag_csn)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    // let spi_bus_mutex = embassy_sync::mutex::Mutex::new(spi_bus);

    // let spi_device = SpiDevice::new(&spi_bus_mutex, pin_mag_csn);

    // let sensor = mt6701::MT6701Spi::new(spi_device.into());

    info!("Embassy initialized!");

    // TODO: Spawn some tasks
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
