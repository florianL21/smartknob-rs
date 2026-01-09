#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    usb_serial_jtag::UsbSerialJtag,
};
use log::info;

use smartknob_rs::{
    cli::menu_handler,
    flash::FlashHandler,
    motor_control::{motor_driver::mcpwm::Pins6PWM, update_foc},
};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

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

    let f = FlashHandler::new(peripherals.FLASH).await;

    static FLASH: StaticCell<FlashHandler> = StaticCell::new();
    let flash = FLASH.init(f);

    // -DPIN_UH=46
    //   -DPIN_UL=2
    //   -DPIN_VH=42
    //   -DPIN_VL=39
    //   -DPIN_WH=21
    //   -DPIN_WL=38

    //   -DPIN_BUTTON_NEXT=-1
    //   -DPIN_BUTTON_PREV=-1
    //   -DPIN_LED_DATA=6
    //   -DPIN_LCD_BACKLIGHT=8

    //   -DPIN_SDA=4
    //   -DPIN_SCL=5

    //   -DPIN_MT_DATA=9
    //   -DPIN_MT_CLOCK=10
    //   -DPIN_MT_CSN=11

    // pins for TMC6300
    let pin_tmc_uh = peripherals.GPIO46;
    let pin_tmc_ul = peripherals.GPIO2;
    let pin_tmc_vh = peripherals.GPIO42;
    let pin_tmc_vl = peripherals.GPIO39;
    let pin_tmc_wh = peripherals.GPIO21;
    let pin_tmc_wl = peripherals.GPIO38;
    // let _pin_tmc_diag = peripherals.GPIO47;

    // pins for MT6701-CT
    let pin_mag_clk = peripherals.GPIO10;
    let pin_mag_do = peripherals.GPIO9;
    let pin_mag_csn = peripherals.GPIO11;
    // let _pin_mag_push = peripherals.GPIO3;

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
        flash,
    ));

    let serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();
    spawner.spawn(menu_handler(serial, flash)).ok();

    info!("All tasks spawned");
    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);
}
