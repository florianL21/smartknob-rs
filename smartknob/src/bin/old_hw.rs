#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::{DataPartitionSubType, PartitionType};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq};
use esp_hal::system::Stack;
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
use esp_rtos::embassy::Executor;
use log::info;

use smartknob_core::flash::FlashHandling;
use smartknob_core::system_settings::log_toggles::LogToggleWatcher;
use smartknob_core::system_settings::{HapticSystemStoreSignal, StoreSignals};
use smartknob_esp32::flash::FlashHandler;
use smartknob_esp32::motor_driver::mcpwm::Pins6PWM;
use smartknob_rs::{cli::menu_handler, motor_control::update_foc};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

type LogWatcher = LogToggleWatcher<CriticalSectionRawMutex, 6>;

#[embassy_executor::task]
pub async fn flash_task(
    flash_handler: &'static FlashHandler,
    store_signals: &'static StoreSignals<CriticalSectionRawMutex>,
) {
    loop {
        flash_handler.run(store_signals).await;
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

    static LOG_TOGGLES: StaticCell<LogWatcher> = StaticCell::new();
    let log_toggles = LOG_TOGGLES.init(LogToggleWatcher::new());

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

    info!("All tasks spawned");
    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);
}
