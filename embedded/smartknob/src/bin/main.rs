#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, time::Rate};
use log::info;
use smartknob_core::haptics::{CurveBuilder, CurveSegment, DetailedSettings, set_curve_config};
use smartknob_core::{
    haptics::get_encoder_position,
    system_settings::log_toggles::{LogChannel, LogToggleReceiver, may_log},
};
use smartknob_esp32::building_blocks::{
    Peripherals, SmartknobSystemConfig, SystemPins,
    button::{LDCButtons, LDCPins},
    display::{DisplayPins, SPIDisplay},
    foc::{EncoderPins, FOCMT6701With6PWM},
    led::WS2812LedRingForPushEvents,
    shutdown::PinBasedShutdown,
    smartknob_main,
};
use smartknob_esp32::{display::Orientation, motor_driver::mcpwm::Pins6PWM};

esp_bootloader_esp_idf::esp_app_desc!();

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

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // For now only enable this for debugging early startup issues or problems where a PC connection cannot yet be established.
    // Also keep in mind that this is exclusive with the logger_task in uplink.rs
    // esp_println::logger::init_logger_from_env();
    let psram_config = PsramConfig {
        flash_frequency: FlashFreq::FlashFreq80m,
        ram_frequency: SpiRamFreq::Freq80m,
        mode: esp_hal::psram::PsramMode::OctalSpi,
        // core_clock: Some(SpiTimingConfigCoreClock::SpiTimingConfigCoreClock240m),
        ..Default::default()
    };

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_alloc::heap_allocator!(size: 72 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram, psram_config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
    info!("Embassy initialized!");

    // so far unused pins for MT6701-CT
    let _pin_mag_push = peripherals.GPIO3;

    // unused pins for TMC6300
    let _pin_tmc_diag = peripherals.GPIO47;

    let system_config = SmartknobSystemConfig {
        peripherals: Peripherals {
            cpu_ctrl: peripherals.CPU_CTRL,
            flash: peripherals.FLASH,
            usb: peripherals.USB0,
            usb_device: peripherals.USB_DEVICE,
        },
        sw_interrupt: sw_int.software_interrupt1,
        pins: SystemPins {
            usb_plus: peripherals.GPIO20,
            usb_minus: peripherals.GPIO19,
        },
    };

    let pin_tmc_uh = peripherals.GPIO48;
    let pin_tmc_ul = peripherals.GPIO17;
    let pin_tmc_vh = peripherals.GPIO21;
    let pin_tmc_vl = peripherals.GPIO46;
    let pin_tmc_wh = peripherals.GPIO18;
    let pin_tmc_wl = peripherals.GPIO45;

    let foc = FOCMT6701With6PWM {
        spi: peripherals.SPI2,
        mcpwm_channel: peripherals.MCPWM0,
        dma_channel: peripherals.DMA_CH0,
        foc_refresh_rate: None,
        haptic_settings: DetailedSettings::default(),
        spi_frequency: Rate::from_mhz(5),
        pwm_pins: Pins6PWM::new(
            pin_tmc_uh.into(),
            pin_tmc_ul.into(),
            pin_tmc_vh.into(),
            pin_tmc_vl.into(),
            pin_tmc_wh.into(),
            pin_tmc_wl.into(),
        ),
        encoder_pins: EncoderPins {
            mag_clk: peripherals.GPIO11,
            mag_do: peripherals.GPIO10,
            mag_csn: peripherals.GPIO12,
        },
    };

    let display = SPIDisplay {
        spi_peripheral: peripherals.SPI3,
        ledc_peripheral: peripherals.LEDC,
        dma_channel: peripherals.DMA_CH1,
        brightness_sensor: None,
        display_orientation: Orientation::new(),
        display_pins: DisplayPins {
            lcd_sck: peripherals.GPIO13,
            lcd_mosi: peripherals.GPIO14,
            lcd_dc: peripherals.GPIO16, //pin 22
            // BEWARE!!! Schematic has mismatched pins!
            lcd_cs: peripherals.GPIO15, //pin 21 // BL pin on the base PCB; CS on the display PCB
            lcd_bl: peripherals.GPIO9,  // RST pin on the base PCB; BL on the display PCB
            lcd_rst: peripherals.GPIO8, // CS pin on the base PCB; RST on the display PCB
        },
    };

    let buttons = LDCButtons {
        i2c_peripheral: peripherals.I2C0,
        i2c_freq: Rate::from_khz(300),
        pins: LDCPins {
            i2c_scl: peripherals.GPIO42,
            i2c_sda: peripherals.GPIO41,
            ldc_int: peripherals.GPIO40,
        },
    };

    let led_ring = WS2812LedRingForPushEvents {
        pin_leds: peripherals.GPIO39.into(),
        rmt_frequency: Rate::from_mhz(80),
        rmt_peripheral: peripherals.RMT,
    };

    let shutdown = PinBasedShutdown {
        high_is_off: true,
        shutdown_pin: peripherals.GPIO38,
    };

    let mut curve_builder = CurveBuilder::new();
    let detent = curve_builder.new_segment(
        CurveSegment::new()
            .add_bezier3(0.2, [0.0, 0.0, -0.7])
            .add_bezier3(0.2, [0.7, 0.0, 0.0]),
    );
    let zero = curve_builder.new_segment(CurveSegment::new().add_const(1.0, 0.0));
    let detent_curve = curve_builder
        .push(zero)
        .push_repeated(detent, 25)
        .finish(-2.0)
        .without_pattern_layer();

    set_curve_config(detent_curve);

    smartknob_main(
        spawner,
        foc,
        system_config,
        buttons,
        led_ring,
        display,
        shutdown,
    )
    .await;
}
