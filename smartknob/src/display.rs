#[cfg(feature = "lvgl")]
pub mod lvgl;
#[cfg(feature = "slint")]
pub mod slint;

use crate::signals::DISPLAY_BRIGHTNESS_SIGNAL;
use embassy_executor::SpawnError;
use embassy_time::Timer;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::spi;
use esp_hal::{gpio::Output, time::Rate};
use lcd_async::options::Orientation;
use log::{error, warn};
use smartknob_core::system_settings::log_toggles::LogToggleReceiver;
use thiserror::Error;

const BRIGHTNESS_FADE_DURATION_MS: u16 = 1000;

pub type DisplaySpiBus = spi::master::SpiDma<'static, esp_hal::Blocking>;

pub struct DisplayHandles {
    pub spi_bus: DisplaySpiBus,
    pub lcd_cs: Output<'static>,
    pub dc_output: Output<'static>,

    pub reset_output: Output<'static>,
    pub orientation: Orientation,
}

pub struct BacklightHandles {
    pub ledc: Ledc<'static>,
    pub adc_instance: esp_hal::peripherals::ADC1<'static>,
    // pub brightness_sensor_pin: GPIO4<'static>,
    pub backlight_output: Output<'static>,
}

#[derive(Error, Debug)]
pub enum DisplayTaskError {
    #[error("Log receiver has no more capacity. Increase the max number of log receivers")]
    LogReceiverOutOfCapacity,
    #[error("Failed to spawn at least one required task: {0}")]
    FailedToSpawnTask(#[from] SpawnError),
}

#[embassy_executor::task]
async fn brightness_task(handles: BacklightHandles, mut _log_receiver: LogToggleReceiver) {
    let mut current_brightness = 0;

    // backlight control
    let mut ledc = handles.ledc;
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty10Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(ledc::channel::Number::Channel0, handles.backlight_output);
    channel0
        .configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: current_brightness,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    loop {
        if !channel0.is_duty_fade_running()
            && let Some(new_brighness) = DISPLAY_BRIGHTNESS_SIGNAL.try_take()
        {
            if let Err(e) = channel0.start_duty_fade(
                current_brightness,
                new_brighness,
                BRIGHTNESS_FADE_DURATION_MS,
            ) {
                warn!("Display backlight fade failed: {e:?}");
            } else {
                current_brightness = new_brighness;
            }
        }
        Timer::after_millis(200).await;
    }
}
