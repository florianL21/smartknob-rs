#[cfg(feature = "lvgl")]
pub mod lvgl;
#[cfg(feature = "slint")]
pub mod slint;

use embassy_executor::SpawnError;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use esp_hal::{
    analog::adc::{Adc, AdcCalBasic, AdcChannel, AdcConfig, AdcPin, Attenuation},
    gpio::{AnalogPin, DriveMode, Output},
    ledc::{self, LSGlobalClkSource, Ledc, LowSpeed, channel::ChannelIFace, timer::TimerIFace},
    peripherals::GPIO4,
    spi,
    time::Rate,
};
use lcd_async::options::Orientation;
use log::{error, info, warn};
use smartknob_core::system_settings::log_toggles::{LogChannel, LogToggleReceiver, may_log};
use static_cell::StaticCell;
use thiserror::Error;

const BRIGHTNESS_FADE_DURATION_MS: u16 = 1000;

pub static DISPLAY_BRIGHTNESS_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

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
    /// Pin for the brightness sensor. If not available can be set to None
    pub brightness_sensor: Option<&'static mut dyn BrightnessSensor>,
    pub backlight_output: Output<'static>,
}

#[derive(Error, Debug)]
pub enum DisplayTaskError {
    #[error("Log receiver has no more capacity. Increase the max number of log receivers")]
    LogReceiverOutOfCapacity,
    #[error("Failed to spawn at least one required task: {0}")]
    FailedToSpawnTask(#[from] SpawnError),
}

pub trait BrightnessSensor {
    fn sample(&self) -> Option<u16>;
}

// pub struct ADCBrightnessSensor<'a, ADC, PIN> {
//     adc: Adc<'a, ADC, esp_hal::Blocking>,
//     adc_pin: Option<AdcPin<PIN, ADC, AdcCalBasic<ADC>>>,
// }

// impl<'a, ADC, PIN> ADCBrightnessSensor<'a, ADC, PIN> {
//     fn new(pin: PIN) -> Self {
//         let mut adc1_config = AdcConfig::new();
//         let mut pin = adc1_config.enable_pin_with_cal::<_, AdcCalBasic<_>>(pin, Attenuation::_0dB);
//         let mut adc = Adc::new(pin, adc1_config);
//         Self { adc, adc_pin: pin }
//     }
// }

// impl<'a, ADC, PIN> BrightnessSensor for ADCBrightnessSensor<'a, ADC, PIN>
// where
//     ADC: esp_hal::analog::adc::RegisterAccess,
//     PIN: AdcChannel + AnalogPin,
// {
//     fn sample(&self) -> Option<u16> {
//         if let Some(ref mut pin) = self.adc_pin
//             && let Ok(val) = self.adc.read_oneshot(pin)
//         {
//             Some(val)
//         } else {
//             None
//         }
//     }
// }

pub struct BacklightTask<'a> {
    log_receiver: LogToggleReceiver,
    ledc_channel: ledc::channel::Channel<'a, LowSpeed>,
    current_brightness: u8,
    brightness_sensor: Option<&'static mut dyn BrightnessSensor>,
}

impl<'a> BacklightTask<'a> {
    pub fn new(handles: BacklightHandles, log_receiver: LogToggleReceiver) -> Self {
        // backlight control
        let mut ledc = handles.ledc;
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
        static TIMER: StaticCell<ledc::timer::Timer<LowSpeed>> = StaticCell::new();
        let lstimer0 = TIMER.init(ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0));
        lstimer0
            .configure(ledc::timer::config::Config {
                duty: ledc::timer::config::Duty::Duty10Bit,
                clock_source: ledc::timer::LSClockSource::APBClk,
                frequency: Rate::from_khz(24),
            })
            .unwrap();

        let mut channel0: ledc::channel::Channel<'_, LowSpeed> =
            ledc.channel(ledc::channel::Number::Channel0, handles.backlight_output);
        channel0
            .configure(ledc::channel::config::Config {
                timer: lstimer0,
                duty_pct: 0,
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

        Self {
            current_brightness: 0,
            ledc_channel: channel0,
            log_receiver,
            brightness_sensor: handles.brightness_sensor,
        }
    }

    pub async fn run(&mut self) {
        loop {
            if !self.ledc_channel.is_duty_fade_running()
                && let Some(new_brighness) = DISPLAY_BRIGHTNESS_SIGNAL.try_take()
            {
                if let Err(e) = self.ledc_channel.start_duty_fade(
                    self.current_brightness,
                    new_brighness,
                    BRIGHTNESS_FADE_DURATION_MS,
                ) {
                    warn!("Display backlight fade failed: {e:?}");
                } else {
                    self.current_brightness = new_brighness;
                }
            }
            if let Some(sensor) = &self.brightness_sensor
                && let Some(sample) = sensor.sample()
            {
                may_log(&mut self.log_receiver, LogChannel::Brightness, || {
                    info!("Brightness: {sample}")
                })
                .await;
            }
            Timer::after_millis(200).await;
        }
    }
}
