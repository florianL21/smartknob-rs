use super::ShutdownBlock;

use embassy_sync::blocking_mutex::raw::RawMutex;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use smartknob_core::{shutdown::shutdown_handler, system_settings::log_toggles::LogToggleWatcher};

/// Pre-made building block where a full system shutdown can be requested by setting a GPIO
/// pin to a certain level. This is for example useful if the power rail of the whole
/// system can be shutdown on request of the uC
pub struct PinBasedShutdown<P: OutputPin + 'static> {
    /// Pin to pull when a shutdown was requested
    pub shutdown_pin: P,
    /// Whether the system is going to shut down when the pin is pulled high
    pub high_is_off: bool,
}

impl<P: OutputPin + 'static> ShutdownBlock for PinBasedShutdown<P> {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: embassy_executor::Spawner,
        _: &'static LogToggleWatcher<M, N>,
    ) {
        let initial = if self.high_is_off {
            Level::Low
        } else {
            Level::High
        };
        let power_off_pin = Output::new(self.shutdown_pin, initial, OutputConfig::default());
        spawner.spawn(shutdown_task(power_off_pin, self.high_is_off).unwrap());
    }
}

#[embassy_executor::task]
pub async fn shutdown_task(power_off_pin: Output<'static>, high_is_off: bool) {
    shutdown_handler(power_off_pin, high_is_off).await;
}
