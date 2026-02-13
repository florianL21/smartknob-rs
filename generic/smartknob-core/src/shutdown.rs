use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_hal::digital::OutputPin;
use log::info;

pub static REQUEST_POWER_DOWN: Signal<CriticalSectionRawMutex, bool> = Signal::new();

/// This shutdown handler will shutdown a board which can disconnect its own power supply by pulling a GPIO pin high or low.
/// # Arguments
/// * `power_off_pin`: The pin which should be pulled high or low
/// * `high_is_off`: `true` if the pin should be pulled high to turn off the power supply
///
/// The shutdown handler is to be called inside an embassy task.
/// It will consume the [`REQUEST_POWER_DOWN`] signal.
pub async fn shutdown_handler(mut power_off_pin: impl OutputPin, high_is_off: bool) -> ! {
    let _ = if high_is_off {
        power_off_pin.set_low()
    } else {
        power_off_pin.set_high()
    };
    loop {
        if REQUEST_POWER_DOWN.wait().await {
            info!("Shutdown requested. Powering off...");
            let _ = if high_is_off {
                power_off_pin.set_high()
            } else {
                power_off_pin.set_low()
            };
        }
    }
}
