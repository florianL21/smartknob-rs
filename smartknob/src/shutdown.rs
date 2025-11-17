use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::gpio::Output;
use log::info;

pub static REQUEST_POWER_DOWN: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
pub async fn shutdown_handler(mut power_off_pin: Output<'static>) {
    power_off_pin.set_high();
    loop {
        if REQUEST_POWER_DOWN.wait().await {
            info!("Shutdown requested. Powering off...");
            power_off_pin.set_low();
        }
    }
}
