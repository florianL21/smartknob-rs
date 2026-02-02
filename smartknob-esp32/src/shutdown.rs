use esp_hal::gpio::Output;
use smartknob_core::shutdown::shutdown_handler;

#[embassy_executor::task]
pub async fn shutdown_task(power_off_pin: Output<'static>, high_is_off: bool) {
    shutdown_handler(power_off_pin, high_is_off).await;
}
