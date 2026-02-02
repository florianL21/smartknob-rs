use crate::flash::FlashHandler;
use esp_hal::{Async, usb_serial_jtag::UsbSerialJtag};
use smartknob_core::cli::CLIHandler;
use smartknob_core::system_settings::log_toggles::LogToggleSender;

#[embassy_executor::task]
pub async fn menu_handler(
    serial: UsbSerialJtag<'static, Async>,
    flash: &'static FlashHandler,
    log_toggle_sender: LogToggleSender,
) {
    let (mut rx, tx) = serial.split();

    let cli = CLIHandler::new(flash, log_toggle_sender, tx).await;
    if let Some(mut cli) = cli {
        loop {
            cli.run(&mut rx, flash).await;
        }
    }
}
