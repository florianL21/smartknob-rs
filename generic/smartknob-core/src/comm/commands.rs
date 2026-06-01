use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use crate::{comm::EmbeddedError, system_settings::log_toggles::LogChannel};

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Command {
    /// Initiate a system shutdown
    Shutdown,
    /// Enable/Disable a logging channel
    Log {
        channel: LogChannel,
        enabled: bool,
    },
    /// Format the whole flash
    FlashErase,
    /// Run full motor calibration sequence
    MotorCalibrate,
    // Measure the encoder for non-linearity and give a verdict if the error is too big for proper operation
    EncoderValidate,
    /// offset the current motor calibration by the given value
    MotorTune(f32),
    /// Save the modified current electrical angle offset to flash
    MotorTuneStore,
    /// Make a beep sound
    Beep {
        /// Frequency of the beep
        freq: f32,
        /// duration in ms
        duration: u64,
        /// volume in % between 0 and 100
        volume: f32,
        /// Note offsetfreq
        note_offset: u32,
    },
    /// Set the backlight brightness of the display
    Brightness {
        /// Brightness in percent 0-100
        percent: u8,
    },
    /// Used to test system if the communication is working
    Ping,
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Response {
    /// Sent if operation was completed successfully
    Ack,
    /// Sent when there was a framing error. Simply repeating the mesage could likeley fix the issue in this case
    Repeat,
    /// Sent when there was an error with executing the operation
    Error(EmbeddedError),
}
