#![cfg_attr(not(feature = "host"), no_std)]

#[cfg(feature = "host")]
use clap::ValueEnum;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const VID: u16 = 0x303A;
pub const PID: u16 = 0x3001;
pub const MANUFACTURER: &'static str = "FlorianL21";
pub const PRODUCT: &'static str = "smartknob-rs";
pub const SERIAL: &'static str = "12345678";

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
#[cfg_attr(feature = "host", derive(ValueEnum))]
pub enum LogChannel {
    Encoder,
    DisplayTransfer,
    DisplayRender,
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Command {
    LogEnable(LogChannel),
    LogDisable(LogChannel),
    FlashErase,
    MotorCalibrate,
    Ping,
}

#[derive(Deserialize, Serialize, Error, Debug, Clone)]
pub enum EmbeddedError {
    #[error("Postcard deserialize failed")]
    PostcardDecodeError(#[from] postcard::Error),
}

impl MaxSize for EmbeddedError {
    // Wild guess. TODO: check this holds true
    const POSTCARD_MAX_SIZE: usize = 2;
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

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Comm {
    Response(Response),
    Event(Event),
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Event {
    EncoderAngle(f32),
    Button(ButtonEvent),
    Tilt { angle: f32, magnitude: f32 },
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum ButtonEvent {
    PressDown,
    PressUp,
    TiltStart(Direction),
    TiltStop,
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
}
