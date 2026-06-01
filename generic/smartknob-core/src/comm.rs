pub mod commands;
pub mod events;

use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub use crate::comm::{commands::Command, commands::Response, events::Event};
pub use crate::haptics::base::ConfigError;
pub use crate::system_settings::log_toggles::LogChannel;

pub const VID: u16 = 0x303A;
pub const PID: u16 = 0x3001;
pub const MANUFACTURER: &str = "FlorianL21";
pub const PRODUCT: &str = "smartknob-rs";
pub const SERIAL: &str = "12345678";

pub const COMMAND_BUFFER_SIZE: usize = 2048;

#[derive(Deserialize, Serialize, Error, Debug, Clone)]
pub enum EmbeddedError {
    #[error("Postcard deserialize failed")]
    PostcardDecodeError(#[from] postcard::Error),
    #[error("Flash error")]
    FlashError,
    #[error("Encountered encoder error")]
    EncoderError,
    #[error("Failed to initialize haptic curve")]
    FailedToIntantiate(#[from] ConfigError),
}

impl MaxSize for EmbeddedError {
    // Wild guess. TODO: check this holds true
    const POSTCARD_MAX_SIZE: usize = 2;
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Comm {
    Response(Response),
    Event(Event),
}
