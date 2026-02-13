mod mt6701;

pub use mt6701::MT6701Spi;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use core::error::Error;
use core::fmt::Debug;
use core::future;
use fixed::types::I16F16;

pub trait EncoderError: Error + Debug {}

#[derive(Debug)]
pub enum EncoderMagneticFieldStatus {
    Normal,
    TooStrong,
    TooWeak,
}

#[derive(Debug)]
pub struct EncoderMeasurement {
    pub angle: I16F16,
    pub position: I16F16,
    pub magnetic_field: EncoderMagneticFieldStatus,
}

#[derive(Debug, PartialEq, Copy, Clone, Deserialize, Serialize, MaxSize)]
pub enum EncoderDirection {
    CW,
    CCW,
}

pub trait AbsolutePositionEncoder {
    type Error: EncoderError;

    fn update(&mut self) -> impl future::Future<Output = Result<EncoderMeasurement, Self::Error>>;

    /// Specify the direction of the encoder readings
    fn set_direction(&mut self, direction: EncoderDirection);

    /// Return the currently set direction of the encoder readings
    /// Returns None if the direction has not been set yet
    fn get_direction(&mut self) -> Option<EncoderDirection>;
}
