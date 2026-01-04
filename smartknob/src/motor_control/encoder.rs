mod mt6701;

use embassy_time::Timer;
pub use mt6701::MT6701Spi;

use core::fmt::Debug;
use core::future;
use fixed::types::I16F16;

const ZERO_TOLERANCE: I16F16 = I16F16::lit("0.005");
const ROUGH_STEP_SIZE: I16F16 = I16F16::lit("0.04");

pub enum EncoderMagneticFieldStatus {
    Normal,
    TooStrong,
    TooWeak,
}

pub struct EncoderMeasurement {
    pub angle: I16F16,
    pub position: I16F16,
    pub magnetic_field: EncoderMagneticFieldStatus,
}

pub trait AbsolutePositionEncoder {
    type Error: Debug;

    fn update(&mut self) -> impl future::Future<Output = Result<EncoderMeasurement, Self::Error>>;
}

enum SearchDirection {
    Undetermined,
    TryFW,
    FW,
    BW,
}

pub enum EncoderLinearityMeasurement {
    Undetermined,
    WaitForZeroSettle,
    ZeroSearch {
        current_angle: I16F16,
        last_measurement: EncoderMeasurement,
        search_direction: SearchDirection,
    },
    KnownOffset(I16F16),
}

impl EncoderLinearityMeasurement {
    pub fn new() -> Self {
        EncoderLinearityMeasurement::Undetermined
    }

    pub async fn do_measurement<E: AbsolutePositionEncoder>(
        &mut self,
        encoder: &mut E,
    ) -> Result<Option<I16F16>, E::Error> {
        // match self {
        //     EncoderLinearityMeasurement::Undetermined => {
        //         *self = EncoderLinearityMeasurement::WaitForZeroSettle;
        //         Ok(Some(I16F16::ZERO))
        //     }
        //     EncoderLinearityMeasurement::WaitForZeroSettle => {
        //         Timer::after_millis(300).await;
        //         *self = EncoderLinearityMeasurement::ZeroSearch {
        //             current_angle: I16F16::ZERO,
        //             last_measurement: encoder.update().await?,
        //             search_direction: SearchDirection::Undetermined,
        //         };
        //         Ok(Some(I16F16::ZERO))
        //     }
        //     EncoderLinearityMeasurement::ZeroSearch {
        //         current_angle,
        //         last_measurement,
        //         search_direction,
        //     } => {
        //         let new_measurement = encoder.update().await?;
        //         if new_measurement.angle < ZERO_TOLERANCE {
        //             *self = EncoderLinearityMeasurement::KnownOffset(*current_angle);
        //         }
        //         match search_direction {
        //             SearchDirection::Undetermined => {
        //                 let new_angle = current_angle + ROUGH_STEP_SIZE;
        //                 *self = EncoderLinearityMeasurement::ZeroSearch {
        //                     current_angle: new_angle,
        //                     last_measurement: new_measurement,
        //                     search_direction: SearchDirection::TryFW,
        //                 };
        //                 return Ok(Some(new_angle));
        //             }
        //             SearchDirection::TryFW => {
        //                 let dir = if new_measurement > *last_angle {
        //                     SearchDirection::FW
        //                 } else {
        //                     SearchDirection::BW
        //                 };
        //                 *self = EncoderLinearityMeasurement::ZeroSearch {
        //                     current_angle: new_measurement,
        //                     last_angle: new_measurement,
        //                     search_direction: SearchDirection::BW,
        //                 };
        //             }
        //         }
        //         Ok(None)
        //     }
        // }
        Ok(None)
    }
}
