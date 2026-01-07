use core::fmt::Debug;
use embedded_hal_async::spi::{Operation, SpiDevice};
use fixed::types::I16F16;
use thiserror::Error;

use super::AbsolutePositionEncoder;
use crate::motor_control::encoder::{
    EncoderDirection, EncoderError, EncoderMagneticFieldStatus, EncoderMeasurement,
};

const OVERFLOW_LOW_BAND: u16 = 1600;
const OVERFLOW_HIGH_BAND: u16 = (2u16.pow(14)) - OVERFLOW_LOW_BAND;

const ENCODER_MAGIC: I16F16 = I16F16::lit("16384");

#[derive(Debug, Error)]
pub enum MT6701Error {
    #[error("SPI communication error")]
    SpiError,
    #[error("CSN pin error")]
    CsnError,
}

impl EncoderError for MT6701Error {}

#[derive(Debug)]
pub struct MT6701Spi<SPI> {
    spi: SPI,
    turns: i64,
    prev_raw: u16,
    direction: Option<EncoderDirection>,
    inverted: bool,
}

impl<SPI> MT6701Spi<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        MT6701Spi {
            spi,
            turns: 0,
            prev_raw: 0,
            direction: None,
            inverted: false,
        }
    }
}

impl<SPI> MT6701Spi<SPI>
where
    SPI: SpiDevice,
{
    async fn read_raw_angle(&mut self) -> Result<(u16, EncoderMagneticFieldStatus), MT6701Error> {
        let mut buffer = [0; 2];

        self.spi
            .transaction(&mut [Operation::Read(&mut buffer)])
            .await
            .map_err(|_| MT6701Error::SpiError)?;

        let buf: u16 = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
        let status = match buf & 0x03 {
            0 => EncoderMagneticFieldStatus::Normal,
            1 => EncoderMagneticFieldStatus::TooStrong,
            2 => EncoderMagneticFieldStatus::TooWeak,
            _ => EncoderMagneticFieldStatus::Normal,
        };
        Ok((buf >> 2, status))
    }
}

impl<SPI> AbsolutePositionEncoder for MT6701Spi<SPI>
where
    SPI: SpiDevice,
{
    type Error = MT6701Error;

    /// Update the encoder and return the absolute position of the encoder in radians.
    async fn update(&mut self) -> Result<EncoderMeasurement, MT6701Error> {
        let (raw_angle, status) = self.read_raw_angle().await?;

        let mut angle = (I16F16::from_num(raw_angle) / ENCODER_MAGIC) * I16F16::TAU;

        if self.inverted {
            angle = I16F16::TAU - angle;
        }

        if self.prev_raw > OVERFLOW_HIGH_BAND && raw_angle < OVERFLOW_LOW_BAND {
            if self.inverted {
                self.turns -= 1;
            } else {
                self.turns += 1;
            }
        } else if self.prev_raw < OVERFLOW_LOW_BAND && raw_angle > OVERFLOW_HIGH_BAND {
            if self.inverted {
                self.turns += 1;
            } else {
                self.turns -= 1;
            }
        }

        self.prev_raw = raw_angle;

        Ok(EncoderMeasurement {
            angle,
            position: I16F16::from_num(self.turns) * I16F16::TAU + angle,
            magnetic_field: status,
        })
    }

    fn set_direction(&mut self, direction: EncoderDirection) {
        self.direction = Some(direction);
        self.inverted = self
            .direction
            .map(|d| d == EncoderDirection::CCW)
            .unwrap_or(false);
    }

    fn get_direction(&mut self) -> Option<EncoderDirection> {
        self.direction
    }
}
