use crate::motor_control::encoder::{EncoderMagneticFieldStatus, EncoderMeasurement};

use super::AbsolutePositionEncoder;
use embedded_hal_async::spi::{Operation, SpiDevice};

use core::fmt::Debug;
use fixed::types::I16F16;

const OVERFLOW_LOW_BAND: u16 = 1600;
const OVERFLOW_HIGH_BAND: u16 = (2u16.pow(14)) - OVERFLOW_LOW_BAND;

const ENCODER_MAGIC: I16F16 = I16F16::lit("16384");

#[derive(Debug)]
pub enum MT6701Error {
    SpiError,
    CsnError,
}

#[derive(Debug)]
pub struct MT6701Spi<SPI> {
    spi: SPI,
    turns: i64,
    prev_raw: u16,
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

        let angle = (I16F16::from_num(raw_angle) / ENCODER_MAGIC) * I16F16::TAU;

        if self.prev_raw > OVERFLOW_HIGH_BAND && raw_angle < OVERFLOW_LOW_BAND {
            self.turns += 1;
        } else if self.prev_raw < OVERFLOW_LOW_BAND && raw_angle > OVERFLOW_HIGH_BAND {
            self.turns -= 1;
        }

        self.prev_raw = raw_angle;

        Ok(EncoderMeasurement {
            angle,
            position: I16F16::from_num(self.turns) * I16F16::TAU + angle,
            magnetic_field: status,
        })
    }
}
