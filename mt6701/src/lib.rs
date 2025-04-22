#![no_std]

use embedded_hal_async::spi::{Operation, SpiDevice};

use core::f32::consts::PI;
use core::fmt::Debug;
use core::future;
use core::time::Duration;
use libm::fabsf;

const _2PI: f32 = PI * 2.0;

#[derive(Debug)]
pub enum MT6701Error {
    SpiError,
    CsnError,
}

pub trait AngleSensorTrait {
    type Error: Debug;

    fn read_raw_angle(&mut self) -> impl future::Future<Output = Result<u16, Self::Error>>;
    fn update(&mut self, now_us: Duration)
    -> impl future::Future<Output = Result<(), Self::Error>>;
    fn get_angle(&mut self) -> f32;
    fn get_turns(&mut self) -> i64;
    fn get_position(&mut self) -> f64;
    fn get_velocity(&mut self) -> f32;
}

#[derive(Debug)]
pub struct MT6701Spi<SPI> {
    spi: SPI,
    turns: i64,
    angle: f32,
    angle_prev: f32,
    position: f64,
    position_prev: f64,
    velocity: f32,
}

impl<SPI> MT6701Spi<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        MT6701Spi {
            spi,
            turns: 0,
            position: 0.0,
            position_prev: 0.0,
            angle: 0.0,
            angle_prev: 0.0,
            velocity: 0.0,
        }
    }

    fn cal_velocity(&mut self, time_delta: Duration) {
        if time_delta.is_zero() {
            self.velocity = 0.0;
            return;
        }

        let mut ts = time_delta.as_micros() as f32 * 1e-6;
        if ts < 0.0 {
            ts = 1e-3;
        }

        self.velocity = (self.position - self.position_prev) as f32 / ts;

        self.position_prev = self.position;
    }
}

impl<SPI> AngleSensorTrait for MT6701Spi<SPI>
where
    SPI: SpiDevice,
{
    type Error = MT6701Error;

    async fn read_raw_angle(&mut self) -> Result<u16, MT6701Error> {
        let mut buffer: [u8; 2] = [0; 2];

        self.spi
            .transaction(&mut [Operation::Read(&mut buffer)])
            .await
            .map_err(|_| MT6701Error::SpiError)?;

        let buf: u16 = ((buffer[0] as u16) << 8) | (buffer[1] as u16);

        Ok((buf >> 1) & 0x3FFF)
    }

    /// Update the encoder state and re-calculate angle, turns, position, and velocity
    async fn update(&mut self, duration_since_last_call: Duration) -> Result<(), MT6701Error> {
        let raw_angle = self.read_raw_angle().await?;

        self.angle = (raw_angle as f32 / 16384_f32) * _2PI;
        let move_angle = self.angle - self.angle_prev;

        if fabsf(move_angle) > (0.8 * _2PI) {
            self.turns += if move_angle > 0.0 { -1 } else { 1 };
        }

        self.position = (self.turns as f32 * _2PI + self.angle) as f64;

        self.cal_velocity(duration_since_last_call);

        self.angle_prev = self.angle;

        Ok(())
    }

    /// Get the current angle in radians
    fn get_angle(&mut self) -> f32 {
        self.angle
    }

    /// Get the number of whole absolute turns
    fn get_turns(&mut self) -> i64 {
        self.turns
    }

    /// Get the current position (turns + angle) in radians
    fn get_position(&mut self) -> f64 {
        self.position
    }

    /// Get the current velocity in radians per second
    fn get_velocity(&mut self) -> f32 {
        self.velocity
    }
}
