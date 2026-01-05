use embassy_time::Timer;
use enterpolation::Signal;
use enterpolation::linear::Linear;
use fixed::types::I16F16;
use foc::park_clarke;
use foc::pwm::{Modulation, SpaceVector};
use heapless::Vec;
use log::info;
use thiserror::Error;

use super::encoder::AbsolutePositionEncoder;
use super::motor_driver::MotorDriver;
use crate::motor_control::encoder::EncoderDirection;

// TODO: This needs to be configurable
const PWM_RESOLUTION: u16 = 999;

// TODO: This needs to adapt to the configured pole pairs
const MIN_ANGLE_DETECT_MOVEMENT: I16F16 = I16F16::lit("0.1");

#[derive(Error, Debug)]
pub enum AlignmentError<E: core::error::Error> {
    #[error("Failed to detect any motor movement")]
    NoMovementDetected,
    #[error("Motor pole pairs did not match expected. Expected {0} measured {1}")]
    PolePairMismatch(u8, u8),
    #[error("Encoder error during alignment: {0}")]
    EncoderError(#[from] E),
}

fn normalize_angle(angle: I16F16) -> I16F16 {
    let ang = angle % I16F16::TAU;
    if ang >= I16F16::ZERO {
        ang
    } else {
        ang + I16F16::TAU
    }
}

pub struct HapticSystem<E, D>
where
    E: AbsolutePositionEncoder,
    D: MotorDriver,
{
    encoder: E,
    motor_driver: D,
    alignment_voltage: I16F16,
    pole_pairs: I16F16,
}

fn get_phase_voltage(uq: I16F16, ud: I16F16, angle: I16F16) -> [u16; 3] {
    let (sin_angle, cos_angle) = cordic::sin_cos(angle);
    let orthogonal_voltage = park_clarke::inverse_park(
        cos_angle,
        sin_angle,
        park_clarke::RotatingReferenceFrame { d: ud, q: uq },
    );

    // Modulate the result to PWM values
    SpaceVector::as_compare_value::<PWM_RESOLUTION>(orthogonal_voltage)
}

impl<E: AbsolutePositionEncoder, D: MotorDriver> HapticSystem<E, D> {
    pub async fn new(
        mut encoder: E,
        motor_driver: D,
        alignment_voltage: I16F16,
        pole_pairs: u8,
    ) -> Self {
        // Sample the encoder once to ge its initial position and have it set its internal state
        let _ = encoder.update().await;
        Self {
            encoder,
            motor_driver,
            alignment_voltage,
            pole_pairs: I16F16::from_num(pole_pairs),
        }
    }

    fn drive_phases(&mut self, set_angle: I16F16) {
        self.motor_driver.set_pwm(&get_phase_voltage(
            self.alignment_voltage,
            I16F16::ZERO,
            set_angle,
        ));
    }

    /// Figure out the spinning direction of the motor in relation to the encoder and reverse the encoders direction accordingly
    async fn set_sensor_direction(&mut self) -> Result<(), AlignmentError<E::Error>> {
        const NUM_STEPS: u16 = 500;

        let mut set_angle = I16F16::ZERO;
        let step_angle = I16F16::TAU / I16F16::from_num(NUM_STEPS);
        self.drive_phases(set_angle);

        for _ in 0..NUM_STEPS {
            Timer::after_millis(2).await;
            set_angle += step_angle;
            self.drive_phases(set_angle);
        }
        let mid_angle = self.encoder.update().await?;

        for _ in 0..NUM_STEPS {
            Timer::after_millis(2).await;
            set_angle -= step_angle;
            self.drive_phases(set_angle);
        }
        let end_angle = self.encoder.update().await?;
        let moved = (mid_angle.position - end_angle.position).abs();
        if moved < MIN_ANGLE_DETECT_MOVEMENT {
            return Err(AlignmentError::NoMovementDetected);
        }
        self.encoder
            .set_direction(if mid_angle.position < end_angle.position {
                EncoderDirection::CCW
            } else {
                EncoderDirection::CW
            });
        Ok(())
    }

    async fn measure_non_linearity(&mut self) -> Result<(), AlignmentError<E::Error>> {
        const ZERO_ANGLE_TOLERANCE: I16F16 = I16F16::lit("0.05");
        const SEARCH_STEP_SIZE: I16F16 = I16F16::lit("0.02");
        let mut current_electrical_angle = I16F16::ZERO;
        while self.encoder.update().await?.angle >= ZERO_ANGLE_TOLERANCE {
            Timer::after_millis(2).await;
            self.drive_phases(current_electrical_angle);
            current_electrical_angle += SEARCH_STEP_SIZE;
        }
        let mut absolute_zero_electrical_angle = current_electrical_angle;
        info!("Found 0 at electrical angle {absolute_zero_electrical_angle}!");

        #[derive(Debug)]
        struct DeviationAtAngle {
            angle: I16F16,
            deviation: I16F16,
        }

        let mut min_deviation = DeviationAtAngle {
            angle: I16F16::ZERO,
            deviation: I16F16::ZERO,
        };
        let mut max_deviation = DeviationAtAngle {
            angle: I16F16::ZERO,
            deviation: I16F16::ZERO,
        };
        let mut closest_to_expected = DeviationAtAngle {
            angle: I16F16::ZERO,
            deviation: I16F16::ONE,
        };

        let mut measurement = self.encoder.update().await?.angle;

        const NUM_POINTS: usize = 20;
        let mut expected: Vec<f32, NUM_POINTS> = Vec::new();
        let mut real: Vec<f32, NUM_POINTS> = Vec::new();
        let mut last_measurement = I16F16::ZERO;
        let measure_step = I16F16::TAU / I16F16::from_num(NUM_POINTS);
        while measurement <= I16F16::TAU - ZERO_ANGLE_TOLERANCE {
            Timer::after_millis(2).await;
            let expected_angle =
                (current_electrical_angle - absolute_zero_electrical_angle) / self.pole_pairs;
            let diff = measurement - expected_angle;
            if diff.is_negative() {
                if diff.abs() > min_deviation.deviation {
                    min_deviation.deviation = diff.abs();
                    min_deviation.angle = expected_angle;
                }
            } else {
                if diff > max_deviation.deviation {
                    max_deviation.deviation = diff;
                    max_deviation.angle = expected_angle;
                }
            }
            // Cut off the start and the end of the curve as we want to catch the deviation at the curves mid point
            if expected_angle > I16F16::ONE && expected_angle < I16F16::TAU - I16F16::ONE {
                if diff.abs() < closest_to_expected.deviation {
                    closest_to_expected.deviation = diff.abs();
                    closest_to_expected.angle = expected_angle;
                }
            }
            if expected_angle - last_measurement > measure_step {
                last_measurement = expected_angle;
                let _ = expected.push(expected_angle.to_num());
                let _ = real.push(measurement.to_num());
            }
            self.drive_phases(current_electrical_angle);
            current_electrical_angle += SEARCH_STEP_SIZE;
            measurement = self.encoder.update().await?.angle;
        }
        absolute_zero_electrical_angle = current_electrical_angle;
        info!(
            "Min deviation: {min_deviation:#?}\nMax deviation: {max_deviation:#?}\nMidpoint zero crossing: {closest_to_expected:#?}"
        );
        let interp = Linear::builder()
            .elements(expected.as_slice())
            .knots(real.as_slice())
            .build()
            .unwrap();

        let mut min_deviation = DeviationAtAngle {
            angle: I16F16::ZERO,
            deviation: I16F16::ZERO,
        };
        let mut max_deviation = DeviationAtAngle {
            angle: I16F16::ZERO,
            deviation: I16F16::ZERO,
        };
        let mut closest_to_expected = DeviationAtAngle {
            angle: I16F16::ZERO,
            deviation: I16F16::ONE,
        };

        let mut measurement = self.encoder.update().await?.angle;
        while measurement >= ZERO_ANGLE_TOLERANCE {
            Timer::after_millis(2).await;
            let expected_angle = I16F16::TAU
                - (absolute_zero_electrical_angle - current_electrical_angle) / self.pole_pairs;
            let corrected = I16F16::from_num(interp.sample([measurement.to_num()]).next().unwrap());
            let diff = corrected - expected_angle;
            if diff.is_negative() {
                if diff.abs() > min_deviation.deviation {
                    min_deviation.deviation = diff.abs();
                    min_deviation.angle = expected_angle;
                }
            } else {
                if diff > max_deviation.deviation {
                    max_deviation.deviation = diff;
                    max_deviation.angle = expected_angle;
                }
            }
            // Cut off the start and the end of the curve as we want to catch the deviation at the curves mid point
            if expected_angle > I16F16::ONE && expected_angle < I16F16::TAU - I16F16::ONE {
                if diff.abs() < closest_to_expected.deviation {
                    closest_to_expected.deviation = diff.abs();
                    closest_to_expected.angle = expected_angle;
                }
            }
            self.drive_phases(current_electrical_angle);
            current_electrical_angle -= SEARCH_STEP_SIZE;
            measurement = self.encoder.update().await?.angle;
        }
        info!(
            "With correction: \nMin deviation: {min_deviation:#?}\nMax deviation: {max_deviation:#?}\nMidpoint zero crossing: {closest_to_expected:#?}"
        );
        Ok(())
    }

    pub async fn align(&mut self) -> Result<(), AlignmentError<E::Error>> {
        self.set_sensor_direction().await?;
        self.measure_non_linearity().await?;

        self.motor_driver.set_pwm(&[0; 3]);
        Ok(())
    }
}
