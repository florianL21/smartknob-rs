use embassy_time::Timer;
use enterpolation::Signal;
use enterpolation::linear::Linear;
use fixed::types::I16F16;
use foc::park_clarke;
use foc::pwm::{Modulation, SpaceVector};
use log::info;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::encoder::AbsolutePositionEncoder;
use super::motor_driver::MotorDriver;
use crate::motor_control::encoder::{EncoderDirection, EncoderMeasurement};

// TODO: This needs to be configurable
const PWM_RESOLUTION: u16 = 999;

// TODO: This needs to adapt to the configured pole pairs
const MIN_ANGLE_DETECT_MOVEMENT: I16F16 = I16F16::lit("0.1");

const ZERO_ANGLE_TOLERANCE: I16F16 = I16F16::lit("0.05");
const SEARCH_STEP_SIZE: I16F16 = I16F16::lit("0.02");
const CALIBRATION_NUM_POINTS: usize = 20;

type EncoderCalibrationCurve = Linear<
    enterpolation::Sorted<[f32; CALIBRATION_NUM_POINTS]>,
    [f32; CALIBRATION_NUM_POINTS],
    enterpolation::Identity,
>;

#[derive(Error, Debug)]
pub enum HapticSystemError<E: core::error::Error> {
    #[error("Failed to detect any motor movement")]
    NoMovementDetected,
    #[error("Motor pole pairs did not match expected. Expected {0} measured {1}")]
    PolePairMismatch(u8, u8),
    #[error("Encoder error during alignment: {0}")]
    EncoderError(#[from] E),
    #[error("Failed to interpolate encoder data")]
    InterpolationFailed,
}

fn normalize_angle(angle: I16F16) -> I16F16 {
    let ang = angle % I16F16::TAU;
    if ang >= I16F16::ZERO {
        ang
    } else {
        ang + I16F16::TAU
    }
}

fn electrical_angle(
    pole_pairs: I16F16,
    mechanical_angle: I16F16,
    electrical_angle_offset: I16F16,
) -> I16F16 {
    normalize_angle(pole_pairs * mechanical_angle - electrical_angle_offset)
}

#[derive(Debug, PartialEq, Copy, Clone, Deserialize, Serialize)]
pub struct NonLinearityCorrection(EncoderCalibrationCurve);

impl NonLinearityCorrection {
    fn compensate<E: core::error::Error>(
        &self,
        measured_angle: I16F16,
    ) -> Result<I16F16, HapticSystemError<E>> {
        Ok(I16F16::from_num(
            self.0
                .sample([measured_angle.to_num()])
                .next()
                .ok_or_else(|| HapticSystemError::InterpolationFailed)?,
        ))
    }
}

#[derive(Debug, PartialEq, Copy, Clone, Deserialize, Serialize)]
pub struct CalibrationData {
    pole_pairs: I16F16,
    sensor_direction: EncoderDirection,
    calibration_curve: NonLinearityCorrection,
    electrical_angle_offset: I16F16,
}

impl CalibrationData {
    fn electrical_angle(self, mechanical_angle: I16F16) -> I16F16 {
        electrical_angle(
            self.pole_pairs,
            mechanical_angle,
            self.electrical_angle_offset,
        )
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
    calibration: Option<CalibrationData>,
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
            calibration: None,
        }
    }

    pub fn restore_calibration(&mut self, cal_data: CalibrationData) {
        self.encoder.set_direction(cal_data.sensor_direction);
        self.calibration = Some(cal_data);
    }

    pub fn get_calibration(&self) -> Option<&CalibrationData> {
        self.calibration.as_ref()
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibration.is_some()
    }

    fn drive_phases_alignment(&mut self, set_angle: I16F16) {
        self.motor_driver.set_pwm(&get_phase_voltage(
            self.alignment_voltage,
            I16F16::ZERO,
            set_angle,
        ));
    }

    /// Figure out the spinning direction of the motor in relation to the encoder and reverse the encoders direction accordingly
    async fn set_sensor_direction(
        &mut self,
    ) -> Result<EncoderDirection, HapticSystemError<E::Error>> {
        const NUM_STEPS: u16 = 500;

        let mut set_angle = I16F16::ZERO;
        let step_angle = I16F16::TAU / I16F16::from_num(NUM_STEPS);
        self.drive_phases_alignment(set_angle);

        for _ in 0..NUM_STEPS {
            Timer::after_millis(2).await;
            set_angle += step_angle;
            self.drive_phases_alignment(set_angle);
        }
        let mid_angle = self.encoder.update().await?;

        for _ in 0..NUM_STEPS {
            Timer::after_millis(2).await;
            set_angle -= step_angle;
            self.drive_phases_alignment(set_angle);
        }
        let end_angle = self.encoder.update().await?;
        let moved = (mid_angle.position - end_angle.position).abs();
        if moved < MIN_ANGLE_DETECT_MOVEMENT {
            return Err(HapticSystemError::NoMovementDetected);
        }
        let direction = if mid_angle.position < end_angle.position {
            EncoderDirection::CCW
        } else {
            EncoderDirection::CW
        };
        self.encoder.set_direction(direction);
        Ok(direction)
    }

    async fn measure_non_linearity(
        &mut self,
    ) -> Result<NonLinearityCorrection, HapticSystemError<E::Error>> {
        let mut current_electrical_angle = I16F16::ZERO;
        while self.encoder.update().await?.angle >= ZERO_ANGLE_TOLERANCE {
            Timer::after_millis(2).await;
            self.drive_phases_alignment(current_electrical_angle);
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

        let mut expected = [0.0; CALIBRATION_NUM_POINTS];
        let mut real = [0.0; CALIBRATION_NUM_POINTS];
        let mut last_measurement = I16F16::ZERO;
        let measure_step = I16F16::TAU / I16F16::from_num(CALIBRATION_NUM_POINTS);
        let mut count: usize = 0;
        let mut expected_angle = I16F16::ZERO;
        while measurement <= I16F16::TAU - ZERO_ANGLE_TOLERANCE {
            Timer::after_millis(2).await;
            expected_angle =
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
                expected[count] = expected_angle.to_num();
                real[count] = measurement.to_num();
                count += 1;
            }
            self.drive_phases_alignment(current_electrical_angle);
            current_electrical_angle += SEARCH_STEP_SIZE;
            measurement = self.encoder.update().await?.angle;
        }
        // Take a final measurement at the end point; this is also needed to fill up the last entry in the arrays
        expected[count] = expected_angle.to_num();
        real[count] = measurement.to_num();

        absolute_zero_electrical_angle = current_electrical_angle;
        info!(
            "Min deviation: {min_deviation:#?}\nMax deviation: {max_deviation:#?}\nMidpoint zero crossing: {closest_to_expected:#?}"
        );
        info!("Cal data: exp={expected:?}; real={real:?}");
        let interp = Linear::builder()
            .elements(expected)
            .knots(real)
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
            self.drive_phases_alignment(current_electrical_angle);
            current_electrical_angle -= SEARCH_STEP_SIZE;
            measurement = self.encoder.update().await?.angle;
        }
        info!(
            "With correction: \nMin deviation: {min_deviation:#?}\nMax deviation: {max_deviation:#?}\nMidpoint zero crossing: {closest_to_expected:#?}"
        );
        Ok(NonLinearityCorrection(interp))
    }

    async fn find_electrical_angle_offset(
        &mut self,
        cal_curve: &NonLinearityCorrection,
    ) -> Result<I16F16, HapticSystemError<E::Error>> {
        const _3PI_2: I16F16 = I16F16::PI
            .unwrapped_mul(I16F16::lit("3.0"))
            .unwrapped_div(I16F16::lit("2.0"));
        for i in 0..500 {
            let angle = _3PI_2 + I16F16::TAU * I16F16::from_num(i) / I16F16::from_num(500.0);
            self.drive_phases_alignment(angle);
            Timer::after_millis(2).await;
        }
        let _mid_angle = cal_curve.compensate(self.encoder.update().await?.angle)?;
        for i in 500..0 {
            let angle = _3PI_2 + I16F16::TAU * I16F16::from_num(i) / I16F16::from_num(500.0);
            self.drive_phases_alignment(angle);
            Timer::after_millis(2).await;
        }
        self.drive_phases_alignment(_3PI_2);
        Timer::after_millis(700).await;
        let end_angle = cal_curve.compensate(self.encoder.update().await?.angle)?;

        // Pole pair sanity check
        // let moved = (_mid_angle - end_angle).abs();
        // if (moved * self.pole_pairs - I16F16::TAU).abs() > I16F16::from_num(0.8) {
        //     let estimated_pole_pairs = (I16F16::TAU / moved).to_num();
        //     return Err(HapticSystemError::PolePairMismatch(
        //         self.pole_pairs.to_num(),
        //         estimated_pole_pairs,
        //     ));
        // }
        let electrical_zero_angle = electrical_angle(self.pole_pairs, end_angle, I16F16::ZERO);
        info!("Electrical zero angle offset: {electrical_zero_angle}");
        Ok(electrical_zero_angle)
    }

    pub async fn align(&mut self) -> Result<(), HapticSystemError<E::Error>> {
        let sensor_dir = self.set_sensor_direction().await?;
        let cal_curve = self.measure_non_linearity().await?;
        let electrical_angle_offset = self.find_electrical_angle_offset(&cal_curve).await?;

        self.calibration = Some(CalibrationData {
            pole_pairs: self.pole_pairs,
            sensor_direction: sensor_dir,
            calibration_curve: cal_curve,
            electrical_angle_offset,
        });

        info!("Calibration complete: {:#?}", self.calibration);

        self.motor_driver.set_pwm(&[0; 3]);
        Ok(())
    }

    pub async fn run(
        &mut self,
        mut torque: impl FnMut(&EncoderMeasurement) -> I16F16,
    ) -> Result<EncoderMeasurement, HapticSystemError<E::Error>> {
        let measurement = self.encoder.update().await?;
        if let Some(cal_data) = &self.calibration {
            let compensated_angle = cal_data
                .calibration_curve
                .compensate::<E::Error>(measurement.angle)?;
            let electrical_angle = cal_data.electrical_angle(compensated_angle);
            self.motor_driver.set_pwm(&get_phase_voltage(
                torque(&measurement),
                I16F16::ZERO,
                electrical_angle,
            ));
        }
        Ok(measurement)
    }
}
