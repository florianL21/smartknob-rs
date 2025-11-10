use core::{f32, f64};

use atomic_float::AtomicF32;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{AnyPin, Output},
    mcpwm::{
        operator::{DeadTimeCfg, PwmPinConfig},
        timer::PwmWorkingMode,
        McPwm, PeripheralClockConfig,
    },
    time::Rate,
};
use fixed::{types::I16F16, FixedI32};
use foc::pwm::Modulation;
use foc::{park_clarke, pid::PIController, pwm::SpaceVector, Foc};
use mt6701::{self, AngleSensorTrait};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use log::{error, info};
use thiserror::Error;

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

pub static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);
pub static ENCODER_ANGLE: AtomicF32 = AtomicF32::new(0.0);

pub static MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();

const PWM_RESOLUTION: u16 = 999;
const MOTOR_POLE_PAIRS: I16F16 = I16F16::lit("4");

const _3PI_2: I16F16 = I16F16::PI
    .unwrapped_mul(I16F16::lit("3.0"))
    .unwrapped_div(I16F16::lit("2.0"));
const _2PI: I16F16 = I16F16::PI.unwrapped_mul(I16F16::lit("2.0"));
const MIN_ANGLE_DETECT_MOVEMENT: I16F16 = _2PI.unwrapped_div(I16F16::lit("101.0"));
const ALIGNMENT_VOLTAGE: I16F16 = I16F16::lit("1.0");

pub struct Pins6PWM {
    pub uh: AnyPin<'static>,
    pub ul: AnyPin<'static>,
    pub vh: AnyPin<'static>,
    pub vl: AnyPin<'static>,
    pub wh: AnyPin<'static>,
    pub wl: AnyPin<'static>,
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum SensorDirection {
    CW,
    CCW,
}

fn normalize_angle(angle: I16F16) -> I16F16 {
    let ang = angle % _2PI;
    if ang >= I16F16::ZERO {
        ang
    } else {
        ang + _2PI
    }
}

impl SensorDirection {
    fn electrical_angle(self, mechanical_angle: I16F16) -> I16F16 {
        match self {
            SensorDirection::CW => normalize_angle(MOTOR_POLE_PAIRS * mechanical_angle),
            SensorDirection::CCW => normalize_angle(-MOTOR_POLE_PAIRS * mechanical_angle),
        }
    }
}

#[derive(Error, Debug, PartialEq)]
enum AlignmentFailureReason {
    #[error("Failed to detect any motor movement")]
    NoMovementDetected,
    #[error("Motor pole pairs did not match expected. Expected {MOTOR_POLE_PAIRS} measured {0}")]
    PolePairMismatch(u8),
}

#[derive(PartialEq, Debug)]
enum AlignmentState {
    NeedsAlignment,
    AlignForward {
        counter: u16,
    },
    AlignBackward {
        counter: u16,
        mid_angle: I16F16,
    },
    CalculateDirection {
        mid_angle: I16F16,
        end_angle: I16F16,
    },
    PrepareMeasureAngleOffset {
        sensor_direction: SensorDirection,
    },
    WaitForMotorToSettle {
        sensor_direction: SensorDirection,
        start_time: Instant,
    },
    MeasureAngleOffset {
        sensor_direction: SensorDirection,
    },
    IsAligned {
        sensor_direction: SensorDirection,
        electric_zero_angle: I16F16,
    },
    AlignmentFailure(AlignmentFailureReason),
}

impl AlignmentState {
    /// Initiate the alignment procedure
    fn start_alignment(&mut self) {
        *self = AlignmentState::AlignForward { counter: 0 };
    }

    /// Runs the alignment procedure.
    /// Never blocks. Alignment is done in iterations, This function never loops internally but instead
    /// keeps track of the alignment process via it's own internal state.
    /// It is crucial to pass the function the most up to date encoder angle values
    async fn do_alignment(
        &mut self,
        encoder_position: I16F16,
        encoder_angle: I16F16,
    ) -> Option<[u16; 3]> {
        match self {
            AlignmentState::AlignForward { counter } => {
                if *counter >= 500 {
                    info!("Captured mid angle: {}", encoder_position);
                    *self = AlignmentState::AlignBackward {
                        counter: 500,
                        mid_angle: encoder_position,
                    };
                    None
                } else {
                    // We can wait here to make the movement to the motor slow and hopefully not miss a step.
                    // Waiting before is acceptable here because we don't care for super accurate encoder values yet
                    Timer::after_millis(2).await;
                    let angle =
                        _3PI_2 + _2PI * I16F16::from_num(*counter) / I16F16::from_num(500.0);
                    let pwm = get_phase_voltage(ALIGNMENT_VOLTAGE, I16F16::ZERO, angle);
                    *self = AlignmentState::AlignForward {
                        counter: *counter + 1,
                    };
                    Some(pwm)
                }
            }
            AlignmentState::AlignBackward { counter, mid_angle } => {
                if *counter == 0 {
                    info!("Captured end angle: {}", encoder_position);
                    *self = AlignmentState::CalculateDirection {
                        mid_angle: *mid_angle,
                        end_angle: encoder_position,
                    };
                    None
                } else {
                    // We can wait here to make the movement to the motor slow and hopefully not miss a step.
                    // Waiting before is acceptable here because we don't care for super accurate encoder values yet
                    Timer::after_millis(2).await;
                    let angle =
                        _3PI_2 + _2PI * I16F16::from_num(*counter) / I16F16::from_num(500.0);
                    let pwm = get_phase_voltage(ALIGNMENT_VOLTAGE, I16F16::ZERO, angle);
                    *self = AlignmentState::AlignBackward {
                        counter: *counter - 1,
                        mid_angle: *mid_angle,
                    };
                    Some(pwm)
                }
            }
            AlignmentState::CalculateDirection {
                mid_angle,
                end_angle,
            } => {
                let moved = (*mid_angle - *end_angle).abs();
                if moved < MIN_ANGLE_DETECT_MOVEMENT {
                    *self = AlignmentState::AlignmentFailure(
                        AlignmentFailureReason::NoMovementDetected,
                    );
                    error!("{self:?}");
                    return Some([0; 3]);
                }
                let sensor_direction = if mid_angle < end_angle {
                    SensorDirection::CCW
                } else {
                    SensorDirection::CW
                };
                info!("Mid angle: {}, End angle: {}", mid_angle, end_angle);
                // 0.5f is arbitrary number it can be lower or higher!
                // Something here seems fishy because with 0.5 it never passes.
                // Increased to 0.9 for now, will need to check back at a later time
                if (moved * I16F16::from_num(MOTOR_POLE_PAIRS) - _2PI).abs() > I16F16::from_num(0.8)
                {
                    let estimated_pole_pairs = (_2PI / moved).to_num();
                    let error = AlignmentFailureReason::PolePairMismatch(estimated_pole_pairs);
                    error!("{error}; Moved {moved}");
                    *self = AlignmentState::AlignmentFailure(error);
                    return Some([0; 3]);
                }
                *self = AlignmentState::PrepareMeasureAngleOffset { sensor_direction };
                None
            }
            AlignmentState::PrepareMeasureAngleOffset { sensor_direction } => {
                *self = AlignmentState::WaitForMotorToSettle {
                    sensor_direction: *sensor_direction,
                    start_time: Instant::now(),
                };
                Some(get_phase_voltage(ALIGNMENT_VOLTAGE, I16F16::ZERO, _3PI_2))
            }
            AlignmentState::WaitForMotorToSettle {
                sensor_direction,
                start_time,
            } => {
                if start_time.elapsed().as_millis() > 700 {
                    *self = AlignmentState::MeasureAngleOffset {
                        sensor_direction: *sensor_direction,
                    };
                }
                None
            }
            AlignmentState::MeasureAngleOffset { sensor_direction } => {
                *self = AlignmentState::IsAligned {
                    sensor_direction: *sensor_direction,
                    electric_zero_angle: sensor_direction.electrical_angle(encoder_angle),
                };
                info!("Alignment finished. Electrical zero angle is: {encoder_angle}");
                Some([0; 3])
            }
            AlignmentState::IsAligned { .. } => None,
            Self::AlignmentFailure(_) => None,
            AlignmentState::NeedsAlignment => None,
        }
    }

    /// Returns true if motor and sensor are aligned and the motor is allowed to move
    fn is_aligned(&self) -> bool {
        matches!(*self, AlignmentState::IsAligned { .. })
    }
}

pub enum MotorCommand {
    StartAlignment,
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

#[embassy_executor::task]
pub async fn update_foc(
    spi_bus: &'static SpiBus1,
    mag_csn: Output<'static>,
    mcpwm0: esp_hal::peripherals::MCPWM0<'static>,
    pwm_pins: Pins6PWM,
) {
    let mut alignment_state = AlignmentState::NeedsAlignment;
    const MAX_CURRENT: u8 = 2;
    // about 2% dead time
    const PWM_DEAD_TIME: u16 = 20;
    let spi_device = SpiDevice::new(spi_bus, mag_csn);

    let mut encoder: mt6701::MT6701Spi<
        SpiDevice<
            '_,
            NoopRawMutex,
            esp_hal::spi::master::SpiDmaBus<'_, esp_hal::Async>,
            Output<'_>,
        >,
    > = mt6701::MT6701Spi::new(spi_device);
    let mut t = Instant::now();
    info!("encoder init done!");

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    let mut mcpwm = McPwm::new(mcpwm0, clock_cfg);

    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);

    let mut _pwm_u = mcpwm.operator0.with_linked_pins(
        pwm_pins.uh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pwm_pins.ul,
        PwmPinConfig::UP_ACTIVE_HIGH,
        DeadTimeCfg::new_ahc(),
    );
    let mut _pwm_v = mcpwm.operator1.with_linked_pins(
        pwm_pins.vh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pwm_pins.vl,
        PwmPinConfig::UP_ACTIVE_HIGH,
        DeadTimeCfg::new_ahc(),
    );
    let mut _pwm_w = mcpwm.operator2.with_linked_pins(
        pwm_pins.wh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pwm_pins.wl,
        PwmPinConfig::UP_ACTIVE_HIGH,
        DeadTimeCfg::new_ahc(),
    );
    _pwm_u.set_falling_edge_deadtime(PWM_DEAD_TIME);
    _pwm_u.set_rising_edge_deadtime(PWM_DEAD_TIME);
    _pwm_v.set_falling_edge_deadtime(PWM_DEAD_TIME);
    _pwm_v.set_rising_edge_deadtime(PWM_DEAD_TIME);
    _pwm_w.set_falling_edge_deadtime(PWM_DEAD_TIME);
    _pwm_w.set_rising_edge_deadtime(PWM_DEAD_TIME);

    // Turn off all outputs
    _pwm_u.set_timestamp_a(0);
    _pwm_u.set_timestamp_b(0);
    _pwm_v.set_timestamp_a(0);
    _pwm_v.set_timestamp_b(0);
    _pwm_w.set_timestamp_a(0);
    _pwm_w.set_timestamp_b(0);

    // period here is in relation to all other periods further down.
    // Dead time and set_timestamp methods respectiveley
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(PWM_RESOLUTION, PwmWorkingMode::Increase, Rate::from_khz(25))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let _fcc = PIController::new(I16F16::from_num(0.6), I16F16::from_num(0));
    let _tcc = PIController::new(I16F16::from_num(0.6), I16F16::from_num(0));

    let mut foc: Foc<SpaceVector, PWM_RESOLUTION> = Foc::new(_fcc, _tcc);
    let mut angle = I16F16::from_num(0.0);
    let mut last_pwm = [0u16; 3];
    let mut encoder_pos: f64 = 0.0;
    let mut encoder_angle: f32 = 0.0;
    loop {
        if encoder.update(t.elapsed().into()).await.is_ok() {
            t = Instant::now();
            encoder_pos = encoder.get_position();
            encoder_angle = encoder.get_angle();
            ENCODER_POSITION.store(encoder_pos as f32, core::sync::atomic::Ordering::Relaxed);
            ENCODER_ANGLE.store(encoder_angle, core::sync::atomic::Ordering::Relaxed);
        }
        let encoder_pos = I16F16::from_num(encoder_pos);
        let encoder_angle = I16F16::from_num(encoder_angle);
        if MOTOR_COMMAND_SIGNAL.signaled() {
            alignment_state.start_alignment();
            MOTOR_COMMAND_SIGNAL.reset();
        }
        // In case the alignment was not yet done, this will do it in a non blocking way
        if let Some(pwm) = alignment_state
            .do_alignment(encoder_pos, encoder_angle)
            .await
        {
            _pwm_u.set_timestamp_a(pwm[0]);
            _pwm_v.set_timestamp_a(pwm[1]);
            _pwm_w.set_timestamp_a(pwm[2]);
        }
        if alignment_state.is_aligned() {
            let fake_amps0 = if last_pwm[0] != 0 {
                (PWM_RESOLUTION as f32 / last_pwm[0] as f32) * MAX_CURRENT as f32
            } else {
                0f32
            };
            let fake_amps1 = if last_pwm[1] != 0 {
                (PWM_RESOLUTION as f32 / last_pwm[1] as f32) * MAX_CURRENT as f32
            } else {
                0f32
            };

            let pwm = foc.update(
                [
                    FixedI32::from_num(fake_amps0),
                    FixedI32::from_num(fake_amps1),
                ],
                angle,
                I16F16::from_num(1),
                I16F16::from_num(t.elapsed().as_micros()),
            );
            angle += I16F16::from_num(0.01);
            // _pwm_u.set_timestamp_a(pwm[0]);
            // _pwm_v.set_timestamp_a(pwm[1]);
            // _pwm_w.set_timestamp_a(pwm[2]);
            last_pwm = pwm;
            Timer::after_millis(2).await;
        }
    }
}
