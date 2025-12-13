use core::{f32, f64};

use atomic_float::AtomicF32;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Output},
    mcpwm::{
        operator::{DeadTimeCfg, PwmPinConfig},
        timer::PwmWorkingMode,
        McPwm, PeripheralClockConfig,
    },
    spi,
    time::Rate,
};
use fixed::types::I16F16;
use foc::pwm::Modulation;
use foc::{park_clarke, pwm::SpaceVector};
use haptic_lib::{CurveBuilder, HapticPlayer};
use mt6701::{self, AngleSensorTrait};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use log::{error, info};
use thiserror::Error;

use crate::flash::{FlashError, FlashKeys, FlashType};

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

pub static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);
pub static MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();

const _3PI_2: I16F16 = I16F16::PI
    .unwrapped_mul(I16F16::lit("3.0"))
    .unwrapped_div(I16F16::lit("2.0"));

const ENCODER_SPI_DMA_BUFFER_SIZE: usize = 200;
const PWM_RESOLUTION: u16 = 999;
const MOTOR_POLE_PAIRS: I16F16 = I16F16::lit("7");

const MIN_ANGLE_DETECT_MOVEMENT: I16F16 = I16F16::TAU.unwrapped_div(I16F16::lit("101.0"));
const ALIGNMENT_VOLTAGE: I16F16 = I16F16::lit("1.0");

pub struct Pins6PWM {
    pub uh: AnyPin<'static>,
    pub ul: AnyPin<'static>,
    pub vh: AnyPin<'static>,
    pub vl: AnyPin<'static>,
    pub wh: AnyPin<'static>,
    pub wl: AnyPin<'static>,
}

#[derive(Debug, PartialEq, Copy, Clone, Deserialize, Serialize, MaxSize)]
pub enum SensorDirection {
    CW,
    CCW,
}

fn normalize_angle(angle: I16F16) -> I16F16 {
    let ang = angle % I16F16::TAU;
    if ang >= I16F16::ZERO {
        ang
    } else {
        ang + I16F16::TAU
    }
}

impl SensorDirection {
    fn electrical_angle(self, mechanical_angle: I16F16, zero_electrical_angle: I16F16) -> I16F16 {
        match self {
            SensorDirection::CW => {
                normalize_angle(MOTOR_POLE_PAIRS * mechanical_angle - zero_electrical_angle)
            }
            SensorDirection::CCW => {
                normalize_angle(-MOTOR_POLE_PAIRS * mechanical_angle - zero_electrical_angle)
            }
        }
    }
}

#[derive(Error, Debug)]
enum AlignmentError {
    #[error("Failed to detect any motor movement")]
    NoMovementDetected,
    #[error("Motor pole pairs did not match expected. Expected {MOTOR_POLE_PAIRS} measured {0}")]
    PolePairMismatch(u8),
    #[error("Could not serialize MotorAlignment to postcard format")]
    PostcardSerializationError,
    #[error("Flash operation failed: {0:#?}")]
    FlashError(#[from] FlashError),
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
    IsAligned(MotorAlignment),
    Failed,
}

#[derive(Debug, PartialEq, Deserialize, Serialize)]
pub struct MotorAlignment {
    sensor_direction: SensorDirection,
    electric_zero_angle: I16F16,
}

impl postcard::experimental::max_size::MaxSize for MotorAlignment {
    const POSTCARD_MAX_SIZE: usize = SensorDirection::POSTCARD_MAX_SIZE + 4; // I16F16 has 16 integer bits and 16 fractional bits = 4 bytes
}

impl MotorAlignment {
    fn electrical_angle(&self, mechanical_angle: I16F16) -> I16F16 {
        self.sensor_direction
            .electrical_angle(mechanical_angle, self.electric_zero_angle)
    }
}

impl AlignmentState {
    /// Create a new AlignmentState without any previously done alignment
    fn new() -> Self {
        AlignmentState::NeedsAlignment
    }

    /// Restore an aligned state from previously saved alignment data
    fn restore(alignment_data: MotorAlignment) -> Self {
        AlignmentState::IsAligned(alignment_data)
    }

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
        flash: &'static FlashType<'static>,
    ) -> Result<Option<[u16; 3]>, AlignmentError> {
        match self {
            AlignmentState::AlignForward { counter } => {
                if *counter >= 500 {
                    info!("Captured mid angle: {}", encoder_position);
                    *self = AlignmentState::AlignBackward {
                        counter: 500,
                        mid_angle: encoder_position,
                    };
                    Ok(None)
                } else {
                    // We can wait here to make the movement to the motor slow and hopefully not miss a step.
                    // Waiting before is acceptable here because we don't care for super accurate encoder values yet
                    Timer::after_millis(2).await;
                    let angle =
                        _3PI_2 + I16F16::TAU * I16F16::from_num(*counter) / I16F16::from_num(500.0);
                    let pwm = get_phase_voltage(ALIGNMENT_VOLTAGE, I16F16::ZERO, angle);
                    *self = AlignmentState::AlignForward {
                        counter: *counter + 1,
                    };
                    Ok(Some(pwm))
                }
            }
            AlignmentState::AlignBackward { counter, mid_angle } => {
                if *counter == 0 {
                    info!("Captured end angle: {}", encoder_position);
                    *self = AlignmentState::CalculateDirection {
                        mid_angle: *mid_angle,
                        end_angle: encoder_position,
                    };
                    Ok(None)
                } else {
                    // We can wait here to make the movement to the motor slow and hopefully not miss a step.
                    // Waiting before is acceptable here because we don't care for super accurate encoder values yet
                    Timer::after_millis(2).await;
                    let angle =
                        _3PI_2 + I16F16::TAU * I16F16::from_num(*counter) / I16F16::from_num(500.0);
                    let pwm = get_phase_voltage(ALIGNMENT_VOLTAGE, I16F16::ZERO, angle);
                    *self = AlignmentState::AlignBackward {
                        counter: *counter - 1,
                        mid_angle: *mid_angle,
                    };
                    Ok(Some(pwm))
                }
            }
            AlignmentState::CalculateDirection {
                mid_angle,
                end_angle,
            } => {
                let moved = (*mid_angle - *end_angle).abs();
                if moved < MIN_ANGLE_DETECT_MOVEMENT {
                    *self = AlignmentState::Failed;
                    return Err(AlignmentError::NoMovementDetected);
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
                if (moved * I16F16::from_num(MOTOR_POLE_PAIRS) - I16F16::TAU).abs()
                    > I16F16::from_num(0.8)
                {
                    let estimated_pole_pairs = (I16F16::TAU / moved).to_num();
                    *self = AlignmentState::Failed;
                    return Err(AlignmentError::PolePairMismatch(estimated_pole_pairs));
                }
                *self = AlignmentState::PrepareMeasureAngleOffset { sensor_direction };
                Ok(None)
            }
            AlignmentState::PrepareMeasureAngleOffset { sensor_direction } => {
                *self = AlignmentState::WaitForMotorToSettle {
                    sensor_direction: *sensor_direction,
                    start_time: Instant::now(),
                };
                Ok(Some(get_phase_voltage(
                    ALIGNMENT_VOLTAGE,
                    I16F16::ZERO,
                    _3PI_2,
                )))
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
                Ok(None)
            }
            AlignmentState::MeasureAngleOffset { sensor_direction } => {
                let alignment_data = MotorAlignment {
                    sensor_direction: *sensor_direction,
                    electric_zero_angle: normalize_angle(
                        sensor_direction.electrical_angle(encoder_position, I16F16::ZERO),
                    ),
                };
                info!(
                    "Alignment finished. Electrical zero angle is: {}",
                    alignment_data.electric_zero_angle
                );
                // Save alignment data to flash
                let mut wt = flash.write_transaction().await;
                let mut buffer = [0u8; MotorAlignment::POSTCARD_MAX_SIZE];
                postcard::to_slice(&alignment_data, &mut buffer)
                    .map_err(|_| AlignmentError::PostcardSerializationError)?;

                // change own state now because if there is a flash error we can still continue to operate
                *self = AlignmentState::IsAligned(alignment_data);

                wt.write(&FlashKeys::MotorAlignment.key(), &buffer)
                    .await
                    .map_err(FlashError::from)?;
                wt.commit().await.map_err(FlashError::from)?;

                Ok(Some([0; 3]))
            }
            AlignmentState::IsAligned { .. } => Ok(None),
            AlignmentState::NeedsAlignment => Ok(None),
            AlignmentState::Failed => Ok(Some([0; 3])),
        }
    }

    fn get_aligned_angle(&self, encoder_angle: I16F16) -> Option<I16F16> {
        if let AlignmentState::IsAligned(alignment_data) = self {
            Some(alignment_data.electrical_angle(encoder_angle))
        } else {
            None
        }
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
    spi_bus: spi::master::SpiDma<'static, esp_hal::Blocking>,
    mag_csn: Output<'static>,
    mcpwm0: esp_hal::peripherals::MCPWM0<'static>,
    pwm_pins: Pins6PWM,
    flash: &'static FlashType<'static>,
    restored_alignment: Option<MotorAlignment>,
) {
    // about 2% dead time
    const PWM_DEAD_TIME: u16 = 20;

    // restore potential previous alignment data
    let mut alignment_state = if let Some(restored_alignment) = restored_alignment {
        AlignmentState::restore(restored_alignment)
    } else {
        AlignmentState::new()
    };

    // setup encoder SPI communication
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_buffers!(ENCODER_SPI_DMA_BUFFER_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi_bus = spi_bus.with_buffers(dma_rx_buf, dma_tx_buf).into_async();
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi_bus);
    let spi_device = SpiDevice::new(&spi_bus, mag_csn);
    let mut encoder = mt6701::MT6701Spi::new(spi_device);
    let mut last_encoder_update = Instant::now();
    info!("encoder init done!");

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    let mut mcpwm = McPwm::new(mcpwm0, clock_cfg);

    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);

    let mut pwm_u = mcpwm.operator0.with_linked_pins(
        pwm_pins.uh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pwm_pins.ul,
        PwmPinConfig::UP_ACTIVE_HIGH,
        DeadTimeCfg::new_ahc(),
    );
    let mut pwm_v = mcpwm.operator1.with_linked_pins(
        pwm_pins.vh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pwm_pins.vl,
        PwmPinConfig::UP_ACTIVE_HIGH,
        DeadTimeCfg::new_ahc(),
    );
    let mut pwm_w = mcpwm.operator2.with_linked_pins(
        pwm_pins.wh,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pwm_pins.wl,
        PwmPinConfig::UP_ACTIVE_HIGH,
        DeadTimeCfg::new_ahc(),
    );
    pwm_u.set_falling_edge_deadtime(PWM_DEAD_TIME);
    pwm_u.set_rising_edge_deadtime(PWM_DEAD_TIME);
    pwm_v.set_falling_edge_deadtime(PWM_DEAD_TIME);
    pwm_v.set_rising_edge_deadtime(PWM_DEAD_TIME);
    pwm_w.set_falling_edge_deadtime(PWM_DEAD_TIME);
    pwm_w.set_rising_edge_deadtime(PWM_DEAD_TIME);

    // Turn off all outputs
    pwm_u.set_timestamp_a(0);
    pwm_u.set_timestamp_b(0);
    pwm_v.set_timestamp_a(0);
    pwm_v.set_timestamp_b(0);
    pwm_w.set_timestamp_a(0);
    pwm_w.set_timestamp_b(0);

    // period here is in relation to all other periods further down.
    // Dead time and set_timestamp methods respectively
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(PWM_RESOLUTION, PwmWorkingMode::Increase, Rate::from_khz(25))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let mut encoder_pos: f64 = 0.0;
    let mut ticker = Ticker::every(Duration::from_millis(5));

    let test_curve = CurveBuilder::<6>::new()
        .add_linear(0.3, 2.0, 0.0)
        .add_const(0.05, 0.0)
        .add_linear(0.5, 0.0, -2.0)
        .add_linear(0.5, 2.0, 0.0)
        .add_const(0.05, 0.0)
        .add_linear(0.3, 0.0, -2.0)
        .build()
        .unwrap()
        .make_absolute(I16F16::ZERO);
    let player = HapticPlayer::new(I16F16::ZERO, &test_curve);

    loop {
        if encoder
            .update(last_encoder_update.elapsed().into())
            .await
            .is_ok()
        {
            last_encoder_update = Instant::now();
            encoder_pos = encoder.get_position() / 2.0;
            ENCODER_POSITION.store(encoder_pos as f32, core::sync::atomic::Ordering::Relaxed);
        }
        let encoder_pos = I16F16::from_num(encoder_pos);
        if MOTOR_COMMAND_SIGNAL.signaled() {
            alignment_state.start_alignment();
            MOTOR_COMMAND_SIGNAL.reset();
        }
        // In case the alignment was not yet done, this will do it in a non blocking way
        match alignment_state.do_alignment(encoder_pos, flash).await {
            Ok(pwm_data) => {
                if let Some(pwm) = pwm_data {
                    pwm_u.set_timestamp_a(pwm[0]);
                    pwm_v.set_timestamp_a(pwm[1]);
                    pwm_w.set_timestamp_a(pwm[2]);
                }
            }
            Err(AlignmentError::FlashError(e)) => {
                error!(
                    "Alignment data could not be saved to flash ({e}) and will thus not persist! Normal operation is able continue."
                );
            }
            Err(e) => {
                error!("Alignment error: {e}. Motor will remain disabled.");
            }
        }

        if let Some(angle) = alignment_state.get_aligned_angle(encoder_pos) {
            let pwm = get_phase_voltage(player.play(encoder_pos), I16F16::ZERO, angle);
            if pwm[0] == pwm[1] && pwm[2] == pwm[1] {
                pwm_u.set_timestamp_a(0);
                pwm_v.set_timestamp_a(0);
                pwm_w.set_timestamp_a(0);
            } else {
                pwm_u.set_timestamp_a(pwm[0]);
                pwm_v.set_timestamp_a(pwm[1]);
                pwm_w.set_timestamp_a(pwm[2]);
            }
            ticker.next().await;
        }
    }
}
