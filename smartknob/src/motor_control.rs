pub mod encoder;
mod haptic_system;
pub mod motor_driver;

use atomic_float::AtomicF32;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker};
use encoder::{AbsolutePositionEncoder, MT6701Spi};
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Output},
    spi,
    time::Rate,
};
use esp_println::println;
use fixed::types::I16F16;
use foc::pwm::Modulation;
use foc::{park_clarke, pwm::SpaceVector};
use haptic_lib::{CurveBuilder, Easing, EasingType, HapticPlayer};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use log::{error, info};
use thiserror::Error;

use crate::{
    cli::KEY_PRESS_EVENTS,
    flash::{FlashError, FlashKeys, FlashType},
    motor_control::motor_driver::{
        mcpwm::{Pins6PWM, MCPWM6},
        MotorDriver,
    },
};

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
    AlignUV,
    SettleUV {
        start_time: Instant,
    },
    MeasureUV,
    AlignUW {
        uv_angle: I16F16,
    },
    SettleUW {
        start_time: Instant,
        uv_angle: I16F16,
    },
    MeasureUW {
        uv_angle: I16F16,
    },
    Calc {
        uv_angle: I16F16,
        uw_angle: I16F16,
    },
    IsAligned(MotorAlignment),
    Failed,
}

#[derive(Debug, PartialEq, Deserialize, Serialize, Clone)]
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

    async fn save_to_flash(
        &self,
        flash: &'static FlashType<'static>,
    ) -> Result<(), AlignmentError> {
        let mut wt = flash.write_transaction().await;
        let mut buffer = [0u8; MotorAlignment::POSTCARD_MAX_SIZE];
        postcard::to_slice(&self, &mut buffer)
            .map_err(|_| AlignmentError::PostcardSerializationError)?;

        wt.write(&FlashKeys::MotorAlignment.key(), &buffer)
            .await
            .map_err(FlashError::from)?;
        wt.commit().await.map_err(FlashError::from)?;
        Ok(())
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
        *self = AlignmentState::AlignUV;
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
            AlignmentState::AlignUV => {
                info!("UV alignment");
                *self = AlignmentState::SettleUV {
                    start_time: Instant::now(),
                };
                Ok(Some([400, 400, 0]))
            }
            AlignmentState::SettleUV { start_time } => {
                if start_time.elapsed().as_millis() > 300 {
                    *self = AlignmentState::MeasureUV;
                }
                Ok(None)
            }
            AlignmentState::MeasureUV => {
                *self = AlignmentState::AlignUW {
                    uv_angle: encoder_position,
                };
                Ok(None)
            }
            AlignmentState::AlignUW { uv_angle } => {
                info!("UW alignment");
                *self = AlignmentState::SettleUW {
                    start_time: Instant::now(),
                    uv_angle: *uv_angle,
                };
                Ok(Some([0, 400, 400]))
            }
            AlignmentState::SettleUW {
                start_time,
                uv_angle,
            } => {
                if start_time.elapsed().as_millis() > 300 {
                    *self = AlignmentState::MeasureUW {
                        uv_angle: *uv_angle,
                    };
                }
                Ok(None)
            }
            AlignmentState::MeasureUW { uv_angle } => {
                *self = AlignmentState::Calc {
                    uv_angle: *uv_angle,
                    uw_angle: encoder_position,
                };
                Ok(Some([0; 3]))
            }
            AlignmentState::Calc { uv_angle, uw_angle } => {
                let avg = (*uv_angle + *uw_angle) / 2;
                info!("uv: {uv_angle}, uw: {uw_angle}, avg: {avg}");
                *self = AlignmentState::IsAligned(MotorAlignment {
                    sensor_direction: SensorDirection::CW,
                    electric_zero_angle: avg,
                });
                Ok(None)
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
    pwm_pins: Pins6PWM<'static, AnyPin<'static>>,
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
    let mut encoder = MT6701Spi::new(spi_device);
    info!("encoder init done!");

    let mut motor_driver = MCPWM6::new(
        mcpwm0,
        Rate::from_mhz(32),
        pwm_pins,
        PWM_RESOLUTION,
        PWM_DEAD_TIME,
    );

    let mut encoder_pos: I16F16 = I16F16::ZERO;
    let mut ticker = Ticker::every(Duration::from_millis(5));

    let test_curve = CurveBuilder::<6>::new()
        .add_eased(0.3, 1.0, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.5, 0.0, -1.0, Easing::Cubic(EasingType::In))
        .add_eased(0.5, 1.0, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.3, 0.0, -1.0, Easing::Cubic(EasingType::In))
        .build()
        .unwrap()
        .make_absolute(I16F16::ZERO);
    let mut player = HapticPlayer::new(I16F16::ZERO, &test_curve).with_scale(2.0);

    let mut set_angle = I16F16::ZERO;
    loop {
        if let Ok(key) = KEY_PRESS_EVENTS.try_receive() {
            if let AlignmentState::IsAligned(ref mut alignment) = alignment_state {
                match key {
                    b'+' => {
                        alignment.electric_zero_angle += I16F16::from_num(0.01);
                        info!("{alignment:?}");
                    }
                    b'-' => {
                        alignment.electric_zero_angle -= I16F16::from_num(0.01);
                        info!("{alignment:?}");
                    }
                    b'w' => {
                        let res = alignment.save_to_flash(flash).await;
                        info!("Save to flash result: {res:?}");
                    }
                    _ => { /* ignore other keys */ }
                }
            }
        }
        if let Ok(meas) = encoder.update().await {
            encoder_pos = meas.position;
            ENCODER_POSITION.store(encoder_pos.to_num(), core::sync::atomic::Ordering::Relaxed);
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
                    motor_driver.set_pwm(&pwm);
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
            // info!("pos: {encoder_pos}");
            let mut ang = encoder_pos;
            while ang >= I16F16::TAU {
                ang -= I16F16::TAU;
            }

            println!(">angle:{}:{}|xy", set_angle / 6, ang);
            set_angle += I16F16::from_num(0.04);
            if set_angle >= 6 * I16F16::TAU {
                set_angle -= 6 * I16F16::TAU;
            }
            let pwm = get_phase_voltage(I16F16::ONE, I16F16::ZERO, set_angle);
            motor_driver.set_pwm(&pwm);

            // let pwm = get_phase_voltage(player.play(encoder_pos), I16F16::ZERO, angle);
            // if pwm[0] == pwm[1] && pwm[2] == pwm[1] {
            //     pwm_u.set_timestamp_a(0);
            //     pwm_v.set_timestamp_a(0);
            //     pwm_w.set_timestamp_a(0);
            // } else {
            //     pwm_u.set_timestamp_a(pwm[0]);
            //     pwm_v.set_timestamp_a(pwm[1]);
            //     pwm_w.set_timestamp_a(pwm[2]);
            // }
            ticker.next().await;
        }
    }
}
