pub mod encoder;
mod haptic_system;
pub mod motor_driver;

use atomic_float::AtomicF32;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Ticker};
use encoder::MT6701Spi;
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Output},
    spi,
    time::Rate,
};
use fixed::types::I16F16;
use haptic_lib::{CurveBuilder, Easing, EasingType, HapticPlayer};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use log::{error, info};
use thiserror::Error;

use crate::{
    cli::KEY_PRESS_EVENTS,
    flash::{FlashError, FlashKeys, FlashType},
    motor_control::{
        haptic_system::HapticSystem,
        motor_driver::mcpwm::{MCPWM6, Pins6PWM},
    },
};

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

pub static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);
pub static MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();

const ENCODER_SPI_DMA_BUFFER_SIZE: usize = 200;
const PWM_RESOLUTION: u16 = 999;
const MOTOR_POLE_PAIRS: I16F16 = I16F16::lit("7");

const ALIGNMENT_VOLTAGE: I16F16 = I16F16::lit("1.0");

#[derive(Debug, PartialEq, Copy, Clone, Deserialize, Serialize, MaxSize)]
pub enum SensorDirection {
    CW,
    CCW,
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

#[derive(Debug, PartialEq, Deserialize, Serialize, Clone)]
pub struct MotorAlignment {
    sensor_direction: SensorDirection,
    electric_zero_angle: I16F16,
}

impl postcard::experimental::max_size::MaxSize for MotorAlignment {
    const POSTCARD_MAX_SIZE: usize = SensorDirection::POSTCARD_MAX_SIZE + 4; // I16F16 has 16 integer bits and 16 fractional bits = 4 bytes
}

impl MotorAlignment {
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

pub enum MotorCommand {
    StartAlignment,
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
        // AlignmentState::restore(restored_alignment)
    } else {
        // AlignmentState::new()
    };

    // setup encoder SPI communication
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_buffers!(ENCODER_SPI_DMA_BUFFER_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi_bus = spi_bus.with_buffers(dma_rx_buf, dma_tx_buf).into_async();
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi_bus);
    let spi_device = SpiDevice::new(&spi_bus, mag_csn);
    let encoder = MT6701Spi::new(spi_device);
    info!("encoder init done!");

    let motor_driver = MCPWM6::new(
        mcpwm0,
        Rate::from_mhz(32),
        pwm_pins,
        PWM_RESOLUTION,
        PWM_DEAD_TIME,
    );

    let mut haptics = HapticSystem::new(
        encoder,
        motor_driver,
        ALIGNMENT_VOLTAGE,
        MOTOR_POLE_PAIRS.to_num(),
    )
    .await;

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

    loop {
        if let Ok(key) = KEY_PRESS_EVENTS.try_receive() {
            // if let AlignmentState::IsAligned(ref mut alignment) = alignment_state {
            //     match key {
            //         b'+' => {
            //             alignment.electric_zero_angle += I16F16::from_num(0.01);
            //             info!("{alignment:?}");
            //         }
            //         b'-' => {
            //             alignment.electric_zero_angle -= I16F16::from_num(0.01);
            //             info!("{alignment:?}");
            //         }
            //         b'w' => {
            //             let res = alignment.save_to_flash(flash).await;
            //             info!("Save to flash result: {res:?}");
            //         }
            //         _ => { /* ignore other keys */ }
            //     }
            // }
        }
        if MOTOR_COMMAND_SIGNAL.signaled() {
            MOTOR_COMMAND_SIGNAL.reset();
            let result = haptics.align().await;
            info!("Alignment result: {result:?}");
            haptics.disengage();
        }
        if let Ok(encoder_meas) = haptics.run(|meas| player.play(meas.position)).await {
            ENCODER_POSITION.store(
                encoder_meas.position.to_num(),
                core::sync::atomic::Ordering::Relaxed,
            );
        }

        ticker.next().await;
    }
}
