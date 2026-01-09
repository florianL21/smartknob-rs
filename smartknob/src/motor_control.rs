pub mod encoder;
pub mod haptic_system;
pub mod motor_driver;

use atomic_float::AtomicF32;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};
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
use foc::pwm::SpaceVector;
use haptic_lib::{Command, CurveBuilder, Easing, EasingType, HapticPlayer, Playback};
use log::{error, info, warn};

pub use crate::motor_control::haptic_system::CalibrationData;
use crate::{
    flash::{FLASH_LOAD_REQUEST, FLASH_LOAD_RESPONSE, FLASH_STORE_SIGNAL},
    motor_control::{
        haptic_system::HapticSystem,
        motor_driver::mcpwm::{MCPWM6, Pins6PWM},
    },
};

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

pub static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);
pub static MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();

const ENCODER_SPI_DMA_BUFFER_SIZE: usize = 200;
const PWM_RESOLUTION: u16 = 1800;
const MOTOR_POLE_PAIRS: u8 = 7;

const ALIGNMENT_VOLTAGE: f32 = 1.0;

pub enum MotorCommand {
    StartAlignment,
    TuneAlignment(I16F16),
    VerifyEncoder,
    TuneStore,
    Beep {
        freq: I16F16,
        duration: Duration,
        volume: I16F16,
        note_offset: u32,
    },
}

#[embassy_executor::task]
pub async fn update_foc(
    spi_bus: spi::master::SpiDma<'static, esp_hal::Blocking>,
    mag_csn: Output<'static>,
    mcpwm0: esp_hal::peripherals::MCPWM0<'static>,
    pwm_pins: Pins6PWM<'static, AnyPin<'static>>,
) {
    // about 2% dead time
    const PWM_DEAD_TIME: u16 = 20;

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

    let motor_driver: MCPWM6<_, PWM_RESOLUTION> = MCPWM6::new(
        mcpwm0,
        Rate::from_mhz(40),
        Rate::from_khz(20),
        pwm_pins,
        PWM_DEAD_TIME,
    )
    .unwrap();

    let mut haptics: HapticSystem<_, _, SpaceVector, _> =
        HapticSystem::new(encoder, motor_driver, ALIGNMENT_VOLTAGE, MOTOR_POLE_PAIRS).await;

    // restore potential previous alignment data
    FLASH_LOAD_REQUEST.signal(());
    if let Some(cal) = FLASH_LOAD_RESPONSE.wait().await {
        info!("Restored motor alignment data from flash");
        haptics.restore_calibration(cal)
    }

    let test_curve = CurveBuilder::<6>::new()
        .add_eased(0.3, 1.0, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.5, 0.0, -1.0, Easing::Cubic(EasingType::In))
        .add_eased(0.5, 1.0, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.3, 0.0, -1.0, Easing::Cubic(EasingType::In))
        .build()
        .unwrap()
        .make_absolute(I16F16::ZERO);
    let mut player = HapticPlayer::new(I16F16::ZERO, &test_curve).with_scale(2.0);
    // This task intentionally does not sleep. It should run exclusively on one core
    loop {
        if let Some(sig) = MOTOR_COMMAND_SIGNAL.try_take() {
            match sig {
                MotorCommand::StartAlignment => {
                    let result = haptics.align().await;
                    if let Ok(cal_data) = result {
                        FLASH_STORE_SIGNAL.signal(*cal_data);
                    } else {
                        error!("There is no calibration data to store");
                    }

                    info!("Alignment result: {result:?}");
                    haptics.disengage();
                }
                MotorCommand::TuneAlignment(tune) => {
                    if let Some(a) = haptics.tune_alignment(tune) {
                        info!("New alignment value is: {a}");
                    } else {
                        warn!(
                            "No alignment adjustment was made. Please complete motor alignment first"
                        )
                    }
                }
                MotorCommand::VerifyEncoder => match haptics.validate_encoder().await {
                    Ok(_) => info!("Encoder validation successful!"),
                    Err(e) => warn!("encoder validation failed: {e}"),
                },
                MotorCommand::TuneStore => {
                    if let Some(cal) = haptics.get_cal_data() {
                        FLASH_STORE_SIGNAL.signal(*cal);
                    } else {
                        warn!("System was not yet calibrated. No value was stored!");
                    }
                }
                MotorCommand::Beep {
                    freq,
                    duration,
                    volume,
                    note_offset,
                } => {
                    haptics.play_tone(freq, duration, volume, note_offset);
                }
            }
        }
        if let Ok(encoder_meas) = haptics.update_encoder().await {
            ENCODER_POSITION.store(
                encoder_meas.position.to_num(),
                core::sync::atomic::Ordering::Relaxed,
            );
            let playback = player.play(encoder_meas.position);
            match playback {
                Playback::Value(v) => {
                    if let Err(e) = haptics.set_motor(encoder_meas, v) {
                        error!("Failed to set motor torque: {e}");
                    }
                }
                Playback::Sequence(p) => {
                    let commands = p.play();
                    for command in commands {
                        match command {
                            Command::Delay(d) => {
                                Timer::after(embassy_time::Duration::from_micros(
                                    d.as_micros() as u64
                                ))
                                .await
                            }
                            Command::Torque(t) => {
                                if let Ok(enc) = haptics.update_encoder().await
                                    && let Err(e) = haptics.set_motor(enc, t)
                                {
                                    error!("Failed to set motor torque: {e}");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
