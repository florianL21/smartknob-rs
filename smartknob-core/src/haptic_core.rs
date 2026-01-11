//! # Core haptic system of a smartknob
//! This crate provides a hardware agnostic abstraction layer for a interactive haptic system.
//! The [`SmartknobHapticCore`] struct it the main interface of this module.
//! Check its documentation to learn about constructing one and running it.
//!
//! # Additional interfaces of this module
//! This module has a few additional interfaces which need to be serviced for it to work.
//! These interfaces are usually in the form of embassy signal which need to be haled/sent.
//! ## Signals required for operation
//! - The [`FLASH_STORE_SIGNAL`] is an output of this module.
//!   It needs to be monitored by a different component in the system.
//!   If it was set the value it contains should be written to non-volatile storage
//! - The [`FLASH_LOAD_REQUEST`] is an output of this module.
//!   It needs to be monitored by a different component in the system.
//!   If it was set, this module is waiting for the [`FLASH_LOAD_RESPONSE`] signal
//!   and can not continue its startup before it arrives.
//! - The [`FLASH_LOAD_RESPONSE`] is an input to this module.
//!   It needs to be sent by a different component in the system.
//!   It needs to be set as a response to a received [`FLASH_LOAD_REQUEST`].
//!   The contents of the signal should be the loaded calibration data from
//!   non-volatile storage, or `None` if no such data exists.
//!
//! ## Other Signals
//! - The [`MOTOR_COMMAND_SIGNAL`] is an input to this module.
//!   It triggers various actions, some of which may be essential
//!   for the proper operation of this module.
pub mod encoder;
mod haptic_hardware;
pub mod motor_driver;

use encoder::AbsolutePositionEncoder;
pub use haptic_hardware::CalibrationData;
use haptic_hardware::{HapticSystem, HapticSystemError};
use motor_driver::MotorDriver;

use atomic_float::AtomicF32;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker, Timer};
use fixed::types::I16F16;
use foc::pwm::Modulation;
use haptic_lib::{AbsoluteCurve, Command, HapticPlayer, Playback};
use log::{error, info, warn};

use crate::haptic_core::haptic_hardware::InactivitySettings;

/// This signal can be set to trigger different action of the haptic system
pub static MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();
/// Signal which is sent by this component to request the persistent storage of new calibration data
/// Processing of the signal needs to be handled outside of this module
pub static FLASH_STORE_SIGNAL: Signal<CriticalSectionRawMutex, CalibrationData> = Signal::new();
/// This signal is sent by this module to request the flash handler to load the calibration data
/// from flash and send it via the [`FLASH_LOAD_RESPONSE`] signal
pub static FLASH_LOAD_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// This signal is received by this module and should be sent by the flash handler after it has received a [`FLASH_LOAD_REQUEST`]
pub static FLASH_LOAD_RESPONSE: Signal<CriticalSectionRawMutex, Option<CalibrationData>> =
    Signal::new();

/// This stores the absolute encoder position
static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);

/// Returns the absolute encoder position of the system since boot
pub fn get_encoder_position() -> f32 {
    ENCODER_POSITION.load(core::sync::atomic::Ordering::Relaxed)
}

/// The actions which the haptic system may perform
pub enum MotorCommand {
    /// Starts the alignment procedure which tries to determine encoder linearity,
    /// motor movement direction and electrical zero offset for the FOC loop.
    /// This command needs to be sent at least once on every fresh system to
    /// determine all calibration values and store them to flash.
    StartAlignment,
    /// Can be sent to tune the current zero electrical angle alignment of the FOC system.
    /// Values are relative to the currently set alignment
    TuneAlignment(I16F16),
    /// Trigger saving of a potentially modified alignment value to flash
    TuneStore,
    /// When triggered, checks the encoder for non-linearity and give a verdict
    /// if the accuracy is good enough for a satisfying operation
    VerifyEncoder,
    /// Use the motor to make a sound
    /// This is a strong work in progress and does not work very well yet!
    Beep {
        freq: I16F16,
        duration: Duration,
        volume: I16F16,
        note_offset: u32,
    },
}

pub struct SmartknobHapticCore<
    'a,
    E: AbsolutePositionEncoder,
    D: MotorDriver<PWM_RESOLUTION>,
    M: Modulation,
    const PWM_RESOLUTION: u16,
    const MAX_CURVE_ELEM: usize,
> {
    haptics: HapticSystem<E, D, M, PWM_RESOLUTION>,
    player: Option<HapticPlayer<'a, MAX_CURVE_ELEM>>,
    curve_scale: f32,
    ticker: Option<Ticker>,
}

pub struct DetailedSettings {
    /// Torque setting used for alignment procedures
    alignment_voltage: f32,
    /// Defines how the system should scale any set haptic curves.
    /// This may be useful to scale down haptics on a more powerful system for example.
    curve_scale: f32,
    /// Settings for preserving energy
    inactivity: InactivitySettings,
}

impl Default for DetailedSettings {
    fn default() -> Self {
        Self {
            alignment_voltage: 1.0,
            curve_scale: 1.0,
            inactivity: InactivitySettings::default(),
        }
    }
}

impl<
    'a,
    E: AbsolutePositionEncoder,
    D: MotorDriver<PWM_RESOLUTION>,
    M: Modulation,
    const PWM_RESOLUTION: u16,
    const MAX_CURVE_ELEM: usize,
> SmartknobHapticCore<'a, E, D, M, PWM_RESOLUTION, MAX_CURVE_ELEM>
{
    /// Create a new haptic core given an `encoder` and a motor `driver`.
    /// - The number of `pole_pairs` of the motor needs to be known at this point.
    /// - `curve_scale` defines how the system should scale any set haptic curves.
    ///   This may be useful to scale down haptics on a more powerful system for example.
    /// - `refresh_rate` can be set to None if the system allocates one whole core exclusively to running the haptic system.
    ///   If the core is shared with other tasks it is recommended to set your desired
    ///   target refresh rate by passing in a Duration::from_hz().
    ///   Please note that if the system is too slow for reaching this refresh rate this task will still consume 100% of the CPU
    pub async fn new(
        encoder: E,
        driver: D,
        pole_pairs: u8,
        refresh_rate: Option<Duration>,
        settings: DetailedSettings,
    ) -> Self {
        let mut haptics =
            HapticSystem::new(encoder, driver, settings.alignment_voltage, pole_pairs).await;
        haptics.set_inactivity_detection(settings.inactivity);
        // restore potential previous alignment data
        FLASH_LOAD_REQUEST.signal(());
        if let Some(cal) = FLASH_LOAD_RESPONSE.wait().await {
            info!("Restored motor alignment data from flash");
            haptics.restore_calibration(cal)
        }

        Self {
            haptics,
            player: None,
            curve_scale: settings.curve_scale,
            ticker: refresh_rate.map(Ticker::every),
        }
    }

    /// Set a haptic curve to play back.
    /// `curve_scale` scales the curve itself on top of the scaling value set in the system.
    /// For example if a curve is intended to feel weaker than normal this may be set to a value < 1.
    /// If no modification of the curve is desired this should be set to 1.0
    pub async fn set_curve(
        &mut self,
        curve: &'a AbsoluteCurve<MAX_CURVE_ELEM>,
        curve_scale: f32,
    ) -> Result<(), HapticSystemError<E::Error>> {
        let meas = self.haptics.update_encoder().await?;
        self.player = Some(
            HapticPlayer::new(meas.position, curve).with_scale(self.curve_scale * curve_scale),
        );
        Ok(())
    }

    /// Run the core haptic system. This should be called in a task loop endlessly
    pub async fn run(&mut self) {
        if let Some(sig) = MOTOR_COMMAND_SIGNAL.try_take() {
            match sig {
                MotorCommand::StartAlignment => {
                    let result = self.haptics.align().await;
                    if let Ok(cal_data) = result {
                        FLASH_STORE_SIGNAL.signal(*cal_data);
                    } else {
                        error!("There is no calibration data to store");
                    }

                    info!("Alignment result: {result:?}");
                    self.haptics.disengage();
                }
                MotorCommand::TuneAlignment(tune) => {
                    if let Some(a) = self.haptics.tune_alignment(tune) {
                        info!("New alignment value is: {a}");
                    } else {
                        warn!(
                            "No alignment adjustment was made. Please complete motor alignment first"
                        )
                    }
                }
                MotorCommand::VerifyEncoder => match self.haptics.validate_encoder().await {
                    Ok(_) => info!("Encoder validation successful!"),
                    Err(e) => warn!("encoder validation failed: {e}"),
                },
                MotorCommand::TuneStore => {
                    if let Some(cal) = self.haptics.get_cal_data() {
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
                    self.haptics.play_tone(freq, duration, volume, note_offset);
                }
            }
        }
        if let Ok(encoder_meas) = self.haptics.update_encoder().await {
            ENCODER_POSITION.store(
                encoder_meas.position.to_num(),
                core::sync::atomic::Ordering::Relaxed,
            );
            if let Some(ref mut player) = self.player {
                let playback = player.play(encoder_meas.position);
                match playback {
                    Playback::Value(v) => {
                        if let Err(e) = self.haptics.set_motor(encoder_meas, v) {
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
                                    if let Ok(enc) = self.haptics.update_encoder().await
                                        && let Err(e) = self.haptics.set_motor(enc, t)
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
        if let Some(ref mut ticker) = self.ticker {
            ticker.next().await;
        }
    }
}
