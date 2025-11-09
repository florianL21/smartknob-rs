use core::f32;

use atomic_float::AtomicF32;
use cordic::CordicNumber;
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
use log::info;

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

pub static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);
pub static ENCODER_ANGLE: AtomicF32 = AtomicF32::new(0.0);

pub static MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand> = Signal::new();

const PWM_RESOLUTION: u16 = 999;

const _3PI_2: f32 = 3.0 * f32::consts::PI / 2.0;
const _2PI: f32 = 2.0 * f32::consts::PI;

pub struct Pins6PWM {
    pub uh: AnyPin<'static>,
    pub ul: AnyPin<'static>,
    pub vh: AnyPin<'static>,
    pub vl: AnyPin<'static>,
    pub wh: AnyPin<'static>,
    pub wl: AnyPin<'static>,
}

enum AlignmentState {
    NeedsAlignment,
    AlignForward { counter: u16 },
    AlignBackward { counter: u16, mid_angle: f32 },
    FinishAlignment { mid_angle: f32, end_angle: f32 },
    IsAligned,
}

pub enum MotorCommand {
    StartAlignment,
}

fn get_phase_voltage(alignment_voltage: f32, angle: f32) -> [u16; 3] {
    let (sin_angle, cos_angle) = cordic::sin_cos(I16F16::from_num(angle));
    let orthogonal_voltage = park_clarke::inverse_park(
        cos_angle,
        sin_angle,
        park_clarke::RotatingReferenceFrame {
            d: FixedI32::zero(),
            q: FixedI32::from_num(alignment_voltage),
        },
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
        if MOTOR_COMMAND_SIGNAL.signaled() {
            alignment_state = AlignmentState::NeedsAlignment;
        }
        match alignment_state {
            AlignmentState::NeedsAlignment => {
                MOTOR_COMMAND_SIGNAL.wait().await;
                alignment_state = AlignmentState::AlignForward { counter: 0 };
            }
            AlignmentState::AlignForward { counter } => {
                alignment_state = if counter >= 500 {
                    AlignmentState::AlignBackward {
                        counter: 500,
                        mid_angle: encoder_angle,
                    }
                } else {
                    let angle = _3PI_2 + _2PI * counter as f32 / 500.0f32;
                    info!("Aligning fw... angle: {}", angle);
                    let pwm = get_phase_voltage(4.0, angle);
                    _pwm_u.set_timestamp_a(pwm[0]);
                    _pwm_v.set_timestamp_a(pwm[1]);
                    _pwm_w.set_timestamp_a(pwm[2]);
                    AlignmentState::AlignForward {
                        counter: counter + 1,
                    }
                };
                Timer::after_millis(2).await;
            }
            AlignmentState::AlignBackward { counter, mid_angle } => {
                alignment_state = if counter == 0 {
                    AlignmentState::FinishAlignment {
                        mid_angle,
                        end_angle: encoder_angle,
                    }
                } else {
                    let angle = _3PI_2 + _2PI * counter as f32 / 500.0f32;
                    info!("Aligning bw... angle: {}", angle);
                    let pwm = get_phase_voltage(4.0, angle);
                    _pwm_u.set_timestamp_a(pwm[0]);
                    _pwm_v.set_timestamp_a(pwm[1]);
                    _pwm_w.set_timestamp_a(pwm[2]);
                    AlignmentState::AlignBackward {
                        counter: counter - 1,
                        mid_angle,
                    }
                };
                Timer::after_millis(2).await;
            }
            AlignmentState::FinishAlignment {
                mid_angle,
                end_angle,
            } => {
                _pwm_u.set_timestamp_a(0);
                _pwm_v.set_timestamp_a(0);
                _pwm_w.set_timestamp_a(0);
                info!(
                    "Alignment complete. Mid angle: {}, End angle: {}",
                    mid_angle, end_angle
                );
                alignment_state = AlignmentState::IsAligned;
            }
            AlignmentState::IsAligned => {
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
}
