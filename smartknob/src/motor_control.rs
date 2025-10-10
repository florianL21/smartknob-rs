use atomic_float::AtomicF32;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
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
use foc::{pid::PIController, pwm::SpaceVector, Foc};
use mt6701::{self, AngleSensorTrait};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use log::info;

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

pub static ENCODER_POSITION: AtomicF32 = AtomicF32::new(0.0);
pub static ENCODER_ANGLE: AtomicF32 = AtomicF32::new(0.0);

pub struct Pins6PWM {
    pub uh: AnyPin<'static>,
    pub ul: AnyPin<'static>,
    pub vh: AnyPin<'static>,
    pub vl: AnyPin<'static>,
    pub wh: AnyPin<'static>,
    pub wl: AnyPin<'static>,
}

#[embassy_executor::task]
pub async fn update_foc(
    spi_bus: &'static SpiBus1,
    mag_csn: Output<'static>,
    mcpwm0: esp_hal::peripherals::MCPWM0<'static>,
    pwm_pins: Pins6PWM,
) {
    const MAX_CURRENT: u8 = 2;
    const PWM_RESOLUTION: u16 = 999;
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
    loop {
        if encoder.update(t.elapsed().into()).await.is_ok() {
            t = Instant::now();
            let pos = encoder.get_position();
            let angle = encoder.get_angle();
            ENCODER_POSITION.store(pos as f32, core::sync::atomic::Ordering::Relaxed);
            ENCODER_ANGLE.store(angle, core::sync::atomic::Ordering::Relaxed);
        }
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
