use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
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
use fixed::{traits::LossyInto, types::I16F16, FixedI32};
use foc::{pid::PIController, pwm::SpaceVector, Foc};
use mt6701::{self, AngleSensorTrait};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use log::info;

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

#[derive(Clone)]
pub struct Encoder {
    pub position: f64,
    pub angle: f32,
}

pub struct Pins6PWM {
    pub uh: AnyPin,
    pub ul: AnyPin,
    pub vh: AnyPin,
    pub vl: AnyPin,
    pub wh: AnyPin,
    pub wl: AnyPin,
}

#[embassy_executor::task]
pub async fn update_foc(
    spi_bus: &'static SpiBus1,
    mag_csn: Output<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, Encoder, 2>,
    mcpwm0: esp_hal::peripherals::MCPWM0,
    pwm_pins: Pins6PWM,
) {
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
    // Turn off all outputs
    _pwm_u.set_timestamp_a(0);
    _pwm_u.set_timestamp_b(0);
    _pwm_v.set_timestamp_a(0);
    _pwm_v.set_timestamp_b(0);
    _pwm_w.set_timestamp_a(0);
    _pwm_w.set_timestamp_b(0);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(6, PwmWorkingMode::UpDown, Rate::from_khz(25))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let _fcc = PIController::new(I16F16::from_num(0.6), I16F16::from_num(0));
    let _tcc = PIController::new(I16F16::from_num(0.6), I16F16::from_num(0));

    let mut foc: Foc<SpaceVector, 16> = Foc::new(_fcc, _tcc);
    let mut angle = I16F16::from_num(0.0);
    loop {
        if encoder.update(t.elapsed().into()).await.is_ok() {
            t = Instant::now();
            let pos = encoder.get_position();
            let angle = encoder.get_angle();
            sender.send(Encoder {
                position: pos,
                angle: angle,
            });
        }
        let t = foc.update(
            [FixedI32::from_num(0), FixedI32::from_num(0)],
            angle,
            I16F16::from_num(0.5),
            I16F16::from_num(t.elapsed().as_micros()),
        );
        angle += I16F16::from_num(0.01);
        // _pwm_u.set_timestamp_a(t[0]);
        // _pwm_u.set_timestamp_b(t[0]);
        // _pwm_v.set_timestamp_a(t[1]);
        // _pwm_v.set_timestamp_b(t[1]);
        // _pwm_w.set_timestamp_a(t[2]);
        // _pwm_w.set_timestamp_b(t[2]);
        Timer::after_millis(2).await;
    }
}
