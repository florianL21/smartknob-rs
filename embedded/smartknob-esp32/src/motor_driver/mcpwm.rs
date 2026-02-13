use core::marker::PhantomData;

use esp_hal::{
    gpio::interconnect::PeripheralOutput,
    mcpwm::{
        FrequencyError, McPwm, PeripheralClockConfig, PwmPeripheral,
        operator::{DeadTimeCfg, LinkedPins, PwmPinConfig},
        timer::PwmWorkingMode,
    },
    time::Rate,
};
use smartknob_core::haptic_core::motor_driver::MotorDriver;

pub struct MCPWM6<'a, PWM: PwmPeripheral, const PWM_RESOLUTION: u16> {
    // mcpwm: McPwm<'a, PWM>,
    pwm_u: LinkedPins<'a, PWM, 0>,
    pwm_v: LinkedPins<'a, PWM, 1>,
    pwm_w: LinkedPins<'a, PWM, 2>,
}

pub struct Pins6PWM<'a, Pin: PeripheralOutput<'a>> {
    uh: Pin,
    ul: Pin,
    vh: Pin,
    vl: Pin,
    wh: Pin,
    wl: Pin,
    phantom: PhantomData<&'a Pin>,
}

impl<'a, Pin: PeripheralOutput<'a>> Pins6PWM<'a, Pin> {
    pub fn new(uh: Pin, ul: Pin, vh: Pin, vl: Pin, wh: Pin, wl: Pin) -> Self {
        Self {
            uh,
            ul,
            vh,
            vl,
            wh,
            wl,
            phantom: PhantomData,
        }
    }
}

impl<'a, PWM: PwmPeripheral + 'a, const PWM_RESOLUTION: u16> MCPWM6<'a, PWM, PWM_RESOLUTION> {
    /// `pwm_dead_time` is relative to the PWM value of the `pwm_resolution`
    pub fn new<P: PeripheralOutput<'a>>(
        peripheral: PWM,
        clock_frequency: Rate,
        target_frequency: Rate,
        pwm_pins: Pins6PWM<'a, P>,
        pwm_dead_time: u16,
    ) -> Result<Self, FrequencyError> {
        let clock_cfg = PeripheralClockConfig::with_frequency(clock_frequency)?;
        let mut mcpwm = McPwm::new(peripheral, clock_cfg);

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
        pwm_u.set_falling_edge_deadtime(pwm_dead_time);
        pwm_u.set_rising_edge_deadtime(pwm_dead_time);
        pwm_v.set_falling_edge_deadtime(pwm_dead_time);
        pwm_v.set_rising_edge_deadtime(pwm_dead_time);
        pwm_w.set_falling_edge_deadtime(pwm_dead_time);
        pwm_w.set_rising_edge_deadtime(pwm_dead_time);

        // Turn off all outputs
        pwm_u.set_timestamp_a(0);
        pwm_u.set_timestamp_b(0);
        pwm_v.set_timestamp_a(0);
        pwm_v.set_timestamp_b(0);
        pwm_w.set_timestamp_a(0);
        pwm_w.set_timestamp_b(0);

        // period here is in relation to all other periods.
        // Dead time and set_timestamp methods respectively
        let timer_clock_cfg = clock_cfg.timer_clock_with_frequency(
            PWM_RESOLUTION,
            PwmWorkingMode::Increase,
            target_frequency,
        )?;
        mcpwm.timer0.start(timer_clock_cfg);
        Ok(MCPWM6 {
            pwm_u,
            pwm_v,
            pwm_w,
        })
    }
}

impl<'a, PWM: PwmPeripheral + 'a, const PWM_RESOLUTION: u16> MotorDriver<PWM_RESOLUTION>
    for MCPWM6<'a, PWM, PWM_RESOLUTION>
{
    fn set_pwm(&mut self, pwm: &[u16; 3]) {
        self.pwm_u.set_timestamp_a(pwm[0]);
        self.pwm_v.set_timestamp_a(pwm[1]);
        self.pwm_w.set_timestamp_a(pwm[2]);
    }
}
