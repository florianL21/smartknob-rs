pub trait MotorDriver<const PWM_RESOLUTION: u16> {
    fn set_pwm(&mut self, pwm: &[u16; 3]);
}
