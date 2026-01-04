pub mod mcpwm;

pub trait MotorDriver {
    fn set_pwm(&mut self, pwm: &[u16; 3]);
}
