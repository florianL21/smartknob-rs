use super::encoder::AbsolutePositionEncoder;
use super::motor_driver::MotorDriver;
struct HapticSystem<E, D>
where
    E: AbsolutePositionEncoder,
    D: MotorDriver,
{
    encoder: E,
    motor_driver: D,
}
