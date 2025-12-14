use crate::{curve::AbsoluteCurve, Angle, Value};

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a, const N: usize> {
    curve: &'a AbsoluteCurve<N>,
    start_offset: Angle,
    curve_width: Angle,
    scale: Value,
}

impl<'a, const N: usize> HapticPlayer<'a, N> {
    /// Create a new player state for plying back a specific curve
    /// `start_offset` defines the angle where the curve playback will start
    pub fn new(start_offset: Angle, curve: &'a AbsoluteCurve<N>) -> Self {
        let curve_width = curve.curve.components.iter().map(|c| c.width()).sum();
        HapticPlayer {
            curve,
            start_offset,
            curve_width,
            scale: Value::ONE,
        }
    }

    /// Set a scale for the output values of the player
    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = Value::from_num(scale);
        self
    }

    /// get the span of the whole curve
    pub fn curve_width(&self) -> Angle {
        self.curve_width
    }

    /// Get the current torque value given a position
    /// Note that this function is not stateless
    pub fn play(&mut self, position: Angle) -> Value {
        let angle = position - self.start_offset;
        if angle < self.curve.start_angle {
            return self.curve.start_value * self.scale;
        } else if angle >= self.curve_width {
            return self.curve.end_value * self.scale;
        }
        for component in self.curve.curve.as_iter() {
            if angle >= component.start_angle && angle < component.end_angle {
                // We found our current component
                return component.component.value(angle - component.start_angle) * self.scale;
            }
        }
        Value::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    #[test]
    fn test_basic_curve() {
        let test_curve = CurveBuilder::<5>::new()
            .add_const(2.0, 5.0)
            .build()
            .unwrap()
            .make_absolute(I16F16::ZERO);
        let player = HapticPlayer::new(I16F16::ZERO, &test_curve);
        assert_eq!(player.play(I16F16::from_num(-1)), I16F16::ZERO);
        assert_eq!(player.play(I16F16::from_num(1)), I16F16::from_num(5.0));
        assert_eq!(player.play(I16F16::from_num(3)), I16F16::ZERO);
    }
}
