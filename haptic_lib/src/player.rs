use crate::{Angle, Command, HapticPattern, Value, curve::CurveInstance};

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a, const N: usize> {
    curve: &'a CurveInstance<N>,
    start_offset: Angle,
    scale: Value,
    prev_angle: Value,
}

#[derive(Debug)]
pub struct ScaledPattern<'a> {
    pattern: &'a HapticPattern,
    scale: f32,
}

impl ScaledPattern<'_> {
    pub fn play(&self) -> impl Iterator<Item = Command> {
        self.pattern.play(self.scale)
    }
}

impl<'a> ScaledPattern<'a> {
    fn new(pattern: &'a HapticPattern, scale: f32) -> Self {
        Self { pattern, scale }
    }
}

#[derive(Debug)]
pub enum Playback<'a> {
    Torque(Value),
    Sequence(ScaledPattern<'a>),
}

impl<'a, const N: usize> HapticPlayer<'a, N> {
    /// Create a new player state for plying back a specific curve
    /// `start_offset` defines the angle where the curve playback will start
    pub fn new(start_offset: Angle, curve: &'a CurveInstance<N>) -> Self {
        HapticPlayer {
            curve,
            start_offset,
            scale: 1.0,
            prev_angle: start_offset,
        }
    }

    /// Set a scale for the output values of the player
    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }

    /// get the span of the whole curve
    pub fn curve_width(&self) -> Angle {
        self.curve.total_width
    }

    /// Get the current torque value given a position
    /// Note that this function is not stateless
    pub fn play(&mut self, position: Angle) -> Playback<'_> {
        let angle = position - self.start_offset;
        if angle < self.curve.start_angle {
            self.prev_angle = angle;
            return Playback::Torque(self.curve.start_value * self.scale);
        } else if angle >= self.curve.start_angle + self.curve.total_width {
            self.prev_angle = angle;
            return Playback::Torque(self.curve.end_value * self.scale);
        }

        for component in self.curve.as_iter() {
            if angle >= component.start_angle && angle < component.end_angle {
                let value = component.component.sample(angle - component.start_angle);
                self.prev_angle = angle;
                return Playback::Torque(value * self.scale);
            }
        }
        self.prev_angle = angle;
        Playback::Torque(0.0)
    }
}
