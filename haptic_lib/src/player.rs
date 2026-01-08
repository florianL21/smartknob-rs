use fixed::types::I16F16;

use crate::{Angle, Command, HapticPattern, Value, curve::AbsoluteCurve};

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a, const N: usize> {
    curve: &'a AbsoluteCurve<N>,
    start_offset: Angle,
    curve_width: Angle,
    scale: Value,
    prev_angle: Value,
}

pub struct ScaledPattern<'a> {
    pattern: &'a HapticPattern,
    scale: I16F16,
}

impl ScaledPattern<'_> {
    pub fn play(&self) -> impl Iterator<Item = Command> {
        self.pattern.play(self.scale)
    }
}

impl<'a> ScaledPattern<'a> {
    fn new(pattern: &'a HapticPattern, scale: I16F16) -> Self {
        Self { pattern, scale }
    }
}

pub enum Playback<'a> {
    Value(Value),
    Sequence(ScaledPattern<'a>),
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
            prev_angle: start_offset,
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
    pub fn play(&mut self, position: Angle) -> Playback<'_> {
        let angle = position - self.start_offset;
        if angle < self.curve.start_angle {
            self.prev_angle = angle;
            return Playback::Value(self.curve.start_value * self.scale);
        } else if angle >= self.curve_width {
            self.prev_angle = angle;
            return Playback::Value(self.curve.end_value * self.scale);
        }

        for component in self.curve.curve.as_iter() {
            if angle >= component.start_angle && angle < component.end_angle {
                // May need to start playing back the entry pattern
                let playback = if let Some(pattern) = component.component.pattern()
                    && self.prev_angle < component.start_angle
                {
                    Playback::Sequence(ScaledPattern::new(pattern, self.scale))
                } else {
                    // We found our current component
                    let value = component.component.value(angle - component.start_angle);
                    Playback::Value(value * self.scale)
                };

                self.prev_angle = angle;
                return playback;
            }
        }
        self.prev_angle = angle;
        Playback::Value(Value::ZERO)
    }
}
