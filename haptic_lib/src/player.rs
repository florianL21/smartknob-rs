use crate::{Angle, HapticPattern, Value, curve::AbsoluteCurve};

struct CurrHapticPattern<'a> {
    index: usize,
    pattern: &'a HapticPattern,
}

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a, const N: usize> {
    curve: &'a AbsoluteCurve<N>,
    start_offset: Angle,
    curve_width: Angle,
    scale: Value,
    prev_angle: Value,
    current_pattern: Option<CurrHapticPattern<'a>>,
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
            current_pattern: None,
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
        // If there is currently a pattern playing we always need to finish it before moving on
        if let Some(ref mut pattern) = self.current_pattern {
            pattern.index += 1;
            if let Some(v) = pattern.pattern.as_iter().nth(pattern.index) {
                self.prev_angle = angle;
                return v * self.scale;
            } else {
                self.current_pattern = None;
            }
        }
        if angle < self.curve.start_angle {
            self.prev_angle = angle;
            return self.curve.start_value * self.scale;
        } else if angle >= self.curve_width {
            self.prev_angle = angle;
            return self.curve.end_value * self.scale;
        }

        for component in self.curve.curve.as_iter() {
            if angle >= component.start_angle && angle < component.end_angle {
                // May need to start playing back the entry pattern
                let value = if let Some(pattern) = component.component.pattern()
                    && self.prev_angle < component.start_angle
                {
                    let mut iter = pattern.as_iter();
                    if let Some(v) = iter.next() {
                        self.prev_angle = angle;
                        self.current_pattern = Some(CurrHapticPattern { index: 0, pattern });
                        *v
                    } else {
                        component.component.value(angle - component.start_angle)
                    }
                } else {
                    // We found our current component
                    component.component.value(angle - component.start_angle)
                };

                self.prev_angle = angle;
                return value * self.scale;
            }
        }
        self.prev_angle = angle;
        Value::ZERO
    }
}
