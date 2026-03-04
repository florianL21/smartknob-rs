use crate::{
    Angle, Command, HapticPattern, Value,
    config::{HapticInstances, HapticStates},
};

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a> {
    states: HapticStates<'a>,
    start_offset: Angle,
    scale: Value,
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

impl<'a> HapticPlayer<'a> {
    /// Create a new player state for plying back a specific curve
    /// `start_offset` defines the angle where the curve playback will start
    pub fn new(start_offset: Angle, haptic_config_instance: &'a HapticInstances) -> Self {
        HapticPlayer {
            states: haptic_config_instance.make_state(),
            start_offset,
            scale: 1.0,
        }
    }

    /// Set a scale for the output values of the player
    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }

    /// Get the current torque value given a position
    /// Note that this function is not stateless
    pub fn play(&mut self, position: Angle) -> Playback<'_> {
        let angle = position - self.start_offset;
        if let Some((ps, pl)) = &mut self.states.pattern
            && let Some(pattern) = ps.sample(pl, angle)
        {
            return Playback::Sequence(ScaledPattern::new(pattern, self.scale));
        }
        Playback::Torque(self.states.curve.sample(angle) * self.scale)
    }
}

#[cfg(test)]
mod tests {
    use matches::assert_matches;

    use crate::{CurveBuilder, CurveSegment, HapticCurveConfig, HapticPlayer, Playback};

    use super::*;
    extern crate std;

    #[test]
    fn test_builder_push_scaled_repeated_sampling_overshoot() {
        let mut builder = CurveBuilder::new();
        let segment = builder.new_segment(CurveSegment::new().add_const(1.0, 0.5));
        let curve = builder
            .push_repeated_scaled(segment, 3, 2.0)
            .finish(0.0)
            .without_pattern_layer()
            .instantiate()
            .unwrap();

        let mut player = HapticPlayer::new(0.0, &curve);
        assert_matches!(player.play(5.0), Playback::Torque(1.0));
    }

    #[test]
    fn test_builder_push_scaled_sampling_undershoot() {
        let mut builder = CurveBuilder::new();
        let segment = builder.new_segment(CurveSegment::new().add_const(1.0, 0.5));
        let curve = builder
            .push_scaled(segment, 2.0)
            .finish(0.0)
            .without_pattern_layer()
            .instantiate()
            .unwrap();

        let mut player = HapticPlayer::new(0.0, &curve);
        assert_matches!(player.play(-1.0), Playback::Torque(1.0));
    }
}
