use crate::{
    Angle, Command, HapticPattern, Value,
    curve::{CurveInstance, CurveState},
};

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a, const N: usize> {
    curve: CurveState<'a>,
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
    fn _new(pattern: &'a HapticPattern, scale: f32) -> Self {
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
    pub fn new(start_offset: Angle, curve: &'a CurveInstance) -> Self {
        HapticPlayer {
            curve: CurveState::new(curve),
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
        Playback::Torque(self.curve.sample(angle) * self.scale)
    }
}
