use log::error;
use rhai::Scope;

use crate::haptics::base::config::ConfigInstance;

use super::{Angle, Command, HapticPattern, Value, config::HapticStates};

/// Maximum number of variables allowed for the script execution scope
const MAX_NUM_VARIABLES: usize = 200;

/// This struct holds the state for playing back a haptic curve and it is the main interface for playing back haptic curves
pub struct HapticPlayer<'a> {
    states: HapticStates<'a>,
    start_offset: Angle,
    scale: Value,
    scope: Scope<'a>,
    run_scripts: bool,
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
    pub fn new(start_offset: Angle, haptic_config_instance: &'a ConfigInstance) -> Self {
        HapticPlayer {
            states: haptic_config_instance.make_state(),
            start_offset,
            scale: 1.0,
            scope: Scope::new(),
            run_scripts: true,
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
        match self.states {
            HapticStates::Layers {
                ref mut curve,
                ref mut pattern,
            } => {
                if let Some((ps, pl)) = pattern
                    && let Some(pattern) = ps.sample(pl, angle)
                {
                    return Playback::Sequence(ScaledPattern::new(pattern, self.scale));
                }
                Playback::Torque(curve.sample(angle) * self.scale)
            }
            HapticStates::Program { engine, ast } => {
                if self.run_scripts {
                    match engine.eval_ast_with_scope::<f32>(&mut self.scope, ast) {
                        Ok(v) => {
                            if self.scope.len() > MAX_NUM_VARIABLES {
                                error!(
                                    "The number of defined variables has exceeded the maximum allowed of {MAX_NUM_VARIABLES}.\n
                                    Make sure to reuse previously defined variables and make use of is_def_var check if variables already exist.\n
                                    This is a safety mechanism to avoid system crashes.\n
                                    Further execution of scripts has been disabled.\n
                                    To re-enable it push a new haptic program."
                                );
                                self.run_scripts = false;
                            }
                            return Playback::Torque(v);
                        }
                        Err(e) => {
                            error!("Failed to evaluate script: {e}");
                        }
                    }
                }
                Playback::Torque(0.0)
            }
        }
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
