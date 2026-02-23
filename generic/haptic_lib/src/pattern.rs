extern crate alloc;

pub mod builder;

use crate::{
    Angle, Value,
    pattern::builder::{_Empty, Builder, HapticPatternBuilder},
};
use alloc::vec::Vec;
use core::{cmp::Ordering, iter, time::Duration};
use serde::{Deserialize, Serialize};
use thiserror::Error;

type CommandVec = Vec<Command>;

#[derive(Error, Debug)]
pub enum PatternLayerError {
    #[error(
        "The activation zones of the pattern at position {0} overlaps with the previous pattern"
    )]
    ActivationZoneOverlap(usize),
    #[error(
        "Got unexpected ratio for deactivation zone size. Deactivation zone ({0}) must be above 0.0 and smaller than activation zone ({1})"
    )]
    DeactivationRatioInvalid(Angle, Angle),
    #[error(
        "Torque command had value out of bounds (allowed range is -1.0 to 1.0) in pattern at position {0}"
    )]
    TorqueOutOfBounds(usize),
}

/// A single command in a command sequence
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum Command {
    /// Apply torque
    Torque(f32),
    /// Delay the next command by this duration
    /// Note: Durations are converted to microseconds internally.
    /// Anything which does not fit into an u32 will be cut away
    Delay(Duration),
}

impl Command {
    /// Create a new delay command
    pub fn delay(duration: Duration) -> Self {
        Self::Delay(duration)
    }

    /// Create a new torque command
    pub fn torque(torque: f32) -> Self {
        Self::Torque(torque)
    }

    pub(crate) fn scale(&self, value: f32) -> Self {
        match self {
            Command::Delay(d) => Self::Delay(*d),
            Command::Torque(t) => Command::Torque(t * value),
        }
    }
}

/// Definition of a haptic pattern. This returns a fixed sequence of values when played back
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct HapticPattern {
    /// Fixed values to return
    commands: CommandVec,
    /// How often to repeat the given pattern before the pattern is considered complete
    repeat: u16,
    /// How many times each command should be repeated before moving to the next command
    multiply: u16,
}

impl HapticPattern {
    /// Create a new haptic pattern
    /// - `commands` is a list torque values over which this pattern should iterate.
    ///   There is no timing information. The speed of the playback is determined by the caller.
    /// - `repeat` defines how many times the pattern should be repeated.
    /// - `multiply` defines how many times each command should be repeated before moving to the next command.
    ///   This is effectively a time compensation factor.
    pub fn new(commands: CommandVec, repeat: u16, multiply: u16) -> Self {
        Self {
            commands,
            repeat,
            multiply,
        }
    }

    pub fn builder() -> HapticPatternBuilder<_Empty> {
        HapticPatternBuilder::default()
    }

    pub fn play(&self, scale: f32) -> impl Iterator<Item = Command> {
        let num = (self.repeat * self.multiply) as usize;
        let mult = self.multiply as usize;
        self.commands
            .iter()
            .cycle()
            .flat_map(move |item| iter::repeat_n(item, mult))
            .take(num)
            .map(move |c| c.scale(scale))
    }

    /// Check if all commands within this pattern are within the range of -1.0 to 1.0
    fn bounds_valid(&self) -> bool {
        let torque_values = self.commands.iter().filter_map(|c| match c {
            Command::Torque(t) => Some(*t),
            Command::Delay(_) => None,
        });
        let max = torque_values
            .clone()
            .max_by(|x, y| x.partial_cmp(y).unwrap_or(Ordering::Equal)) // This should be fine as this check is only used for checking if values are between -1.0 and 1.0, if something is infinity it's not a valid value anyway
            .unwrap_or(0.0);
        let min = torque_values
            .min_by(|x, y| x.partial_cmp(y).unwrap_or(Ordering::Equal)) // This should be fine as this check is only used for checking if values are between -1.0 and 1.0, if something is infinity it's not a valid value anyway
            .unwrap_or(0.0);
        min >= -1.0 && max <= 1.0
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SequenceComponent {
    width: Angle,
    pattern: HapticPattern,
    repeat: usize,
}

impl SequenceComponent {
    pub fn play(&self, scale: f32) -> impl Iterator<Item = Command> {
        let num = (self.pattern.repeat * self.pattern.multiply) as usize;
        let mult = self.pattern.multiply as usize;
        self.pattern
            .commands
            .iter()
            .cycle()
            .flat_map(move |item| iter::repeat_n(item, mult))
            .take(num)
            .map(move |c| c.scale(scale))
    }
}

/// A sequence of patterns
#[derive(Serialize, Deserialize, Debug)]
pub struct PatternLayer {
    /// Individual components of this curve
    pub(crate) components: Vec<SequenceComponent>,
    /// Width of the zone which upon entering will activate a pattern
    activation_zone: Angle,
    /// Width of the activation zone
    deactivation_zone: Angle,
}

impl PatternLayer {
    /// Start building a new pattern layer definition
    pub fn builder() -> Builder<_Empty> {
        Builder::default()
    }

    /// Check the curve for validity. This makes sense to call in case of a newly deserialized curve, to ensure that it is consistent within itself.
    pub fn validate(self) -> Result<Self, PatternLayerError> {
        if self.deactivation_zone <= 0.0 || self.deactivation_zone >= self.activation_zone {
            return Err(PatternLayerError::DeactivationRatioInvalid(
                self.deactivation_zone,
                self.activation_zone,
            ));
        }
        for (i, comp) in self.components.iter().enumerate() {
            if i > 0 && comp.width < self.activation_zone {
                return Err(PatternLayerError::ActivationZoneOverlap(i));
            }
            if !comp.pattern.bounds_valid() {
                return Err(PatternLayerError::TorqueOutOfBounds(i));
            }
        }
        Ok(self)
    }

    fn iter<'a>(&'a self) -> PatternIterator<'a> {
        PatternIterator::new()
    }
}

struct PatternIterator<'a> {
    prev: Option<&'a HapticPattern>,
    index: usize,
}

impl<'a> PatternIterator<'a> {
    fn new() -> Self {
        PatternIterator {
            prev: None,
            index: 0,
        }
    }
}

struct PatternIterView<'a> {
    prev: Option<&'a HapticPattern>,
    curr: &'a HapticPattern,
    start_angle: Angle,
    stop_angle: Angle,
}

impl<'a> Iterator for PatternIterator<'a> {
    type Item = PatternIterView<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        None
    }
}

pub struct PatternLayerState<'a>(PatternIterator<'a>, &'a PatternLayer);

impl<'a> PatternLayerState<'a> {
    pub fn new(pattern_layer: &'a PatternLayer) -> Self {
        PatternLayerState(pattern_layer.iter(), pattern_layer)
    }

    pub fn sample(&self, angle: Angle) -> Option<HapticPattern> {
        None
    }
}

#[cfg(test)]
mod tests {
    use matches::assert_matches;

    use super::*;
    extern crate std;
    use crate::pattern::tests::alloc::vec;

    #[test]
    fn test_pattern_sampling_basics() {
        let layer = PatternLayer {
            activation_zone: 0.2,
            deactivation_zone: 0.15,
            components: vec![SequenceComponent {
                pattern: HapticPattern {
                    commands: vec![Command::Torque(1.1)],
                    multiply: 1,
                    repeat: 1,
                },
                width: 1.5,
                repeat: 1,
            }],
        };
        let state = PatternLayerState::new(&layer);
        assert_matches!(state.sample(1.25), None);
        assert_matches!(state.sample(1.35), Some(_));
    }
}
