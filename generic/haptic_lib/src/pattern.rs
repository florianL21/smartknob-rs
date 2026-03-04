extern crate alloc;

pub mod builder;

use crate::{
    Angle,
    pattern::builder::{_Empty, Builder, HapticPatternBuilder},
};
use alloc::vec::Vec;
use core::{
    cmp::Ordering,
    iter::{self, once},
    time::Duration,
};
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
        "Got unexpected ratio for deactivation zone size. Deactivation zone ({0}) must be above 0.0 and larger than activation zone ({1})"
    )]
    DeactivationRatioInvalid(Angle, Angle),
    #[error(
        "Torque command had value out of bounds (allowed range is -1.0 to 1.0) in pattern at position {0}"
    )]
    TorqueOutOfBounds(usize),
}

/// A single command in a command sequence
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
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

impl SequenceComponent {}

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
        if self.deactivation_zone <= 0.0 || self.activation_zone >= self.deactivation_zone {
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

    /// get the width of the whole pattern layer
    pub fn width(&self) -> Angle {
        self.iter().map(|c| c.width).sum()
    }

    pub(crate) fn make_state<'a>(&'a self) -> PatternLayerState<'a> {
        PatternLayerState::new(self)
    }

    fn get(&self, index: usize) -> Option<&SequenceComponent> {
        let mut remaining_index = index;
        for comp in self.components.iter() {
            if remaining_index > comp.repeat {
                remaining_index -= comp.repeat;
            } else {
                return Some(comp);
            }
        }
        None
    }

    fn len(&self) -> usize {
        self.components.iter().map(|c| c.repeat).sum()
    }

    fn iter(&self) -> impl Iterator<Item = &SequenceComponent> {
        self.components
            .iter()
            .flat_map(|c| once(c).cycle().take(c.repeat))
    }

    fn rev_iter(&self) -> impl Iterator<Item = &SequenceComponent> {
        self.components
            .iter()
            .rev()
            .flat_map(|c| once(c).cycle().take(c.repeat))
    }
}

#[derive(Debug, PartialEq)]
enum ActiveZone {
    Upper,
    Lower,
    None,
}

#[derive(Debug)]
pub struct PatternLayerState<'a> {
    prev: Option<&'a HapticPattern>,
    curr: &'a SequenceComponent,
    index: usize,
    layer_max_index: usize,
    start_angle: Angle,
    active_zone: ActiveZone,
}

fn check_within<'a>(
    angle: Angle,
    comp: &'a SequenceComponent,
    prev: Option<&'a HapticPattern>,
    start_angle: Angle,
    stop_angle: Angle,
) -> Option<PatternIterView<'a>> {
    if angle >= start_angle && angle <= stop_angle {
        return Some(PatternIterView {
            curr: &comp.pattern,
            prev,
            start_angle,
            stop_angle,
        });
    }
    None
}

impl<'a> PatternLayerState<'a> {
    fn new(layer: &'a PatternLayer) -> Self {
        // TODO: Think about what happens if array is empty. Probably should be disallowed by the builder
        PatternLayerState {
            prev: None,
            curr: &layer.components[0],
            layer_max_index: layer.len() - 1,
            index: 0,
            start_angle: 0.0,
            active_zone: ActiveZone::None,
        }
    }

    fn find(&mut self, layer: &'a PatternLayer, angle: Angle) -> Option<PatternIterView<'a>> {
        let stop_angle = self.start_angle + self.curr.width;
        if let Some(curr) = check_within(angle, self.curr, self.prev, self.start_angle, stop_angle)
        {
            // No need to do anything. We are already on the correct component
            return Some(curr);
        } else if angle > stop_angle {
            // Need to search forward
            if self.index >= self.layer_max_index {
                // the current index is already at maximum
                return None;
            }
            self.index += 1;
            self.start_angle = stop_angle;
            for comp in layer.iter().skip(self.index) {
                self.prev = layer.get(self.index - 1).map(|c| &c.pattern);
                self.curr = comp;
                let stop_angle = self.start_angle + comp.width;
                if let Some(curr) =
                    check_within(angle, self.curr, self.prev, self.start_angle, stop_angle)
                {
                    return Some(curr);
                }
                self.start_angle = stop_angle;
                self.index += 1;
            }
        } else if angle < self.start_angle {
            // Need to search backwards
            if self.index == 0 {
                // the current index is already at zero, it would underflow
                return None;
            }
            self.index -= 1;
            for comp in layer.rev_iter().skip(self.layer_max_index - self.index) {
                self.start_angle -= self.curr.width;
                self.prev = if self.index == 0 {
                    None
                } else {
                    layer.get(self.index - 1).map(|c| &c.pattern)
                };
                self.curr = comp;
                let stop_angle = self.start_angle + comp.width;
                if let Some(curr) =
                    check_within(angle, self.curr, self.prev, self.start_angle, stop_angle)
                {
                    return Some(curr);
                }
            }
        }

        None
    }

    pub fn sample(&mut self, layer: &'a PatternLayer, angle: Angle) -> Option<&HapticPattern> {
        if let Some(comp) = self.find(layer, angle) {
            // First drop out of any active zone
            match self.active_zone {
                ActiveZone::Lower => {
                    let lower_deactivation_zone = comp.start_angle + layer.deactivation_zone / 2.0;
                    if angle > lower_deactivation_zone {
                        self.active_zone = ActiveZone::None;
                    }
                }
                ActiveZone::Upper => {
                    let upper_deactivation_zone = comp.stop_angle - layer.deactivation_zone / 2.0;
                    if angle < upper_deactivation_zone {
                        self.active_zone = ActiveZone::None;
                    }
                }
                _ => {}
            }
            match self.active_zone {
                ActiveZone::None => {
                    let lower_activation_zone = comp.start_angle + layer.activation_zone / 2.0;
                    if angle < lower_activation_zone {
                        self.active_zone = ActiveZone::Lower;
                        return comp.prev;
                    }
                    let upper_activation_zone = comp.stop_angle - layer.activation_zone / 2.0;
                    if angle > upper_activation_zone {
                        self.active_zone = ActiveZone::Upper;
                        return Some(comp.curr);
                    }
                }
                ActiveZone::Lower => {
                    return None;
                }
                ActiveZone::Upper => {
                    return None;
                }
            }
        }
        None
    }
}

#[derive(Debug)]
struct PatternIterView<'a> {
    prev: Option<&'a HapticPattern>,
    curr: &'a HapticPattern,
    start_angle: Angle,
    stop_angle: Angle,
}

#[cfg(test)]
mod tests {
    use matches::assert_matches;

    use super::*;
    extern crate std;
    use crate::pattern::tests::alloc::vec;

    macro_rules! assert_haptic_pattern {
        ($expression:expr, $($pattern:tt)+) => {
            let v = $expression;
            assert_matches!(v, Some(_));
            assert_matches!(v.unwrap().commands[..], $($pattern)+);
        };
    }

    #[test]
    fn test_pattern_find() {
        let layer = PatternLayer {
            activation_zone: 0.2,
            deactivation_zone: 0.25,
            components: vec![SequenceComponent {
                pattern: HapticPattern {
                    commands: vec![Command::Torque(1.1)],
                    multiply: 1,
                    repeat: 1,
                },
                width: 1.5,
                repeat: 2,
            }],
        };
        let mut state = layer.make_state();
        assert_matches!(
            state.find(&layer, 0.0),
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.5,
                ..
            })
        );
        assert_matches!(
            state.find(&layer, 1.5),
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.5,
                ..
            })
        );
        assert_matches!(
            state.find(&layer, 2.0),
            Some(PatternIterView {
                start_angle: 1.5,
                stop_angle: 3.0,
                ..
            })
        );
        assert_matches!(state.find(&layer, 3.5), None);
        assert_matches!(
            state.find(&layer, 1.0),
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.5,
                ..
            })
        );
    }

    #[test]
    fn test_pattern_find_overflow() {
        let layer = PatternLayer {
            activation_zone: 0.2,
            deactivation_zone: 0.25,
            components: vec![SequenceComponent {
                pattern: HapticPattern {
                    commands: vec![Command::Torque(1.1)],
                    multiply: 1,
                    repeat: 0,
                },
                width: 1.0,
                repeat: 4,
            }],
        };
        let mut state = layer.make_state();
        // sample once at the start of the layer
        assert_matches!(
            state.find(&layer, 0.5),
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.0,
                ..
            })
        );
        // sample outside of range a few times
        assert_matches!(state.find(&layer, 5.0), None);
        assert_matches!(state.find(&layer, 10.0), None);
        assert_matches!(state.find(&layer, 15.0), None);
        assert_matches!(state.find(&layer, 20.0), None);
        // come back into range
        assert_matches!(
            state.find(&layer, 1.5),
            Some(PatternIterView {
                start_angle: 1.0,
                stop_angle: 2.0,
                ..
            })
        );
    }

    #[test]
    fn test_pattern_find_underflow() {
        let layer = PatternLayer {
            activation_zone: 0.2,
            deactivation_zone: 0.25,
            components: vec![SequenceComponent {
                pattern: HapticPattern {
                    commands: vec![Command::Torque(1.1)],
                    multiply: 1,
                    repeat: 0,
                },
                width: 1.0,
                repeat: 4,
            }],
        };
        let mut state = layer.make_state();
        // sample once at the start of the layer
        assert_matches!(
            state.find(&layer, 0.5),
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.0,
                ..
            })
        );
        // sample outside of range a few times
        assert_matches!(state.find(&layer, -1.0), None);
        assert_matches!(state.find(&layer, -5.0), None);
        assert_matches!(state.find(&layer, -8.0), None);
        assert_matches!(state.find(&layer, -10.0), None);
        // come back into range
        assert_matches!(
            state.find(&layer, 1.5),
            Some(PatternIterView {
                start_angle: 1.0,
                stop_angle: 2.0,
                ..
            })
        );
    }

    #[test]
    fn test_pattern_find_over_and_underflow() {
        let layer = PatternLayer {
            activation_zone: 0.2,
            deactivation_zone: 0.25,
            components: vec![SequenceComponent {
                pattern: HapticPattern {
                    commands: vec![Command::Torque(1.1)],
                    multiply: 1,
                    repeat: 0,
                },
                width: 1.0,
                repeat: 4,
            }],
        };
        let mut state = layer.make_state();
        // sample once at the start of the layer
        assert_matches!(
            state.find(&layer, 0.5),
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.0,
                ..
            })
        );
        // sample underneith the curve
        assert_matches!(state.find(&layer, -5.0), None);
        assert_matches!(state.find(&layer, -10.0), None);
        assert_matches!(state.find(&layer, -15.0), None);
        assert_matches!(state.find(&layer, -20.0), None);
        // come back into range
        assert_matches!(
            state.find(&layer, 1.5),
            Some(PatternIterView {
                start_angle: 1.0,
                stop_angle: 2.0,
                ..
            })
        );
        assert_matches!(state.find(&layer, 5.0), None);
        assert_matches!(state.find(&layer, 10.0), None);
        assert_matches!(state.find(&layer, 15.0), None);
        assert_matches!(state.find(&layer, 20.0), None);
        // come back into range
        assert_matches!(
            state.find(&layer, 1.5),
            Some(PatternIterView {
                start_angle: 1.0,
                stop_angle: 2.0,
                ..
            })
        );
    }

    #[test]
    fn test_pattern_find_previous() {
        let layer = PatternLayer {
            activation_zone: 0.2,
            deactivation_zone: 0.25,
            components: vec![
                SequenceComponent {
                    pattern: HapticPattern {
                        commands: vec![Command::Torque(1.5)],
                        multiply: 1,
                        repeat: 1,
                    },
                    width: 0.0,
                    repeat: 1,
                },
                SequenceComponent {
                    pattern: HapticPattern {
                        commands: vec![Command::Torque(1.1)],
                        multiply: 1,
                        repeat: 1,
                    },
                    width: 1.5,
                    repeat: 2,
                },
            ],
        };
        let mut state = layer.make_state();
        let res = state.find(&layer, 0.1);
        assert_matches!(
            res,
            Some(PatternIterView {
                start_angle: 0.0,
                stop_angle: 1.5,
                prev: Some(_),
                ..
            })
        );
        assert_haptic_pattern!(res.as_ref().unwrap().prev, [Command::Torque(1.5)]);
    }

    #[test]
    fn test_pattern_sampling_basics() {
        let layer = PatternLayer {
            activation_zone: 0.15,
            deactivation_zone: 0.2,
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
        let mut state = layer.make_state();
        // |<------- 1.5 ------->|
        // **#-----------------#**
        //    |<---- 1.3 ---->|
        // |<------- 1.4 ---->|
        assert_matches!(state.sample(&layer, 1.4), None);
        assert_matches!(state.sample(&layer, 1.46), Some(_));
    }

    #[test]
    fn test_pattern_with_nothing_at_lower_bound() {
        let layer = PatternLayer {
            activation_zone: 0.15,
            deactivation_zone: 0.2,
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
        let mut state = layer.make_state();
        assert_matches!(state.sample(&layer, 0.06), None);
    }

    #[test]
    fn test_pattern_with_zero_width_first_element() {
        let layer = PatternLayer {
            activation_zone: 0.15,
            deactivation_zone: 0.2,
            components: vec![
                SequenceComponent {
                    pattern: HapticPattern {
                        commands: vec![Command::Torque(1.5)],
                        multiply: 1,
                        repeat: 1,
                    },
                    width: 0.0,
                    repeat: 1,
                },
                SequenceComponent {
                    pattern: HapticPattern {
                        commands: vec![Command::Torque(1.1)],
                        multiply: 1,
                        repeat: 1,
                    },
                    width: 1.5,
                    repeat: 1,
                },
            ],
        };
        let mut state = layer.make_state();
        assert_haptic_pattern!(state.sample(&layer, 0.06), [Command::Torque(1.5)]);
    }

    #[test]
    fn test_zones_with_pattern_with_zero_width_as_first_element() {
        let layer = PatternLayer {
            activation_zone: 0.15,
            deactivation_zone: 0.2,
            components: vec![SequenceComponent {
                pattern: HapticPattern {
                    commands: vec![Command::Torque(1.5)],
                    multiply: 1,
                    repeat: 1,
                },
                width: 0.0,
                repeat: 1,
            }],
        };
        let mut state = layer.make_state();
        assert_haptic_pattern!(state.sample(&layer, 0.06), [Command::Torque(1.5)]);
        assert_matches!(state.sample(&layer, 0.5), None);
        assert_haptic_pattern!(state.sample(&layer, -0.06), [Command::Torque(1.5)]);
    }

    #[test]
    fn test_pattern_activation_deactivation_mechanic() {
        let layer = PatternLayer {
            activation_zone: 0.15,
            deactivation_zone: 0.2,
            components: vec![
                SequenceComponent {
                    pattern: HapticPattern {
                        commands: vec![Command::Torque(1.5)],
                        multiply: 1,
                        repeat: 1,
                    },
                    width: 0.0,
                    repeat: 1,
                },
                SequenceComponent {
                    pattern: HapticPattern {
                        commands: vec![Command::Torque(1.1)],
                        multiply: 1,
                        repeat: 1,
                    },
                    width: 1.5,
                    repeat: 1,
                },
            ],
        };
        let mut state = layer.make_state();
        // State starts off as being outside any zone
        assert_matches!(state.sample(&layer, 0.4), None);
        // entering into the activation zone for the first time will yield the pattern
        assert_haptic_pattern!(state.sample(&layer, 0.06), [Command::Torque(1.5)]);
        // moving within a zone will not yield the pattern again
        assert_matches!(state.sample(&layer, 0.05), None);
        // same goes for moves across the boundary of the activation and deactivation zone
        assert_matches!(state.sample(&layer, 0.1), None);
        assert_matches!(state.sample(&layer, 0.05), None);
        // Leaving the zone
        assert_matches!(state.sample(&layer, 0.3), None);
        // Entering the zone again will yield a pattern
        assert_haptic_pattern!(state.sample(&layer, 0.06), [Command::Torque(1.5)]);
        // Entering a zone while being in a different zone will also yield its pattern
        assert_haptic_pattern!(state.sample(&layer, 1.45), [Command::Torque(1.1)]);
    }
}
