extern crate alloc;

use crate::Angle;
use alloc::vec::Vec;
use core::{iter, marker::PhantomData, time::Duration};
use serde::{Deserialize, Serialize};

type CommandVec = Vec<Command>;

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
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SequenceComponent {
    width: Angle,
    pattern: HapticPattern,
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
    /// percentage of the activation zone < 1.0
    deactivation_zone: Angle,
}

impl PatternLayer {
    /// Start building a new pattern layer definition
    pub fn builder() -> Builder<_Empty> {
        Builder::default()
    }
}

/// Marker for a builder which does not have any components yet
#[derive(Default)]
pub struct _Empty {}
/// Marker for a builder which has at least one component
pub struct _NonEmpty {}

/// A builder for creating a haptic pattern lr definition
#[derive(Default)]
pub struct Builder<T> {
    /// Current list of components already built
    components: Vec<SequenceComponent>,
    phantom: PhantomData<T>,
}

impl<T> Builder<T> {
    /// Add some space before the next pattern,
    /// `width` must be larger than
    pub fn with_space(self, width: Angle) -> SpaceBuilder<_NonEmpty, _Empty> {
        SpaceBuilder {
            root_builder: Builder {
                components: self.components,
                phantom: PhantomData::default(),
            },
            width,
            commands: Vec::new(),
            phantom: PhantomData::default(),
        }
    }

    /// Finally build the new layer.
    /// - `activation_zone` is the width of the zone which upon entering will activate a pattern
    /// - `deactivation_zone` is the percentage of the activation zone and must be below 1.0
    /// Note that the `activation_zone` is an absolute value in radiants while the `deactivation_zone` is a percentage of the `activation_zone`.
    pub fn build(self, activation_zone: Angle, deactivation_zone: Angle) -> PatternLayer {
        PatternLayer {
            components: self.components,
            activation_zone,
            deactivation_zone,
        }
    }
}

impl Builder<_Empty> {
    /// Add a new pattern to the zero position of the layer
    pub fn at_zero(self) -> SpaceBuilder<_Empty, _Empty> {
        SpaceBuilder {
            root_builder: self,
            width: 0.0,
            commands: Vec::new(),
            phantom: PhantomData::default(),
        }
    }
}

/// Builder state in the middle of building a new haptic pattern with context
/// of how wide the space before the next pattern should be
pub struct SpaceBuilder<T, M> {
    root_builder: Builder<T>,
    width: Angle,
    commands: CommandVec,
    phantom: PhantomData<M>,
}

impl<T, M> SpaceBuilder<T, M> {
    /// Add a new torque command to this sequence
    /// `torque` must be in the range of -1.0 to 1.0
    pub fn torque(mut self, torque: f32) -> SpaceBuilder<T, _NonEmpty> {
        self.commands.push(Command::torque(torque));
        SpaceBuilder {
            commands: self.commands,
            root_builder: self.root_builder,
            width: self.width,
            phantom: PhantomData::default(),
        }
    }
}

impl<T> SpaceBuilder<T, _NonEmpty> {
    /// Add a new delay command to this sequence
    /// The given duration will delay the execution of the next torque command by the given `duration`
    pub fn delay(mut self, duration: Duration) -> SpaceBuilder<T, _NonEmpty> {
        self.commands.push(Command::delay(duration));
        SpaceBuilder {
            commands: self.commands,
            root_builder: self.root_builder,
            width: self.width,
            phantom: PhantomData::default(),
        }
    }

    /// This function finalizes the current pattern and repeats it `n` times
    pub fn repeated(self, n: u16) -> SequenceBuilder<T> {
        SequenceBuilder {
            root_builder: self.root_builder,
            sequence: SequenceComponent {
                width: self.width,
                pattern: HapticPattern {
                    commands: self.commands,
                    repeat: n,
                    multiply: 1,
                },
            },
        }
    }

    /// This function finalizes the current pattern and duplicated every item within it `n` times in place.
    /// This can be used to "stretch" a pattern.
    pub fn multiplied(self, n: u16) -> SequenceBuilder<T> {
        SequenceBuilder {
            root_builder: self.root_builder,
            sequence: SequenceComponent {
                width: self.width,
                pattern: HapticPattern {
                    commands: self.commands,
                    repeat: 1,
                    multiply: n,
                },
            },
        }
    }

    /// This function finalizes the current pattern and applies both multiply and repeat modifiers.
    /// Check out [`Self::repeated`] and [`Self::multiplied`] for more information
    pub fn multiplied_and_repeated(self, repeat: u16, multiply: u16) -> SequenceBuilder<T> {
        SequenceBuilder {
            root_builder: self.root_builder,
            sequence: SequenceComponent {
                width: self.width,
                pattern: HapticPattern {
                    commands: self.commands,
                    repeat,
                    multiply,
                },
            },
        }
    }

    /// Finish building this command sequence
    pub fn finish(self) -> SequenceBuilder<T> {
        SequenceBuilder {
            root_builder: self.root_builder,
            sequence: SequenceComponent {
                width: self.width,
                pattern: HapticPattern {
                    commands: self.commands,
                    repeat: 1,
                    multiply: 1,
                },
            },
        }
    }
}

/// Builder with context about the next pattern which is about to be added
pub struct SequenceBuilder<T> {
    root_builder: Builder<T>,
    sequence: SequenceComponent,
}

impl SequenceBuilder<_NonEmpty> {
    /// Repeat the whole component which was just constructed `n` times.
    /// This repeats everything, including the space before the pattern as well.
    pub fn insert_repeated(mut self, n: usize) -> Builder<_NonEmpty> {
        for _ in 0..n {
            self.root_builder.components.push(self.sequence.clone());
        }
        self.root_builder
    }
}

impl<T> SequenceBuilder<T> {
    /// Insert the just constructed component once into the layer
    pub fn insert_once(mut self) -> Builder<_NonEmpty> {
        self.root_builder.components.push(self.sequence);
        Builder {
            components: self.root_builder.components,
            phantom: PhantomData::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::builder::CurveComponent;

    use super::*;
    extern crate std;
    use alloc::vec;
    use matches::assert_matches;

    #[test]
    fn test_builder_zero_width_as_first_element() {
        let layer = PatternLayer::builder()
            .at_zero()
            .torque(-1.0)
            .torque(1.0)
            .finish()
            .insert_once()
            .build(0.2, 0.8);
    }

    #[test]
    fn test_builder_transition_from_zero_width() {
        let layer = PatternLayer::builder()
            .at_zero()
            .torque(1.0)
            .finish()
            .insert_once()
            .with_space(2.0)
            .torque(0.5)
            .torque(0.0)
            .repeated(2)
            .insert_repeated(10)
            .build(0.2, 0.8);
    }
}
