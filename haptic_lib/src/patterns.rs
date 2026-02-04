use core::{iter, time::Duration};

use fixed::types::I16F16;
use heapless::Vec;
use serde::{Deserialize, Serialize};

pub const MAX_PATTERN_COMMANDS: usize = 6;

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

type CommandVec = Vec<Command, MAX_PATTERN_COMMANDS>;

/// Definition of a haptic pattern. This returns a fixed sequence of values when played back
#[derive(Serialize, Deserialize, Debug)]
pub struct HapticPattern {
    /// Fixed values to return
    commands: CommandVec,
    /// How often to repeat the given pattern before the pattern is considered complete
    repeat: u16,
    /// How many times each command should be repeated before moving to the next command
    multiply: u16,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum SequenceComponent {
    /// Pattern zone which will play back a haptic pattern upon entry.
    /// Patterns have zero width in the view of the sequence,
    /// so they need to be spaced apart by [`SequenceComponent::Nothing`]
    Pattern {
        /// Defines how much this zone spills out onto its neighbors on both sides symmetrically
        spill: I16F16,
        /// Fixed values to return
        commands: CommandVec,
        /// How often to repeat the given pattern before the pattern is considered complete
        repeat: u16,
        /// How many times each command should be repeated before moving to the next command
        multiply: u16,
    },
    /// Empty space in a Sequence. `space`
    Nothing {
        /// defines how much distance this component covers
        space: I16F16,
    },
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

/// A sequence of patterns
pub struct PatternSequence<const N: usize> {
    /// Individual components of this curve
    pub(crate) components: Vec<SequenceComponent, N>,
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

impl SequenceComponent {
    /// Create a new haptic pattern
    /// - `commands` is a list torque values over which this pattern should iterate.
    ///   There is no timing information. The speed of the playback is determined by the caller.
    /// - `repeat` defines how many times the pattern should be repeated.
    /// - `multiply` defines how many times each command should be repeated before moving to the next command.
    ///   This is effectively a time compensation factor.
    pub fn _pattern(commands: CommandVec, repeat: u16, multiply: u16, spill: f32) -> Self {
        Self::Pattern {
            spill: I16F16::from_num(spill),
            commands,
            repeat,
            multiply,
        }
    }

    pub fn _play(&self, scale: f32) -> Option<impl Iterator<Item = Command>> {
        match self {
            Self::Pattern {
                commands,
                repeat,
                multiply,
                ..
            } => {
                let num = (repeat * multiply) as usize;
                let mult = *multiply as usize;
                Some(
                    commands
                        .iter()
                        .cycle()
                        .flat_map(move |item| iter::repeat_n(item, mult))
                        .take(num)
                        .map(move |c| c.scale(scale)),
                )
            }
            Self::Nothing { .. } => None,
        }
    }
}
