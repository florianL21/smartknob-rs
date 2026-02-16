extern crate alloc;

pub mod builder;

use crate::{
    Angle,
    pattern::builder::{_Empty, Builder, HapticPatternBuilder},
};
use alloc::vec::Vec;
use core::{iter, time::Duration};
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
    /// Width of the activation zone
    deactivation_zone: Angle,
}

impl PatternLayer {
    /// Start building a new pattern layer definition
    pub fn builder() -> Builder<_Empty> {
        Builder::default()
    }
}
