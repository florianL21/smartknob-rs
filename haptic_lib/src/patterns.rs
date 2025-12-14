use core::{
    iter::{self, FlatMap, Repeat, Take},
    slice::Iter,
};

use fixed::types::I16F16;
use heapless::Vec;
use serde::{Deserialize, Serialize};

const MAX_PATTERN_COMMANDS: usize = 6;

type CommandVec = Vec<I16F16, MAX_PATTERN_COMMANDS>;

#[derive(Serialize, Deserialize, Debug)]
pub struct HapticPattern {
    commands: CommandVec,
    repeat: u16,
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

    // pub(crate) fn as_iter(
    //     &self,
    // ) -> Take<
    //     Repeat<
    //         FlatMap<Iter<'_, f32>, Take<Repeat<f32>>, impl FnMut(&f32) -> Take<Repeat<f32>> + '_>,
    //     >,
    // > {
    //     let num = (self.repeat * self.multiply) as usize;
    //     iter::repeat(
    //         self.commands
    //             .iter()
    //             .flat_map(|item| iter::repeat(item.clone()).take(self.multiply as usize)),
    //     )
    //     .take(num)
    // }
}
