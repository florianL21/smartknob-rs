#![no_std]

mod curve;
mod patterns;
mod player;

pub use patterns::{Command, HapticPattern};
pub use player::{HapticPlayer, Playback};

pub use curve::{CurveBuilder, CurveError, HapticCurve};

pub type Angle = f32;
pub type Value = f32;
