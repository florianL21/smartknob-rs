#![no_std]

mod builder;
mod curve;
mod patterns;
mod player;

pub use patterns::{Command, HapticPattern, PatternLayer};
pub use player::{HapticPlayer, Playback};

pub use builder::{CurveBuilder, CurveSegment, HapticCurve};
pub use curve::{CurveError, CurveInstance};

pub type Angle = f32;
pub type Value = f32;
