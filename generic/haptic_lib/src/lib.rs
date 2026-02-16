#![no_std]

mod curve;
mod pattern;
mod player;

pub use pattern::{Command, HapticPattern, PatternLayer};
pub use player::{HapticPlayer, Playback};

pub use curve::{
    CurveError, CurveInstance,
    builder::{CurveBuilder, CurveSegment, HapticCurve},
};

pub type Angle = f32;
pub type Value = f32;
