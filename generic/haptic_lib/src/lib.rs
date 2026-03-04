#![no_std]

mod config;
mod curve;
mod pattern;
mod player;

pub use config::{HapticConfiguration, HapticCurveConfig, HapticInstances};
pub use pattern::{Command, HapticPattern, PatternLayer};
pub use player::{HapticPlayer, Playback};

pub use curve::{
    CurveError, CurveInstance,
    builder::{CurveBuilder, CurveSegment, HapticCurve},
};

pub type Angle = f32;
pub type Value = f32;
