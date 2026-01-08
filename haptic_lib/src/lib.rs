#![no_std]

mod curve;
mod easings;
mod patterns;
mod player;

use fixed::types::I16F16;

pub use easings::{Easing, EasingType};
pub use patterns::{Command, HapticPattern};
pub use player::{HapticPlayer, Playback};

pub use curve::{AbsoluteCurve, CurveBuilder, CurveError, HapticCurve};

pub type Angle = I16F16;
pub type Value = I16F16;
