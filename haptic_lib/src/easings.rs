use easing_fixed::{
    CubicIn, CubicInOut, CubicOut, ExpIn, ExpInOut, ExpOut, Fix, Linear, QuadIn, QuadInOut,
    QuadOut, QuarticIn, QuarticInOut, QuarticOut, SinIn, SinInOut, SinOut,
};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub enum EasingType {
    In,
    Out,
    InOut,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum Easing {
    Linear,
    Quadratic(EasingType),
    Cubic(EasingType),
    Quartic(EasingType),
    Sinusoidal(EasingType),
    Exponential(EasingType),
}

impl Easing {
    pub fn at_normalized(&self, x: Fix) -> Fix {
        match self {
            Easing::Linear => Linear::at_normalized(x),
            Easing::Quadratic(t) => match t {
                EasingType::In => QuadIn::at_normalized(x),
                EasingType::Out => QuadOut::at_normalized(x),
                EasingType::InOut => QuadInOut::at_normalized(x),
            },
            Easing::Cubic(t) => match t {
                EasingType::In => CubicIn::at_normalized(x),
                EasingType::Out => CubicOut::at_normalized(x),
                EasingType::InOut => CubicInOut::at_normalized(x),
            },
            Easing::Quartic(t) => match t {
                EasingType::In => QuarticIn::at_normalized(x),
                EasingType::Out => QuarticOut::at_normalized(x),
                EasingType::InOut => QuarticInOut::at_normalized(x),
            },
            Easing::Sinusoidal(t) => match t {
                EasingType::In => SinIn::at_normalized(x),
                EasingType::Out => SinOut::at_normalized(x),
                EasingType::InOut => SinInOut::at_normalized(x),
            },
            Easing::Exponential(t) => match t {
                EasingType::In => ExpIn::at_normalized(x),
                EasingType::Out => ExpOut::at_normalized(x),
                EasingType::InOut => ExpInOut::at_normalized(x),
            },
        }
    }
}
