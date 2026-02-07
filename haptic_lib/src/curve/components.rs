use enterpolation::{
    Curve, Equidistant, Identity, Signal, TransformInput, bezier::Bezier, linear::Linear,
};

use crate::{Angle, Value};

pub(crate) trait CurveComponentInstance: core::fmt::Debug {
    /// Sample the interpolation curve at the given angle. The angle should be between 0.0 and self.width()
    fn sample(&self, at: Angle) -> Value;

    /// Returns the start and end angles of the curve
    fn domain(&self) -> [Angle; 2];

    /// Returns the value of the component at its start
    fn start(&self) -> Value {
        let t = self.domain();
        self.sample(t[0])
    }

    /// Returns the value of the component at its end
    fn end(&self) -> Value {
        let t = self.domain();
        self.sample(t[1])
    }

    /// Returns the width of the component
    fn width(&self) -> Angle {
        let domain = self.domain();
        domain[1] - domain[0]
    }
}

#[derive(Debug)]
pub(crate) struct BezierComponent<const N: usize> {
    pub(crate) curve:
        TransformInput<Bezier<f32, [f32; N], enterpolation::ConstSpace<f32, N>>, f32, f32>,
}

impl<const N: usize> CurveComponentInstance for BezierComponent<N> {
    fn sample(&self, at: f32) -> f32 {
        self.curve.eval(at)
    }

    fn domain(&self) -> [Angle; 2] {
        self.curve.domain()
    }
}

#[derive(Debug)]
pub(crate) struct ConstComponent {
    pub(crate) value: Value,
    pub(crate) start_angle: Angle,
    pub(crate) width: Angle,
}

impl CurveComponentInstance for ConstComponent {
    fn sample(&self, _: f32) -> f32 {
        self.value
    }

    fn domain(&self) -> [Angle; 2] {
        [self.start_angle, self.start_angle + self.width]
    }
}

#[derive(Debug)]
pub(crate) struct LinearComponent {
    pub(crate) curve: Linear<Equidistant<f32>, [f32; 2], Identity>,
}

impl CurveComponentInstance for LinearComponent {
    fn sample(&self, at: f32) -> f32 {
        self.curve.eval(at)
    }

    fn domain(&self) -> [Angle; 2] {
        self.curve.domain()
    }
}
