use enterpolation::{
    Curve, Equidistant, Identity, Signal, TransformInput, bezier::Bezier, linear::Linear,
};

use crate::{Angle, Value};

pub(crate) trait CurveComponentInstance: core::fmt::Debug {
    fn sample(&self, at: Angle) -> Value;
    fn domain(&self) -> [Angle; 2];

    fn start(&self) -> Value {
        let t = self.domain();
        self.sample(t[0])
    }

    fn end(&self) -> Value {
        let t = self.domain();
        self.sample(t[1])
    }

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
