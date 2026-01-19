extern crate alloc;

mod components;
use crate::curve::components::{
    BezierComponent, ConstComponent, CurveComponentInstance, LinearComponent,
};
use crate::{Angle, Value};
use core::slice::Iter;

use alloc::boxed::Box;
use enterpolation::linear::{Linear, LinearError};
use enterpolation::{
    TransformInput,
    bezier::{Bezier, BezierError},
};
use heapless::Vec;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum InterpolationBuilderError {
    #[error("Failed to build bezier interpolation: {0}")]
    BezierError(BezierError),
    #[error("Failed to build linear interpolation: {0}")]
    LinearError(LinearError),
    #[error("At least one of the values along this components is not within -1.0 to 1.0")]
    ValueOutOfRange,
}

#[derive(Error, Debug)]
pub enum CurveError {
    #[error("curve was empty")]
    EmptyCurve,
    #[error(
        "Not enough capacity for holding all curve components. Increase the curve capacity by {0} elements"
    )]
    NotEnoughCapacity(usize),
    #[error("Value must be in range of -1.0 to 1.0, but `value` was out of range at index {0:?}")]
    ValueOutOfRange(usize),
    #[error("Failed to instantiate curve: {0}")]
    InstantiationError(#[from] InterpolationBuilderError),
}

/// This is a single component of a haptic curve.
/// A component has a width (angle range) and defines how the output value behaves over that range.
/// These components are chained together in a list to form a haptic curve over a certain angle range.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum CurveComponent {
    /// A constant value over a certain angle range
    Const {
        /// The width of this component in the curve
        width: f32,
        /// The constant torque value to apply over this range
        value: f32,
    },
    /// Define a bezier curve with 3 points
    Bezier3 {
        /// The width of this component in the curve
        width: f32,
        /// Points for defining the bezier curve
        points: [f32; 3],
    },
    /// Define a bezier curve with 4 points
    Bezier4 {
        /// The width of this component in the curve
        width: f32,
        /// Points for defining the bezier curve
        points: [f32; 4],
    },
    /// Define a bezier curve with 5 points
    Bezier5 {
        /// The width of this component in the curve
        width: f32,
        /// Points for defining the bezier curve
        points: [f32; 5],
    },
    /// Define a bezier curve with 6 points
    Bezier6 {
        /// The width of this component in the curve
        width: f32,
        /// Points for defining the bezier curve
        points: [f32; 6],
    },
    /// A transition from a start to an end value linearly over its width
    Linear {
        /// The width of this component in the curve
        width: f32,
        /// Value at the start of the component
        start: f32,
        /// Value at the end o the component
        end: f32,
    },
}

fn make_bezier<const N: usize>(
    width: f32,
    points: [f32; N],
) -> Result<
    TransformInput<Bezier<f32, [f32; N], enterpolation::ConstSpace<f32, N>>, f32, f32>,
    InterpolationBuilderError,
> {
    Bezier::builder()
        .elements(points)
        .domain(0.0, width)
        .constant()
        .build()
        .map_err(InterpolationBuilderError::BezierError)
}

impl CurveComponent {
    fn build(self) -> Result<Box<dyn CurveComponentInstance>, InterpolationBuilderError> {
        let curve: Box<dyn CurveComponentInstance> = match self {
            CurveComponent::Bezier3 { width, points } => Box::new(BezierComponent {
                curve: make_bezier(width, points)?,
            }),
            CurveComponent::Bezier4 { width, points } => Box::new(BezierComponent {
                curve: make_bezier(width, points)?,
            }),
            CurveComponent::Bezier5 { width, points } => Box::new(BezierComponent {
                curve: make_bezier(width, points)?,
            }),
            CurveComponent::Bezier6 { width, points } => Box::new(BezierComponent {
                curve: make_bezier(width, points)?,
            }),
            CurveComponent::Const { value, width } => Box::new(ConstComponent {
                value,
                start_angle: 0.0,
                width,
            }),
            CurveComponent::Linear { width, start, end } => Box::new(LinearComponent {
                curve: Linear::builder()
                    .elements([start, end])
                    .equidistant()
                    .domain(0.0, width)
                    .build()
                    .map_err(InterpolationBuilderError::LinearError)?,
            }),
        };
        let sample_step = self.width() / 20.0;
        for i in 0..20 {
            let sample = curve.sample(i as f32 * sample_step);
            if sample < -1.0 || sample > 1.0 {
                return Err(InterpolationBuilderError::ValueOutOfRange);
            }
        }
        Ok(curve)
    }

    /// Get the width of this curve component
    pub(crate) fn width(&self) -> &f32 {
        match self {
            CurveComponent::Const { width, .. } => width,
            CurveComponent::Bezier3 { width, .. } => width,
            CurveComponent::Bezier4 { width, .. } => width,
            CurveComponent::Bezier5 { width, .. } => width,
            CurveComponent::Bezier6 { width, .. } => width,
            CurveComponent::Linear { width, .. } => width,
        }
    }
}

/// Curves always start at negative infinity. They always go left to right (increasing angle values).
/// The angle value after the last curve component is considered to be positive infinity.
/// Because of this the last curve component must be one that can handle infinite values, like for example the [`CurveComponent::Const`] component
#[derive(Debug, Clone)]
pub struct HapticCurve<const N: usize> {
    /// Individual components of this curve
    pub(crate) components: Vec<CurveComponent, N>,
    pub(crate) start_angle: Angle,
}

impl<const N: usize> HapticCurve<N> {
    /// Make a curve description into a playable instance.
    pub fn instantiate(self) -> Result<CurveInstance<N>, CurveError> {
        let total_width = self.components.iter().map(|c| c.width()).sum();
        let mut components = Vec::new();
        for (i, comp) in self.components.into_iter().enumerate() {
            components
                .push(comp.build().map_err(|e| {
                    if matches!(e, InterpolationBuilderError::ValueOutOfRange) {
                        CurveError::ValueOutOfRange(i)
                    } else {
                        e.into()
                    }
                })?)
                .map_err(|_| CurveError::NotEnoughCapacity(i))?;
        }
        let start_value = components.first().ok_or(CurveError::EmptyCurve)?.start();
        let end_value = components.last().ok_or(CurveError::EmptyCurve)?.end();
        Ok(CurveInstance {
            start_angle: self.start_angle,
            total_width,
            components,
            start_value,
            end_value,
        })
    }

    pub fn start_angle(&self) -> Angle {
        self.start_angle
    }
}

#[derive(Debug)]
pub struct CurveInstance<const N: usize> {
    /// List of components in this curve
    pub(crate) components: Vec<Box<dyn CurveComponentInstance>, N>,
    /// Offset from zero which defines where the curve starts at
    pub(crate) start_angle: Angle,
    /// Total width of all elements in the curve
    pub(crate) total_width: Angle,
    /// Torque value at the very start of the curve
    pub(crate) start_value: Value,
    /// Torque value at the very end of the curve
    pub(crate) end_value: Value,
}

impl<const N: usize> CurveInstance<N> {
    pub(crate) fn as_iter(&self) -> CurveIter<'_> {
        CurveIter::new(&self.components, self.start_angle)
    }
}

#[derive(Debug)]
pub(crate) struct ComponentView<'a> {
    pub(crate) component: &'a Box<dyn CurveComponentInstance>,
    pub(crate) start_angle: f32,
    pub(crate) end_angle: f32,
}

pub(crate) struct CurveIter<'a> {
    iter: Iter<'a, Box<dyn CurveComponentInstance>>,
    curr_angle: Angle,
}

impl<'a> CurveIter<'a> {
    fn new(curve: &'a [Box<dyn CurveComponentInstance>], start_angle: Angle) -> Self {
        CurveIter {
            iter: curve.iter(),
            curr_angle: start_angle,
        }
    }
}

impl<'a> Iterator for CurveIter<'a> {
    type Item = ComponentView<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let current = self.iter.next()?;
        let start_angle = self.curr_angle;
        let end_angle = start_angle + current.width();
        self.curr_angle = end_angle;
        Some(ComponentView {
            component: current,
            start_angle,
            end_angle,
        })
    }
}

/// Builder for constructing haptic curves
pub struct CurveBuilder<const N: usize> {
    components: Vec<CurveComponent, N>,
    over_capacity: usize,
    range_error: Option<CurveError>,
}

impl<const N: usize> Default for CurveBuilder<N> {
    fn default() -> Self {
        CurveBuilder {
            components: Vec::new(),
            over_capacity: 0,
            range_error: None,
        }
    }
}

impl<const N: usize> CurveBuilder<N> {
    /// Start constructing a new haptic curve
    pub fn new() -> Self {
        Self::default()
    }

    fn add_component(mut self, component: CurveComponent) -> Self {
        if self.over_capacity > 0 {
            self.over_capacity += 1;
            return self;
        }
        if self.components.push(component).is_err() {
            self.over_capacity = 1;
        }
        self
    }

    /// Add a segment to the curve during which the output value remains constant.
    /// `value` must be in the range of -1.0 to 1.0
    pub fn add_const(self, width: f32, value: f32) -> Self {
        self.add_component(CurveComponent::Const {
            width: width,
            value: value,
        })
    }

    /// Add a segment to the curve during which the output value linearly transitions from `start_value` to `end_value`.
    /// `start_value` and `end_value` must be in the range of -1.0 to 1.0
    pub fn add_linear(self, width: f32, start_value: f32, end_value: f32) -> Self {
        self.add_component(CurveComponent::Linear {
            width: width,
            start: start_value,
            end: end_value,
        })
    }

    /// Add a segment to the curve which is defined via the given control points
    /// During playback it will interpolate smoothly
    /// The control points given via `points` must produce a curve which is always in the range of -1.0 to 1.0
    pub fn add_bezier3(self, width: f32, points: [f32; 3]) -> Self {
        self.add_component(CurveComponent::Bezier3 {
            width: width,
            points,
        })
    }

    /// Finalize the curve building process and return the constructed curve
    /// By specifying `start_angle` you may shift where the start point of the curve is located at as an absolute position.
    /// When a curve starts playing the initial angle will be zero
    pub fn build(self, start_angle: Angle) -> Result<HapticCurve<N>, CurveError> {
        if let Some(e) = self.range_error {
            return Err(e);
        }
        if self.components.is_empty() {
            return Err(CurveError::EmptyCurve);
        }
        if self.over_capacity > 0 {
            return Err(CurveError::NotEnoughCapacity(self.over_capacity));
        }
        Ok(HapticCurve {
            components: self.components,
            start_angle,
        })
    }
}
