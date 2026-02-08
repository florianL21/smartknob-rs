extern crate alloc;

use crate::curve::components::{
    BezierComponent, ConstComponent, CurveComponentInstance, LinearComponent,
};
use crate::{Angle, CurveError, CurveInstance, Value};

use alloc::boxed::Box;
use alloc::vec::Vec;
use enterpolation::linear::{Linear, LinearError};
use enterpolation::{
    TransformInput,
    bezier::{Bezier, BezierError},
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

type BezierInterpolation<const N: usize> =
    TransformInput<Bezier<f32, [f32; N], enterpolation::ConstSpace<f32, N>>, f32, f32>;

#[derive(Error, Debug)]
pub enum InterpolationBuilderError {
    #[error("Failed to build bezier interpolation: {0}")]
    BezierError(BezierError),
    #[error("Failed to build linear interpolation: {0}")]
    LinearError(LinearError),
    #[error("At least one of the values along this components is not within -1.0 to 1.0")]
    ValueOutOfRange,
}

// The following structs are used for creating a curve representation which can be serialized and
// constructed easily by using a builder. This representation of a curve is not yet playable but
// needs to be instantiated first.

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
) -> Result<BezierInterpolation<N>, InterpolationBuilderError> {
    Bezier::builder()
        .elements(points)
        .domain(0.0, width)
        .constant()
        .build()
        .map_err(InterpolationBuilderError::BezierError)
}

impl CurveComponent {
    pub(crate) fn build(
        self,
    ) -> Result<Box<dyn CurveComponentInstance>, InterpolationBuilderError> {
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
            if !(-1.0..=1.0).contains(&sample) {
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CurveSegment {
    pub(crate) components: Vec<CurveComponent>,
    pub(crate) width: Angle,
}

impl CurveSegment {
    fn new(components: Vec<CurveComponent>) -> Self {
        let width = components.iter().map(|c| c.width()).sum();
        Self { components, width }
    }

    fn width(&self) -> Angle {
        self.width
    }
}

/// Reference to a segment. Intentionally uses indexes and not actual refs as this need to be serializable
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct SegmentRef {
    /// Index of the segment in the segments array
    index: usize,
    /// Relative scale of the referenced segment for this instance
    scale: Value,
    /// Repeats the referenced segment the given amount of times in the final curve
    repeat: u16,
}

/// Curves always start at negative infinity. They always go left to right (increasing angle values).
/// The angle value after the last curve component is considered to be positive infinity.
/// Because of this the last curve components value at the very end of its width is taken as the value returned up until positive infinity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticCurve {
    /// Reusable segments of this curve
    pub(crate) segments: Vec<CurveSegment>,
    /// A list of indexes of these reusable segments
    pub(crate) curve: Vec<SegmentRef>,
    /// The start angle can be used to shift the curve left and right relative to the zero point
    pub(crate) start_angle: Angle,
}

impl HapticCurve {
    /// Make a curve description into a playable instance.
    // pub fn instantiate(self) -> Result<CurveInstance, CurveError> {
    //     let total_width = self.curve.iter().map(|c| c.width()).sum();
    //     let mut components = Vec::new();
    //     for (i, comp) in self
    //         .curve
    //         .into_iter()
    //         .flat_map(|i| i.components)
    //         .into_iter()
    //         .enumerate()
    //     {
    //         components.push(comp.build().map_err(|e| {
    //             if matches!(e, InterpolationBuilderError::ValueOutOfRange) {
    //                 CurveError::ValueOutOfRange(i)
    //             } else {
    //                 e.into()
    //             }
    //         })?);
    //     }
    //     let start_value = components.first().ok_or(CurveError::EmptyCurve)?.start();
    //     let end_value = components.last().ok_or(CurveError::EmptyCurve)?.end();
    //     Ok(CurveInstance {
    //         start_angle: self.start_angle,
    //         total_width,
    //         components,
    //         start_value,
    //         end_value,
    //     })
    // }

    pub fn start_angle(&self) -> Angle {
        self.start_angle
    }
}

/// Builder for constructing haptic curves
pub struct CurveBuilder {
    components: Vec<CurveComponent>,
    over_capacity: usize,
    range_error: Option<CurveError>,
}

impl Default for CurveBuilder {
    fn default() -> Self {
        CurveBuilder {
            components: Vec::new(),
            over_capacity: 0,
            range_error: None,
        }
    }
}

impl<'a> CurveBuilder {
    /// Start constructing a new haptic curve
    pub fn new() -> Self {
        Self::default()
    }

    fn add_component(mut self, component: CurveComponent) -> Self {
        if self.over_capacity > 0 {
            self.over_capacity += 1;
            return self;
        }
        self.components.push(component);
        self
    }

    /// Add a segment to the curve during which the output value remains constant.
    /// `value` must be in the range of -1.0 to 1.0
    pub fn add_const(self, width: f32, value: f32) -> Self {
        self.add_component(CurveComponent::Const { width, value })
    }

    /// Add a segment to the curve during which the output value linearly transitions from `start_value` to `end_value`.
    /// `start_value` and `end_value` must be in the range of -1.0 to 1.0
    pub fn add_linear(self, width: f32, start_value: f32, end_value: f32) -> Self {
        self.add_component(CurveComponent::Linear {
            width,
            start: start_value,
            end: end_value,
        })
    }

    /// Add a segment to the curve which is defined via the given control points
    /// During playback it will interpolate smoothly
    /// The control points given via `points` must produce a curve which is always in the range of -1.0 to 1.0
    pub fn add_bezier3(self, width: f32, points: [f32; 3]) -> Self {
        self.add_component(CurveComponent::Bezier3 { width, points })
    }

    // /// Finalize the curve building process and return the constructed curve
    // /// By specifying `start_angle` you may shift where the start point of the curve is located at as an absolute position.
    // /// When a curve starts playing the initial angle will be zero
    // pub fn build(self, start_angle: Angle) -> Result<HapticCurve, CurveError> {
    //     if let Some(e) = self.range_error {
    //         return Err(e);
    //     }
    //     if self.components.is_empty() {
    //         return Err(CurveError::EmptyCurve);
    //     }
    //     if self.over_capacity > 0 {
    //         return Err(CurveError::NotEnoughCapacity(self.over_capacity));
    //     }
    //     Ok(HapticCurve {
    //         components: self.components,
    //         start_angle,
    //     })
    // }
}
