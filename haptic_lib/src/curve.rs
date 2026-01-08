pub use crate::easings::Easing;
use crate::{Angle, HapticPattern, Value, patterns::Command};
use core::slice::Iter;

use fixed::types::I16F16;
use heapless::Vec;
use serde::{Deserialize, Serialize};

use thiserror::Error;

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
}

/// This is a single component of a haptic curve.
/// A component has a width (angle range) and defines how the output value behaves over that range.
/// These components are chained together in a list to form a haptic curve over a certain angle range.
#[derive(Serialize, Deserialize, Debug)]
pub enum CurveComponent {
    /// A constant value over a certain angle range
    Const {
        /// The width of this component in the curve
        width: Angle,
        /// The constant torque value to apply over this range
        value: Value,
        /// Optional haptic pattern to play when entering the angle range of this component
        pattern_on_entry: Option<HapticPattern>,
    },
    /// An transition from a start value to an end value over a certain angle range with a specified easing function
    Eased {
        /// The width of this component in the curve
        width: Angle,
        /// The torque value at the start of this component
        start: Value,
        /// The torque value at the end of this component
        end: Value,
        /// The easing function to use for this transition
        easing: Easing,
        /// Optional haptic pattern to play when entering the angle range of this component
        pattern_on_entry: Option<HapticPattern>,
    },
}

impl CurveComponent {
    /// Get the width of this curve component
    pub(crate) fn width(&self) -> &Angle {
        match self {
            CurveComponent::Const { width, .. } => width,
            CurveComponent::Eased { width, .. } => width,
        }
    }

    /// Get the curve value at a specific angle within this component.
    /// `angle` must be in the range of 0 to width of this component
    pub(crate) fn value(&self, angle: Angle) -> Value {
        match self {
            CurveComponent::Const { value, .. } => *value,
            CurveComponent::Eased {
                start,
                end,
                width,
                easing,
                ..
            } => start + (end - start) * easing.at_normalized(angle / width),
        }
    }

    /// Get the curve value at the start of this component
    pub(crate) fn start(&self) -> &Value {
        match self {
            CurveComponent::Const { value, .. } => value,
            CurveComponent::Eased { start, .. } => start,
        }
    }

    /// Get the curve value at the end of this component
    pub(crate) fn end(&self) -> &Value {
        match self {
            CurveComponent::Const { value, .. } => value,
            CurveComponent::Eased { end, .. } => end,
        }
    }

    /// Check if the values of this component are within valid range
    fn values_valid(&self) -> bool {
        match self {
            CurveComponent::Const { value, .. } => *value >= -Value::ONE && *value <= Value::ONE,
            CurveComponent::Eased { start, end, .. } => {
                *start >= -Value::ONE
                    && *start <= Value::ONE
                    && *end >= -Value::ONE
                    && *end <= Value::ONE
            }
        }
    }

    pub(crate) fn pattern(&self) -> &Option<HapticPattern> {
        match self {
            CurveComponent::Const {
                pattern_on_entry, ..
            } => pattern_on_entry,
            CurveComponent::Eased {
                pattern_on_entry, ..
            } => pattern_on_entry,
        }
    }
}

/// Curves always start at negative infinity. They always go left to right (increasing angle values).
/// The angle value after the last curve component is considered to be positive infinity.
/// Because of this the last curve component must be one that can handle infinite values, like for example the [`CurveComponent::Const`] component
#[derive(Debug)]
pub struct HapticCurve<const N: usize> {
    /// Individual components of this curve
    pub(crate) components: Vec<CurveComponent, N>,
}

impl<const N: usize> HapticCurve<N> {
    pub(crate) fn as_iter(&self) -> CurveIter<'_> {
        CurveIter::new(&self.components)
    }

    /// Convert this curve into an absolute curve starting at a given absolute angle
    pub fn make_absolute(self, start_angle: Angle) -> AbsoluteCurve<N> {
        let start_value = self.components.first().map_or(Angle::ZERO, |c| *c.start());
        let end_value = self.components.last().map_or(Angle::ZERO, |c| *c.end());
        AbsoluteCurve {
            curve: self,
            start_value,
            end_value,
            start_angle,
        }
    }
}

/// This type of curve has a beginning and an end. It is anchored at a specific starting point. In other words this type of curve does not repeat infinitely
pub struct AbsoluteCurve<const N: usize> {
    pub(crate) curve: HapticCurve<N>,
    /// Initial value which the curve starts out with. This can be viewed as a [`CurveComponent::Const`] which starts at -infinity
    pub(crate) start_value: Value,
    /// Value of the curve after the last component. This can be viewed as a [`CurveComponent::Const`] which is appended to the end of the curve and has infinite width
    pub(crate) end_value: Value,
    /// Angle at which the curve will start playing. This is an absolute angle in this context but will be relative to a given start offset during actual playback
    pub(crate) start_angle: Angle,
}

#[derive(Debug)]
pub(crate) struct ComponentView<'a> {
    pub(crate) component: &'a CurveComponent,
    pub(crate) start_angle: Angle,
    pub(crate) end_angle: Angle,
}

pub(crate) struct CurveIter<'a> {
    iter: Iter<'a, CurveComponent>,
    curr_angle: Angle,
}

impl<'a> CurveIter<'a> {
    fn new(curve: &'a [CurveComponent]) -> Self {
        CurveIter {
            iter: curve.iter(),
            curr_angle: Angle::ZERO,
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
        if !component.values_valid() {
            self.range_error = Some(CurveError::ValueOutOfRange(self.components.iter().count()));
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
            width: I16F16::from_num(width),
            value: I16F16::from_num(value),
            pattern_on_entry: None,
        })
    }

    /// Add a segment to the curve which plays a haptic pattern upon entry, the value otherwise remains constant.
    /// `value` must be in the range of -1.0 to 1.0
    pub fn add_pattern(
        self,
        width: f32,
        value: f32,
        pattern: &[Command],
        repeat: u16,
        multiply: u16,
    ) -> Self {
        self.add_component(CurveComponent::Const {
            width: I16F16::from_num(width),
            value: I16F16::from_num(value),
            pattern_on_entry: Some(HapticPattern::new(
                Vec::from_iter(pattern.iter().cloned()),
                repeat,
                multiply,
            )),
        })
    }

    /// Add a segment to the curve during which the output value linearly transitions from `start_value` to `end_value`.
    /// `start_value` and `end_value` must be in the range of -1.0 to 1.0
    pub fn add_linear(self, width: f32, start_value: f32, end_value: f32) -> Self {
        self.add_eased(width, start_value, end_value, Easing::Linear)
    }

    /// Add a segment to the curve during which the output value transitions from `start_value` to `end_value`
    /// smoothly defined by the given easing function.
    /// `start_value` and `end_value` must be in the range of -1.0 to 1.0
    pub fn add_eased(self, width: f32, start_value: f32, end_value: f32, easing: Easing) -> Self {
        self.add_component(CurveComponent::Eased {
            width: I16F16::from_num(width),
            start: I16F16::from_num(start_value),
            end: I16F16::from_num(end_value),
            easing,
            pattern_on_entry: None,
        })
    }

    /// Finalize the curve building process and return the constructed curve
    pub fn build(self) -> Result<HapticCurve<N>, CurveError> {
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
        })
    }
}
