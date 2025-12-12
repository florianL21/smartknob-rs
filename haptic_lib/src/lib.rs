#![no_std]

use core::slice::Iter;

use fixed::types::I16F16;
use heapless::Vec;
use serde::{Deserialize, Serialize};

use thiserror::Error;

#[derive(Error, Debug)]
pub enum CurveError {
    #[error("curve was empty")]
    EmptyCurve,
    #[error("Not enough capacity for holding all curve components")]
    NotEnoughCapacity,
}

type Angle = I16F16;
type Value = I16F16;

/// This is a single component of a haptic curve.
/// A component always has a starting point defined by `from`.
/// One component ends where the `from` of the next one begins.
/// These components are chained together in a list to form a haptic curve over a certain angle range.
/// There are special components for repeating sections of a curve.
#[derive(Serialize, Deserialize, Debug)]
enum CurveComponent {
    Const {
        width: Angle,
        value: Value,
    },
    /// Linear transition of a curve with a given start and end point
    Linear {
        width: Angle,
        start: Value,
        end: Value,
    },
    /// Start a loop from the next curve component and repeat
    /// it the specified amount of times
    Loop {
        times: u16,
    },
    /// Marker for signaling the end of a loop
    EndLoop,
}

impl CurveComponent {
    fn width(&self) -> &Angle {
        match self {
            CurveComponent::Const { width, .. } => width,
            CurveComponent::Linear { width, .. } => width,
            _ => &Angle::ZERO,
        }
    }

    fn value(&self, angle: Angle) -> Option<Value> {
        match self {
            CurveComponent::Const { value, .. } => Some(value.clone()),
            CurveComponent::Linear { start, end, width } => {
                Some(((end - start) / width) * angle + start)
            }
            _ => None,
        }
    }

    fn min(&self) -> Option<&Value> {
        match self {
            CurveComponent::Const { value, .. } => Some(value),
            CurveComponent::Linear { start, end, .. } => {
                Some(if start < end { start } else { end })
            }
            _ => None,
        }
    }

    fn max(&self) -> Option<&Value> {
        match self {
            CurveComponent::Const { value, .. } => Some(value),
            CurveComponent::Linear { start, end, .. } => {
                Some(if start > end { start } else { end })
            }
            _ => None,
        }
    }
}

/// Curves always start at negative infinity. They always go left to right (increasing angle values).
/// The angle value after the last curve component is considered to be positive infinity.
/// Because of this the last curve component must be one that can handle infinite values, like for example the [`CurveComponent::Const`] component
#[derive(Debug)]
pub struct HapticCurve<const N: usize> {
    /// Individual components of this curve
    components: Vec<CurveComponent, N>,
    /// absolute minimal value of all components of the curve. Needed for scaling a curve after the fact
    min: Value,
    /// absolute maximum value of all components of the curve. Needed for scaling a curve after the fact
    max: Value,
}

impl<const N: usize> HapticCurve<N> {
    fn as_iter(&self) -> CurveIter<'_> {
        CurveIter::new(&self.components)
    }

    pub fn make_absolute(
        self,
        start_value: Value,
        end_value: Value,
        start_angle: Angle,
    ) -> AbsoluteCurve<N> {
        AbsoluteCurve {
            curve: self,
            end_value,
            start_angle,
            start_value,
        }
    }
}

pub struct CurveBuilder<const N: usize> {
    components: Vec<CurveComponent, N>,
    capacity_left: bool,
}

impl<const N: usize> CurveBuilder<N> {
    pub fn new() -> Self {
        CurveBuilder {
            components: Vec::new(),
            capacity_left: true,
        }
    }

    pub fn add_const(mut self, width: f32, value: f32) -> Self {
        if self.capacity_left {
            if let Err(_) = self.components.push(CurveComponent::Const {
                width: I16F16::from_num(width),
                value: I16F16::from_num(value),
            }) {
                self.capacity_left = false;
            }
        }

        self
    }

    pub fn add_linear(mut self, width: f32, start_value: f32, end_value: f32) -> Self {
        if let Err(_) = self.components.push(CurveComponent::Linear {
            width: I16F16::from_num(width),
            start: I16F16::from_num(start_value),
            end: I16F16::from_num(end_value),
        }) {
            self.capacity_left = false;
        }
        self
    }

    pub fn build(self) -> Result<HapticCurve<N>, CurveError> {
        if !self.capacity_left {
            return Err(CurveError::NotEnoughCapacity);
        }
        let min = self
            .components
            .iter()
            .filter_map(CurveComponent::min)
            .min()
            .ok_or(CurveError::EmptyCurve)?
            .clone();
        let max = self
            .components
            .iter()
            .filter_map(CurveComponent::min)
            .max()
            .ok_or(CurveError::EmptyCurve)?
            .clone();
        Ok(HapticCurve {
            components: self.components,
            min,
            max,
        })
    }
}

#[derive(Debug)]
struct ComponentView<'a> {
    component: &'a CurveComponent,
    start_angle: Angle,
    end_angle: Angle,
}

struct CurveIter<'a> {
    components: &'a [CurveComponent],
    iter: Iter<'a, CurveComponent>,
    loop_start: Option<Iter<'a, CurveComponent>>,
    loop_counter: Option<usize>,
    curr_angle: Angle,
}

impl<'a> CurveIter<'a> {
    fn new(curve: &'a [CurveComponent]) -> Self {
        CurveIter {
            components: curve,
            iter: curve.iter(),
            loop_start: None,
            loop_counter: None,
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
            start_angle: start_angle,
            end_angle,
        })
    }
}

/// This type of curve has a beginning and an end. It is anchored at a specific starting point. In other words this type of curve does not repeat infinitely
pub struct AbsoluteCurve<const N: usize> {
    curve: HapticCurve<N>,
    /// Initial value which the curve starts out with. This can be viewed as a [`CurveComponent::Const`] which starts at -infinity
    start_value: Value,
    /// Value of the curve after the last component. This can be viewed as a [`CurveComponent::Const`] which is appended to the end of the curve and has infinite width
    end_value: Value,
    /// Angle at which the curve will start playing. This is an absolute angle in this context but will be relative to a given start offset during actual playback
    start_angle: Angle,
}

pub struct HapticPlayer<'a, const N: usize> {
    curve: &'a AbsoluteCurve<N>,
    start_offset: Angle,
    curve_width: Angle,
}

impl<'a, const N: usize> HapticPlayer<'a, N> {
    pub fn new(start_offset: Angle, curve: &'a AbsoluteCurve<N>) -> Self {
        let curve_width = curve.curve.components.iter().map(|c| c.width()).sum();
        HapticPlayer {
            curve,
            start_offset,
            curve_width,
        }
    }

    pub fn curve_width(&self) -> Angle {
        self.curve_width
    }

    pub fn play(&self, position: Angle) -> Value {
        let angle = position - self.start_offset;
        if angle < self.curve.start_angle {
            return self.curve.start_value;
        } else if angle >= self.curve_width {
            return self.curve.end_value;
        }
        for component in self.curve.curve.as_iter() {
            if angle >= component.start_angle && angle < component.end_angle {
                // We found our current component
                if let Some(val) = component.component.value(angle - component.start_angle) {
                    return val;
                }
            }
        }
        Value::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    #[test]
    fn test_empty_curve() {
        let test_curve = CurveBuilder::<5>::new().build();
        assert!(matches!(test_curve, Err(CurveError::EmptyCurve)));
    }

    #[test]
    fn test_basic_curve() {
        let test_curve = CurveBuilder::<5>::new()
            .add_const(I16F16::from_num(2.0), I16F16::from_num(5.0))
            .unwrap()
            .build()
            .unwrap()
            .make_absolute(I16F16::ZERO, I16F16::ZERO, I16F16::ZERO);
        let player = HapticPlayer::new(I16F16::ZERO, &test_curve);
        assert_eq!(player.play(I16F16::from_num(-1)), I16F16::ZERO);
        assert_eq!(player.play(I16F16::from_num(1)), I16F16::from_num(5.0));
        assert_eq!(player.play(I16F16::from_num(3)), I16F16::ZERO);
    }
}
