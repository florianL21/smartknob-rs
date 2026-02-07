extern crate alloc;

pub mod components;

use crate::curve::components::CurveComponentInstance;
use crate::{Angle, Value};
use alloc::boxed::Box;
use alloc::vec::Vec;
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

#[derive(Debug)]
pub struct SegmentInstance {
    /// List of components in this segment
    pub(crate) components: Vec<Box<dyn CurveComponentInstance>>,
    /// Calculated width of this segment for a fast lookup
    pub(crate) width: Angle,
}

#[derive(Debug)]
pub struct SegmentReference<'a> {
    /// Reference to a the actual used segment
    pub(crate) segment: &'a SegmentInstance,
    /// Calculated start angle of this reference within the full curve
    pub(crate) start_angle: Angle,
    /// Calculated end angle of this reference within the full curve
    pub(crate) end_angle: Angle,
    /// Repeat this exact reference a given amount of times in the full curve
    pub(crate) repeat: u16,
    /// Scale all values returned by the referenced segment by this amount.
    /// The resulting scaled value is not allowed to exceed/undershoot 1.0/-1.0
    pub(crate) scale: Value,
}

impl<'a> SegmentReference<'a> {
    fn get(&'a self, index: usize) -> Option<(&'a dyn CurveComponentInstance, Value)> {
        let num_comps = self.segment.components.len();
        if index > num_comps * self.repeat as usize {
            return None;
        }
        self.segment
            .components
            .get(index % self.repeat as usize)
            .map(|i| (i.as_ref(), self.scale))
    }

    fn len(&self) -> usize {
        self.segment.components.len() * self.repeat as usize
    }
}

#[derive(Debug)]
pub struct CurveInstance<'a> {
    /// List of components in this curve
    pub(crate) segments: Vec<SegmentInstance>,
    /// Actual curve definition referring to segments
    pub(crate) curve: Vec<SegmentReference<'a>>,
    /// Offset from zero which defines where the curve starts at
    pub(crate) start_angle: Angle,
    /// Total width of all elements in the curve
    pub(crate) total_width: Angle,
    /// Torque value at the very start of the curve
    pub(crate) start_value: Value,
    /// Torque value at the very end of the curve
    pub(crate) end_value: Value,
}

impl<'a> CurveInstance<'a> {
    /// Get a curve component by absolute curve index
    fn get(&self, index: usize) -> Option<(&dyn CurveComponentInstance, f32)> {
        let mut ind = index;
        for seg in self.curve.iter() {
            let seg_len = seg.len();
            if ind > seg_len {
                ind -= seg_len;
            } else {
                return seg.get(ind);
            }
        }
        None
    }

    /// Returns the number of indexes in this curve
    fn len(&self) -> usize {
        self.curve.iter().map(|i| i.len()).sum()
    }

    fn iter(&'a self) -> CurveIterator<'a> {
        CurveIterator::new(self, self.start_angle)
    }
}

struct CurveIterator<'a> {
    index: usize,
    curve_inst: &'a CurveInstance<'a>,
    // current: Option<CurveIterView<'a>>,
    prev_stop_angle: Angle,
    current: IteratorPosition<'a>,
    underflow: bool,
}

#[derive(Clone)]
enum IteratorPosition<'a> {
    Below,
    Above,
    Within(CurveIterView<'a>),
}

#[derive(Clone)]
struct CurveIterView<'a> {
    comp: &'a dyn CurveComponentInstance,
    start_angle: Angle,
    stop_angle: Angle,
    scale: Value,
}

impl<'a> CurveIterView<'a> {
    fn gauge_position(&self, angle: Angle) -> PositionGauge<'a> {
        if angle < self.start_angle {
            return PositionGauge::Lower;
        }
        if angle > self.stop_angle {
            PositionGauge::Higher
        } else {
            PositionGauge::SpotOn(self.comp)
        }
    }
}

enum PositionGauge<'a> {
    Higher,
    Lower,
    SpotOn(&'a dyn CurveComponentInstance),
}

impl<'a> CurveIterator<'a> {
    fn current(&self) -> IteratorPosition<'a> {
        self.current.clone()
    }

    fn new(curve: &'a CurveInstance, start_angle: Angle) -> Self {
        //TODO: Think how to handle this unwrap gracefully
        let current = curve
            .curve
            .first()
            .and_then(|sr| {
                sr.segment
                    .components
                    .first()
                    .map(|c| (c.as_ref(), sr.scale))
            })
            .map(|(comp, scale)| CurveIterView {
                comp,
                start_angle: start_angle,
                stop_angle: start_angle + comp.width(),
                scale,
            })
            .unwrap();
        CurveIterator {
            index: 0,
            curve_inst: curve,
            prev_stop_angle: start_angle,
            current: IteratorPosition::Within(current),
            underflow: false,
        }
    }
}

impl<'a> Iterator for CurveIterator<'a> {
    type Item = CurveIterView<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        self.underflow = false;
        if let Some((comp, scale)) = self.curve_inst.get(self.index) {
            let view = CurveIterView {
                comp,
                start_angle: self.prev_stop_angle,
                stop_angle: self.prev_stop_angle + comp.width(),
                scale,
            };
            self.prev_stop_angle = view.stop_angle;
            self.current = IteratorPosition::Within(view.clone());
            self.index += 1;
            return Some(view);
        } else {
            self.current = IteratorPosition::Above;
            return None;
        }
    }
}

impl<'a> DoubleEndedIterator for CurveIterator<'a> {
    fn next_back(&mut self) -> Option<Self::Item> {
        // No need to process anything if index is "below zero"
        if self.underflow {
            self.current = IteratorPosition::Below;
            return None;
        } else if let Some((comp, scale)) = self.curve_inst.get(self.index) {
            let view = CurveIterView {
                comp,
                start_angle: self.prev_stop_angle,
                stop_angle: self.prev_stop_angle + comp.width(),
                scale,
            };
            self.prev_stop_angle = view.start_angle;
            self.current = IteratorPosition::Within(view.clone());
            if self.index == 0 {
                // TODO: mark next iterator return as None
                self.underflow = true;
            }
            self.index -= 1;
            return Some(view);
        } else {
            return None;
        }
    }
}

pub struct CurveState<'a>(CurveIterator<'a>, &'a CurveInstance<'a>);

impl<'a> CurveState<'a> {
    pub fn new(curve: &'a CurveInstance<'a>) -> Self {
        Self(curve.iter(), curve)
    }

    fn search_forward(&mut self, angle: Angle) -> Value {
        while let Some(comp) = self.0.next() {
            if let PositionGauge::SpotOn(c) = comp.gauge_position(angle) {
                return c.sample(angle - comp.start_angle) * comp.scale;
            }
        }
        return self.1.end_value;
    }

    fn search_backward(&mut self, angle: Angle) -> Value {
        while let Some(comp) = self.0.next_back() {
            if let PositionGauge::SpotOn(c) = comp.gauge_position(angle) {
                return c.sample(angle - comp.start_angle) * comp.scale;
            }
        }
        return self.1.start_value;
    }

    pub fn sample(&mut self, angle: Angle) -> Value {
        if angle < self.1.start_angle {
            return self.1.start_value;
        } else if angle > self.1.start_angle + self.1.total_width {
            return self.1.end_value;
        }
        match self.0.current() {
            IteratorPosition::Within(curr) => match curr.gauge_position(angle) {
                PositionGauge::Higher => self.search_forward(angle),
                PositionGauge::Lower => self.search_backward(angle),
                PositionGauge::SpotOn(c) => return c.sample(angle - curr.start_angle) * curr.scale,
            },
            IteratorPosition::Above => self.search_backward(angle),
            IteratorPosition::Below => self.search_forward(angle),
        }
    }
}
