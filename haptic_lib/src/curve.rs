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
    // /// Calculated width of this segment for a fast lookup
    // pub(crate) width: Angle,
}

#[derive(Debug)]
pub struct SegmentReference {
    /// Reference to an index in the segments vec of the curve instance
    pub(crate) segment_index: usize,
    // /// Calculated start angle of this reference within the full curve
    // pub(crate) start_angle: Angle,
    // /// Calculated end angle of this reference within the full curve
    // pub(crate) end_angle: Angle,
    /// Repeat this exact reference a given amount of times in the full curve
    pub(crate) repeat: u16,
    /// Scale all values returned by the referenced segment by this amount.
    /// The resulting scaled value is not allowed to exceed/undershoot 1.0/-1.0
    pub(crate) scale: Value,
}

impl SegmentReference {
    fn get<'a>(
        &'a self,
        segments: &'a Vec<SegmentInstance>,
        index: usize,
    ) -> Option<(&'a dyn CurveComponentInstance, Value)> {
        let segment = self.segment(segments);
        let num_comps = segment.components.len();
        if index >= num_comps * self.repeat as usize {
            return None;
        }
        segment
            .components
            .get(index % num_comps as usize)
            .map(|i| (i.as_ref(), self.scale))
    }

    fn segment<'a>(&'a self, segments: &'a Vec<SegmentInstance>) -> &'a SegmentInstance {
        // here we assume that the definition of the curve itself is correct. so this unwrap should never fail
        segments.get(self.segment_index).unwrap()
    }

    fn len(&self, segments: &Vec<SegmentInstance>) -> usize {
        self.segment(segments).components.len() * self.repeat as usize
    }
}

#[derive(Debug)]
pub struct CurveInstance {
    /// List of components in this curve
    pub(crate) segments: Vec<SegmentInstance>,
    /// Actual curve definition referring to segments
    pub(crate) curve: Vec<SegmentReference>,
    /// Offset from zero which defines where the curve starts at
    pub(crate) start_angle: Angle,
    /// Total width of all elements in the curve
    pub(crate) total_width: Angle,
    /// Torque value at the very start of the curve
    pub(crate) start_value: Value,
    /// Torque value at the very end of the curve
    pub(crate) end_value: Value,
}

impl CurveInstance {
    pub fn new(
        segments: Vec<SegmentInstance>,
        curve: Vec<SegmentReference>,
        start_angle: Angle,
    ) -> Self {
        let mut total_width: Angle = 0.0;
        for segment_ref in curve.iter() {
            for index in 0..segment_ref.len(&segments) {
                total_width += segment_ref
                    .get(&segments, index)
                    .map(|s| s.0.width())
                    .unwrap_or_default();
            }
        }
        let end_value = curve
            .last()
            .and_then(|sr| sr.get(&segments, sr.len(&segments).saturating_sub(1)))
            .map(|(ci, scale)| ci.end() * scale)
            .unwrap_or_default();
        let start_value = curve
            .first()
            .and_then(|sr| sr.get(&segments, 0))
            .map(|(ci, scale)| ci.start() * scale)
            .unwrap_or_default();
        Self {
            segments,
            curve,
            start_angle,
            total_width,
            end_value,
            start_value,
        }
    }

    /// Get a curve component by absolute curve index
    fn get(&self, index: usize) -> Option<(&dyn CurveComponentInstance, f32)> {
        let mut ind = index;
        for seg in self.curve.iter() {
            let seg_len = seg.len(&self.segments);
            if ind >= seg_len {
                ind -= seg_len;
            } else {
                return seg.get(&self.segments, ind);
            }
        }
        None
    }

    /// Returns the number of indexes in this curve
    fn len(&self) -> usize {
        self.curve.iter().map(|i| i.len(&self.segments)).sum()
    }

    fn iter<'a>(&'a self) -> CurveIterator<'a> {
        CurveIterator::new(self, self.start_angle)
    }
}

struct CurveIterator<'a> {
    index: usize,
    curve_inst: &'a CurveInstance,
    // current: Option<CurveIterView<'a>>,
    prev_stop_angle: Angle,
    current: IteratorPosition<'a>,
    underflow: bool,
}

#[derive(Debug, Clone)]
enum IteratorPosition<'a> {
    Below,
    Above,
    Within(CurveIterView<'a>),
}

#[derive(Debug, Clone)]
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
                sr.segment(&curve.segments)
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
                self.underflow = true;
            }
            self.index -= 1;
            return Some(view);
        } else {
            return None;
        }
    }
}

pub struct CurveState<'a>(CurveIterator<'a>, &'a CurveInstance);

impl<'a> CurveState<'a> {
    pub fn new(curve: &'a CurveInstance) -> Self {
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

#[cfg(test)]
mod tests {
    use crate::builder::CurveComponent;

    use super::*;
    extern crate std;
    use alloc::vec;
    use matches::assert_matches;

    fn basic_test_curve() -> CurveInstance {
        let segments = vec![SegmentInstance {
            components: vec![
                CurveComponent::Const {
                    width: 1.0,
                    value: 0.1,
                }
                .build()
                .unwrap(),
                CurveComponent::Const {
                    width: 1.0,
                    value: 0.2,
                }
                .build()
                .unwrap(),
            ],
        }];
        CurveInstance::new(
            segments,
            vec![
                SegmentReference {
                    segment_index: 0,
                    repeat: 1,
                    scale: 1.0,
                },
                SegmentReference {
                    segment_index: 0,
                    repeat: 2,
                    scale: 0.5,
                },
            ],
            0.0,
        )
    }

    #[test]
    fn test_segment_ref_get_function() {
        let curve = vec![SegmentInstance {
            components: vec![
                CurveComponent::Const {
                    width: 1.0,
                    value: 0.1,
                }
                .build()
                .unwrap(),
                CurveComponent::Const {
                    width: 1.0,
                    value: 0.2,
                }
                .build()
                .unwrap(),
                CurveComponent::Const {
                    width: 1.0,
                    value: 0.3,
                }
                .build()
                .unwrap(),
            ],
        }];
        let seg_ref = SegmentReference {
            repeat: 2,
            segment_index: 0,
            scale: 1.0,
        };
        assert_eq!(seg_ref.get(&curve, 0).unwrap().0.start(), 0.1);
        assert_eq!(seg_ref.get(&curve, 1).unwrap().0.start(), 0.2);
        assert_eq!(seg_ref.get(&curve, 2).unwrap().0.start(), 0.3);
        assert_eq!(seg_ref.get(&curve, 3).unwrap().0.start(), 0.1);
        assert_eq!(seg_ref.get(&curve, 4).unwrap().0.start(), 0.2);
        assert_eq!(seg_ref.get(&curve, 5).unwrap().0.start(), 0.3);
        assert!(seg_ref.get(&curve, 6).is_none());
    }

    #[test]
    fn test_curve_iter_initial_current_item_angles_are_that_of_first_component() {
        let curve = basic_test_curve();
        let curve_iter = curve.iter();
        assert!(matches!(
            curve_iter.current(),
            IteratorPosition::Within(CurveIterView {
                start_angle: 0.0,
                stop_angle: 1.0,
                ..
            })
        ));
        assert_eq!(curve_iter.prev_stop_angle, 0.0);
    }

    #[test]
    fn test_curve_iter_next2_component_angles_are_added_to_initial_ones() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        // consumes the first element
        let _ = curve_iter.next();
        assert_matches!(
            curve_iter.next(),
            Some(CurveIterView {
                start_angle: 1.0,
                stop_angle: 2.0,
                ..
            })
        );
    }

    #[test]
    fn test_curve_sampling_tests() {
        let curve = basic_test_curve();
        let mut state = CurveState::new(&curve);
        // sample well below the curve; should respect the 1.0 scale
        assert_eq!(state.sample(-2.0), 0.1);
        // 1st segment
        assert_eq!(state.sample(0.01), 0.1);
        assert_eq!(state.sample(0.99), 0.1);
        assert_eq!(state.sample(1.01), 0.2);
        assert_eq!(state.sample(1.99), 0.2);
        // 2nd segment iter 1; has 0.5 scale
        assert_eq!(state.sample(2.01), 0.05);
        assert_eq!(state.sample(2.99), 0.05);
        assert_eq!(state.sample(3.01), 0.1);
        assert_eq!(state.sample(3.99), 0.1);
        // 2nd segment iter 2; has 0.5 scale
        assert_eq!(state.sample(4.01), 0.05);
        assert_eq!(state.sample(4.99), 0.05);
        assert_eq!(state.sample(5.01), 0.1);
        assert_eq!(state.sample(5.99), 0.1);

        // sample well above the curve; should respect the 0.5 scale
        assert_eq!(state.sample(10.0), 0.1);
    }

    #[test]
    fn test_curve_instance_get_tests() {
        let curve = basic_test_curve();
        assert_eq!(curve.get(0).unwrap().0.start(), 0.1);
        assert_eq!(curve.get(1).unwrap().0.start(), 0.2);
        // 2nd ref 1st iteration
        assert_eq!(curve.get(2).unwrap().0.start(), 0.1);
        assert_eq!(curve.get(3).unwrap().0.start(), 0.2);
        // 2nd ref 2nd iteration
        assert_eq!(curve.get(4).unwrap().0.start(), 0.1);
        assert_eq!(curve.get(5).unwrap().0.start(), 0.2);
        assert!(curve.get(6).is_none());
    }

    #[test]
    fn test_curve_iter_next2_yields_second_component() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        let _ = curve_iter.next();
        match curve_iter.next() {
            Some(cv) => assert_eq!(cv.comp.sample(0.0), 0.2),
            _ => panic!("Initial result should return an iterator position within the curve"),
        }
    }
}
