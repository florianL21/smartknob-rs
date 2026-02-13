extern crate alloc;

pub mod components;

use crate::builder::InterpolationBuilderError;
use crate::curve::components::CurveComponentInstance;
use crate::{Angle, Value};
use alloc::boxed::Box;
use alloc::vec::Vec;
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
    #[error(
        "Value must be in range of -1.0 to 1.0, but `value` of segment {0} out of range at index {0:?}"
    )]
    ValueOutOfRange(usize, usize),
    #[error("Builder error")]
    BuilderError(#[from] InterpolationBuilderError),
    #[error("Reference to curve segment {0} is invalid as this index does not exist")]
    ReferenceIndexInvalid(usize),
}

#[derive(Debug)]
pub struct SegmentInstance {
    /// List of components in this segment
    pub(crate) components: Vec<Box<dyn CurveComponentInstance>>,
    // /// Calculated width of this segment for a fast lookup
    // pub(crate) width: Angle,
}

/// Reference to a segment. Intentionally uses indexes and not actual refs as this need to be serializable
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SegmentReference {
    /// Reference to an index in the segments vec of the curve instance
    pub(crate) reference: usize,
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
        segments.get(self.reference).unwrap()
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
    /// Max index which can be gotten from the curve
    pub(crate) max_index: usize,
}

impl CurveInstance {
    pub fn new(
        segments: Vec<SegmentInstance>,
        curve: Vec<SegmentReference>,
        start_angle: Angle,
    ) -> Result<Self, CurveError> {
        let mut total_width: Angle = 0.0;
        for segment_ref in curve.iter() {
            for index in 0..segment_ref.len(&segments) {
                total_width += segment_ref
                    .get(&segments, index)
                    .map(|s| s.0.width())
                    .unwrap_or_default();
            }
        }
        let start_index = curve.first().ok_or(CurveError::EmptyCurve)?.reference;
        let start_value = segments
            .get(start_index)
            .ok_or(CurveError::ReferenceIndexInvalid(start_index))?
            .components
            .first()
            .ok_or(CurveError::EmptyCurve)?
            .start();
        let end_index = curve.last().ok_or(CurveError::EmptyCurve)?.reference;
        let end_value = segments
            .get(end_index)
            .ok_or(CurveError::ReferenceIndexInvalid(end_index))?
            .components
            .first()
            .ok_or(CurveError::EmptyCurve)?
            .end();
        let mut max_index: usize = curve.iter().map(|i| i.len(&segments)).sum();
        max_index = max_index.saturating_sub(1);
        Ok(Self {
            segments,
            curve,
            start_angle,
            total_width,
            end_value,
            start_value,
            max_index,
        })
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

    pub fn width(&self) -> Angle {
        self.total_width
    }

    fn iter<'a>(&'a self) -> CurveIterator<'a> {
        CurveIterator::new(self, self.start_angle)
    }
}

struct CurveIterator<'a> {
    index: usize,
    curve_inst: &'a CurveInstance,
    initial_start_angle: Angle,
    start_angle_next: Angle,
    stop_angle_next_back: Angle,
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

#[derive(Debug)]
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
        CurveIterator {
            index: 0,
            curve_inst: curve,
            initial_start_angle: start_angle,
            start_angle_next: start_angle,
            stop_angle_next_back: start_angle,
            current: IteratorPosition::Below,
            underflow: false,
        }
    }
}

impl<'a> Iterator for CurveIterator<'a> {
    type Item = CurveIterView<'a>;
    fn next(&mut self) -> Option<Self::Item> {
        self.underflow = false;
        if self.index >= self.curve_inst.max_index {
            self.current = IteratorPosition::Above;
            return None;
        }
        // skip increasing the index if this iterator is in its initial position
        if !(matches!(self.current, IteratorPosition::Below) && self.index == 0) {
            self.index += 1;
        }
        if let Some((comp, scale)) = self.curve_inst.get(self.index) {
            let view = CurveIterView {
                comp,
                start_angle: self.start_angle_next,
                stop_angle: self.start_angle_next + comp.width(),
                scale,
            };
            if self.index >= self.curve_inst.max_index {
                // We are clipping the upper limit of the curve. So if we turn around the
                // iterator as the nex operation it's component must end at the curve limit
                self.stop_angle_next_back = view.stop_angle;
            } else {
                self.stop_angle_next_back = view.start_angle;
            }
            self.start_angle_next = view.stop_angle;
            self.current = IteratorPosition::Within(view.clone());
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
        if self.underflow || self.index == 0 {
            self.underflow = true;
            self.current = IteratorPosition::Below;
            return None;
        }
        // skip decreasing the index if this iterator is coming from an overflow
        if !(matches!(self.current, IteratorPosition::Above)
            && self.index == self.curve_inst.max_index)
        {
            self.index = self.index.saturating_sub(1);
        }
        if let Some((comp, scale)) = self.curve_inst.get(self.index) {
            let view = CurveIterView {
                comp,
                start_angle: self.stop_angle_next_back - comp.width(),
                stop_angle: self.stop_angle_next_back,
                scale,
            };
            if self.index == 0 {
                // We are clipping the lower limit of the curve. So if we turn around the
                // iterator as the next operation it's component must start at the curves initial angle
                self.start_angle_next = self.initial_start_angle;
            } else {
                self.start_angle_next = view.stop_angle;
            }
            self.stop_angle_next_back = view.start_angle;
            self.current = IteratorPosition::Within(view.clone());
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
                PositionGauge::SpotOn(c) => {
                    return c.sample(angle - curr.start_angle) * curr.scale;
                }
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
                    reference: 0,
                    repeat: 1,
                    scale: 1.0,
                },
                SegmentReference {
                    reference: 0,
                    repeat: 2,
                    scale: 0.5,
                },
            ],
            0.0,
        )
        .unwrap()
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
            reference: 0,
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

    #[test]
    fn test_curve_width() {
        let curve = basic_test_curve();
        assert_eq!(curve.width(), 6.0);
    }

    fn expect_view(
        view: Option<CurveIterView>,
        start_angle: Angle,
        stop_angle: Angle,
        scale: Value,
        comp_start_value: Value,
    ) {
        assert_matches!(
            view,
            Some(CurveIterView {
                start_angle: start_angle,
                stop_angle: stop_angle,
                scale: scale,
                ..
            })
        );
        if let Some(CurveIterView { comp, .. }) = view {
            assert_eq!(comp.start(), comp_start_value)
        }
    }

    #[test]
    fn test_curve_iter_direction_reversal() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        expect_view(curve_iter.next(), 0.0, 1.0, 1.0, 0.1);
        expect_view(curve_iter.next(), 1.0, 2.0, 1.0, 0.2);
        expect_view(curve_iter.next_back(), 0.0, 1.0, 1.0, 0.1);
        assert_matches!(curve_iter.next_back(), None);
        assert_matches!(curve_iter.next_back(), None);
        expect_view(curve_iter.next(), 0.0, 1.0, 1.0, 0.1);
    }

    #[test]
    fn test_curve_iter_reversal_after_overflow() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        expect_view(curve_iter.next(), 5.0, 6.0, 0.5, 0.2);
        // This will overflow the iterator
        assert_matches!(curve_iter.next(), None);
        expect_view(curve_iter.next_back(), 5.0, 6.0, 0.5, 0.2);
    }

    #[test]
    fn test_curve_iter_recover_form_overflow() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        let _ = curve_iter.next();
        // last one before going out of range
        assert!(curve_iter.next().is_some());
        assert_matches!(curve_iter.current(), IteratorPosition::Within(_));
        // The next call should take the iterator above its range
        assert!(curve_iter.next().is_none());
        assert_matches!(curve_iter.current(), IteratorPosition::Above);
        // Going up further should still return None, but should not overshoot any further
        assert!(curve_iter.next().is_none());
        assert_matches!(curve_iter.current(), IteratorPosition::Above);
        // Going back once should immediately bring the iterator back in range
        assert!(curve_iter.next_back().is_some());
        assert_matches!(curve_iter.current(), IteratorPosition::Within(_));
    }

    #[test]
    fn test_curve_iter_recover_form_underflow() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        // Going back on on the new iterator should immediately go out of range
        assert!(curve_iter.next_back().is_none());
        assert_matches!(curve_iter.current(), IteratorPosition::Below);
        // Doing it again should stay out of range
        assert!(curve_iter.next_back().is_none());
        assert_matches!(curve_iter.current(), IteratorPosition::Below);
        // Going forward once should come back into range, no matte how often next_back was called
        assert!(curve_iter.next().is_some());
        assert_matches!(curve_iter.current(), IteratorPosition::Within(_));
    }

    #[test]
    fn test_curve_iterator_gauge_position() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        if let Some(curr) = curve_iter.next() {
            assert_matches!(curr.gauge_position(0.0), PositionGauge::SpotOn(_));
            assert_matches!(curr.gauge_position(1.0), PositionGauge::SpotOn(_));
            assert_matches!(curr.gauge_position(1.01), PositionGauge::Higher);
            assert_matches!(curr.gauge_position(-0.01), PositionGauge::Lower);
        } else {
            panic!("Expected curve to contain at least one element");
        }
    }

    #[test]
    fn test_curve_iterator_test_relationship_between_current_and_iter_methods() {
        let curve = basic_test_curve();
        let mut curve_iter = curve.iter();
        assert_matches!(curve_iter.current(), IteratorPosition::Below);
        assert_matches!(
            curve_iter.next(),
            Some(CurveIterView {
                start_angle: 0.0,
                ..
            })
        );
        assert_matches!(
            curve_iter.current(),
            IteratorPosition::Within(CurveIterView {
                start_angle: 0.0,
                ..
            })
        );
        assert_matches!(
            curve_iter.next(),
            Some(CurveIterView {
                start_angle: 1.0,
                ..
            })
        );
        assert_matches!(
            curve_iter.current(),
            IteratorPosition::Within(CurveIterView {
                start_angle: 1.0,
                ..
            })
        );
        // Start over and test into the negative direction
        let mut curve_iter = curve.iter();
        assert!(curve_iter.next_back().is_none());
        assert_matches!(curve_iter.current(), IteratorPosition::Below);
        assert!(curve_iter.next().is_some());
        assert_matches!(
            curve_iter.current(),
            IteratorPosition::Within(CurveIterView {
                start_angle: 0.0,
                ..
            })
        );
    }

    #[test]
    fn test_curve_state_search_forward() {
        let curve = basic_test_curve();
        let mut state = CurveState::new(&curve);
        // sample forward, almost until the very end
        assert_eq!(state.search_forward(5.0), 0.05);
        assert_matches!(state.0.current(), IteratorPosition::Within(_));
        // sample forward a bit more, this should advance the iterator by one
        assert_eq!(state.search_forward(6.0), 0.1);
        assert_matches!(state.0.current(), IteratorPosition::Within(_));
        // Sample above the iterator
        assert_eq!(state.search_forward(10.0), 0.1);
        assert_matches!(state.0.current(), IteratorPosition::Above);
        // Now we need to search backwards to get back into the curve
        assert_eq!(state.search_backward(6.0), 0.1);
        assert_matches!(state.0.current(), IteratorPosition::Within(_));
        // Doing the same thing again will exhaust the iterator all the way to the beginning
        assert_eq!(state.search_backward(6.0), 0.1);
        assert_matches!(state.0.current(), IteratorPosition::Below);
    }
}
