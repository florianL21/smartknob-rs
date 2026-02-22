extern crate alloc;

use crate::{
    Angle, Command, HapticPattern, PatternLayer,
    pattern::{CommandVec, SequenceComponent},
};
use alloc::vec::Vec;
use core::{marker::PhantomData, time::Duration};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum PatternBuildError {
    #[error(
        "The activation zones of the pattern at position {0} overlaps with the previous pattern"
    )]
    ActivationZoneOverlap(usize),
    #[error(
        "Go unexpected ratio for deactivation zone size. Expected a value greater than 0.0 but smaller than 1.0, got: {0}"
    )]
    DeactivationRatioInvalid(Angle),
}

/// Marker for a builder which does not have any components yet
#[derive(Default)]
pub struct _Empty {}
/// Marker for a builder which has at least one component
pub struct _NonEmpty {}

/// A builder for creating a haptic pattern lr definition
#[derive(Default)]
pub struct Builder<T> {
    /// Current list of components already built
    components: Vec<SequenceComponent>,
    phantom: PhantomData<T>,
}

impl<T> Builder<T> {
    fn to_non_empty(self) -> Builder<_NonEmpty> {
        Builder {
            components: self.components,
            phantom: PhantomData::default(),
        }
    }
    /// Add some space before the given `pattern`.
    /// `width` must be larger than the `activation_zone`/2.
    /// `activation_zone` is given as an argument to the build function.
    /// This requirement is checked at the time of building the layer
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticPattern, PatternLayer};
    /// let layer = PatternLayer::builder()
    ///        .with_space(1.0, HapticPattern::builder().torque(1.0).finish())
    ///        .insert_once()
    ///        .build(0.2, 0.8).unwrap();
    /// ```
    pub fn with_space(self, width: Angle, pattern: HapticPattern) -> SequenceBuilder<_NonEmpty> {
        SequenceBuilder {
            root_builder: self.to_non_empty(),
            sequence: SequenceComponent {
                width: width,
                pattern,
                repeat: 1,
            },
        }
    }

    /// Finally build the new layer.
    /// - `activation_zone` is the width of the zone which upon entering will activate a pattern
    /// - `deactivation_zone` is the percentage of the activation zone and must be below 1.0
    /// Note that the `activation_zone` is an absolute value in radiants while the `deactivation_zone` is a percentage of the `activation_zone`.
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticPattern, PatternLayer};
    /// let layer = PatternLayer::builder()
    ///        .with_space(1.0, HapticPattern::builder().torque(1.0).finish())
    ///        .insert_once()
    ///        .build(0.2, 0.8).unwrap();
    /// ```
    pub fn build(
        self,
        activation_zone: Angle,
        deactivation_zone: Angle,
    ) -> Result<PatternLayer, PatternBuildError> {
        if deactivation_zone <= 0.0 || deactivation_zone >= 1.0 {
            return Err(PatternBuildError::DeactivationRatioInvalid(
                deactivation_zone,
            ));
        }
        for (i, comp) in self.components.iter().enumerate() {
            if i > 0 && comp.width < activation_zone {
                return Err(PatternBuildError::ActivationZoneOverlap(i));
            }
        }
        // TODO: Check all torque values that for being between -1.0 and 1.0
        Ok(PatternLayer {
            components: self.components,
            activation_zone,
            deactivation_zone: activation_zone * deactivation_zone,
        })
    }
}

impl Builder<_Empty> {
    /// Add a new pattern to the zero position of the layer.
    /// Note that this is only possible to call on an empty layer.
    /// Once at least one element is added by whichever means only
    /// the [`Builder::with_space`] method will be callable.
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticPattern, PatternLayer};
    /// let layer = PatternLayer::builder()
    ///        .at_zero(HapticPattern::builder().torque(1.0).finish())
    ///        .insert_once()
    ///        .build(0.2, 0.8);
    /// ```
    pub fn at_zero(self, pattern: HapticPattern) -> SequenceBuilder<_NonEmpty> {
        SequenceBuilder {
            root_builder: self.to_non_empty(),
            sequence: SequenceComponent {
                width: 0.0,
                pattern,
                repeat: 1,
            },
        }
    }
}

/// Builder state in the middle of building a new haptic pattern with context
/// of how wide the space before the next pattern should be
#[derive(Default)]
pub struct HapticPatternBuilder<M> {
    width: Angle,
    commands: CommandVec,
    phantom: PhantomData<M>,
}

impl<M> HapticPatternBuilder<M> {
    /// Add a new torque command to this sequence
    /// `torque` must be in the range of -1.0 to 1.0
    /// # Example usage:
    /// ```
    /// use haptic_lib::HapticPattern;
    /// let pattern = HapticPattern::builder().torque(1.0).finish();
    /// ```
    pub fn torque(mut self, torque: f32) -> HapticPatternBuilder<_NonEmpty> {
        self.commands.push(Command::torque(torque));
        HapticPatternBuilder {
            commands: self.commands,
            width: self.width,
            phantom: PhantomData::default(),
        }
    }
}

impl HapticPatternBuilder<_NonEmpty> {
    /// Add a new delay command to this sequence
    /// The given duration will delay the execution of the next torque command by the given `duration`
    /// # Example usage:
    /// ```
    /// use haptic_lib::HapticPattern;
    /// use core::time::Duration;
    /// let pattern = HapticPattern::builder().torque(1.0).delay(Duration::from_millis(10)).finish();
    /// ```
    pub fn delay(mut self, duration: Duration) -> HapticPatternBuilder<_NonEmpty> {
        self.commands.push(Command::delay(duration));
        HapticPatternBuilder {
            commands: self.commands,
            width: self.width,
            phantom: PhantomData::default(),
        }
    }

    /// This function finalizes the current pattern and repeats it `n` times
    /// # Example usage:
    /// ```
    /// use haptic_lib::HapticPattern;
    /// let pattern = HapticPattern::builder().torque(1.0).torque(-1.0).repeated(3);
    /// ```
    pub fn repeated(self, n: u16) -> HapticPattern {
        HapticPattern {
            commands: self.commands,
            repeat: n,
            multiply: 1,
        }
    }

    /// This function finalizes the current pattern and duplicated every item within it `n` times in place.
    /// This can be used to "stretch" a pattern.
    /// # Example usage:
    /// ```
    /// use haptic_lib::HapticPattern;
    /// let pattern = HapticPattern::builder().torque(1.0).torque(-1.0).multiplied(2);
    /// ```
    pub fn multiplied(self, n: u16) -> HapticPattern {
        HapticPattern {
            commands: self.commands,
            repeat: 1,
            multiply: n,
        }
    }

    /// This function finalizes the current pattern and applies both multiply and repeat modifiers.
    /// Check out [`Self::repeated`] and [`Self::multiplied`] for more information
    /// # Example usage:
    /// ```
    /// use haptic_lib::HapticPattern;
    /// let pattern = HapticPattern::builder().torque(1.0).torque(-1.0).multiplied_and_repeated(3, 2);
    /// ```
    pub fn multiplied_and_repeated(self, repeat: u16, multiply: u16) -> HapticPattern {
        HapticPattern {
            commands: self.commands,
            repeat,
            multiply,
        }
    }

    /// Finish building this command sequence "as-is" without applying an additional modifiers to it
    /// # Example usage:
    /// ```
    /// use haptic_lib::HapticPattern;
    /// let pattern = HapticPattern::builder().torque(1.0).finish();
    /// ```
    pub fn finish(self) -> HapticPattern {
        HapticPattern {
            commands: self.commands,
            repeat: 1,
            multiply: 1,
        }
    }
}

/// Builder with context about the next pattern which is about to be added
pub struct SequenceBuilder<T> {
    root_builder: Builder<T>,
    sequence: SequenceComponent,
}

impl SequenceBuilder<_NonEmpty> {
    /// Repeat the whole component which was just constructed `n` times.
    /// This repeats everything, including the space before the pattern as well.
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticPattern, PatternLayer};
    /// let layer = PatternLayer::builder()
    ///        .at_zero(HapticPattern::builder().torque(1.0).finish())
    ///        .insert_repeated(2)
    ///        .build(0.2, 0.8);
    /// ```
    pub fn insert_repeated(mut self, n: usize) -> Builder<_NonEmpty> {
        self.sequence.repeat = n;
        self.root_builder.components.push(self.sequence);
        self.root_builder
    }
}

impl<T> SequenceBuilder<T> {
    /// Insert the just constructed component once into the layer
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticPattern, PatternLayer};
    /// let layer = PatternLayer::builder()
    ///        .at_zero(HapticPattern::builder().torque(1.0).finish())
    ///        .insert_once()
    ///        .build(0.2, 0.8);
    /// ```
    pub fn insert_once(mut self) -> Builder<_NonEmpty> {
        self.root_builder.components.push(self.sequence);
        Builder {
            components: self.root_builder.components,
            phantom: PhantomData::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    #[test]
    fn test_builder_zero_width_as_first_element() {
        let _ = PatternLayer::builder()
            .at_zero(HapticPattern::builder().torque(1.0).finish())
            .insert_once()
            .build(0.2, 0.8);
    }

    #[test]
    fn test_builder_transition_from_zero_width() {
        let _ = PatternLayer::builder()
            .at_zero(HapticPattern::builder().torque(1.0).finish())
            .insert_once()
            .with_space(
                2.0,
                HapticPattern::builder().torque(0.5).torque(0.0).repeated(2),
            )
            .insert_repeated(10)
            .build(0.2, 0.8);
    }
}
