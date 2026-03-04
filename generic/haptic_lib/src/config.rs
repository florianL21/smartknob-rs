use crate::{
    Angle, CurveError, CurveInstance, HapticCurve, PatternLayer,
    curve::CurveState,
    pattern::{PatternLayerError, PatternLayerState},
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum ConfigError {
    #[error("Error with the curve layer")]
    CurveLayerError(#[from] CurveError),
    #[error("Error with the pattern layer")]
    PatternLayerError(#[from] PatternLayerError),
}

#[derive(Debug, Serialize, Deserialize)]
pub struct HapticCurveConfig {
    curve_layer: HapticCurve,
    pattern_layer: Option<PatternLayer>,
}

impl HapticCurveConfig {
    pub(crate) fn new(curve: HapticCurve) -> Self {
        Self {
            curve_layer: curve,
            pattern_layer: None,
        }
    }

    /// Attach a pattern layer to the haptic configuration
    ///
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticCurve, CurveSegment, HapticPattern, PatternLayer};
    ///
    /// let mut builder = HapticCurve::builder();
    /// let segment = builder.new_segment(CurveSegment::new().add_const(5.0, 0.5));
    /// let layer = PatternLayer::builder()
    ///     .at_zero(
    ///         HapticPattern::builder()
    ///             .torque(1.0)
    ///             .finish()
    ///     )
    ///     .insert_once()
    ///     .build(0.2, 0.1)
    ///     .unwrap();
    /// let config = builder.push(segment).finish(1.2).with_pattern_layer(layer);
    /// ```
    pub fn with_pattern_layer(mut self, pattern_layer: PatternLayer) -> HapticConfiguration {
        self.pattern_layer = Some(pattern_layer);
        HapticConfiguration::Curve(self)
    }

    /// Build the haptic configuration without a pattern layer
    ///
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticCurve, CurveSegment};
    ///
    /// let mut builder = HapticCurve::builder();
    /// let segment = builder.new_segment(CurveSegment::new().add_const(5.0, 0.5));
    /// let config = builder.push(segment).finish(1.2).without_pattern_layer();
    /// ```
    pub fn without_pattern_layer(self) -> HapticConfiguration {
        HapticConfiguration::Curve(self)
    }

    /// Get the start angle of the curve
    ///
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticCurve, CurveSegment};
    ///
    /// let mut builder = HapticCurve::builder();
    /// let segment = builder.new_segment(CurveSegment::new().add_const(5.0, 0.5));
    /// let curve = builder.push(segment).finish(1.2);
    /// assert_eq!(curve.start_angle(), 1.2);
    /// ```
    pub fn start_angle(&self) -> Angle {
        self.curve_layer.start_angle
    }
}

/// This holds a set of full configurations for the haptic system.
/// This can fully defines all actions it may ever take while being in this config.
/// This should be fully serializable and can be loaded on runtime
#[derive(Debug, Serialize, Deserialize)]
pub enum HapticConfiguration {
    /// This configuration is a curve based config
    Curve(HapticCurveConfig),
    /// This configuration is a haptic program (yet to be implemented)
    Program,
}

impl HapticConfiguration {
    /// Make an instance of a haptic configuration.
    /// This will validate the configuration and run various consistency checks.
    /// If they pass the returned struct will be ready to be played back
    ///
    /// # Example usage:
    /// ```
    /// use haptic_lib::{HapticCurve, CurveSegment, HapticPlayer};
    ///
    /// let mut builder = HapticCurve::builder();
    /// let segment = builder.new_segment(CurveSegment::new().add_const(5.0, 0.5));
    /// let config = builder
    ///     .push(segment)
    ///     .finish(1.2)
    ///     .without_pattern_layer()
    ///     .instantiate()
    ///     .unwrap();
    /// // Config can now be passed to a haptic player
    /// HapticPlayer::new(0.0, &config);
    /// ```
    pub fn instantiate(self) -> Result<HapticInstances, ConfigError> {
        match self {
            HapticConfiguration::Curve(c) => {
                let curve = c.curve_layer.instantiate()?;
                let pattern = if let Some(p) = c.pattern_layer {
                    Some(p.validate()?)
                } else {
                    None
                };
                Ok(HapticInstances { curve, pattern })
            }
            HapticConfiguration::Program => todo!("Haptic programs are not yet implemented"),
        }
    }
}

#[derive(Debug)]
pub struct HapticInstances {
    curve: CurveInstance,
    pattern: Option<PatternLayer>,
}

#[derive(Debug)]
pub struct HapticStates<'a> {
    pub(crate) curve: CurveState<'a>,
    pub(crate) pattern: Option<(PatternLayerState<'a>, &'a PatternLayer)>,
}

impl HapticInstances {
    pub(crate) fn make_state<'a>(&'a self) -> HapticStates<'a> {
        HapticStates {
            curve: self.curve.make_state(),
            pattern: self.pattern.as_ref().map(|p| (p.make_state(), p)),
        }
    }

    /// Get the width of the whole curve.
    pub fn width(&self) -> Angle {
        let curve_width = self.curve.width();
        if let Some(p) = &self.pattern {
            let pattern_width = p.width();
            if curve_width > pattern_width {
                curve_width
            } else {
                pattern_width
            }
        } else {
            curve_width
        }
    }
}
