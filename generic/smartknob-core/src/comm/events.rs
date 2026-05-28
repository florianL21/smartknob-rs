use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

use crate::knob_tilt::KnobTiltEvent;

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum Event {
    EncoderAngle(f32),
    Button(KnobTiltEvent),
    Tilt {
        angle: f32,
        magnitude: f32,
    },
    /// Informs the host about how namy events were missed by the embedded system
    MissedEvents(u64),
}
