use atomic_float::AtomicF32;
use embassy_sync::pubsub::{Publisher, Subscriber};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

pub type KnobTiltSubscriber<M, const CAP: usize, const SUBS: usize, const PUBS: usize> =
    Subscriber<'static, M, KnobTiltEvent, CAP, SUBS, PUBS>;
pub type KnobTiltPublisher<M, const CAP: usize, const SUBS: usize, const PUBS: usize> =
    Publisher<'static, M, KnobTiltEvent, CAP, SUBS, PUBS>;

pub static KNOB_TILT_ANGLE: AtomicF32 = AtomicF32::new(0.0);
pub static KNOB_TILT_MAGNITUDE: AtomicF32 = AtomicF32::new(0.0);

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum TiltDirection {
    Up,
    Right,
    Down,
    Left,
}

#[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
pub enum KnobTiltEvent {
    TiltStart(TiltDirection),
    TiltAdjust,
    TiltEnd,
    PressStart,
    PressEnd,
}
