use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

pub use crate::knob_tilt::{KNOB_EVENTS_CHANNEL, KNOB_TILT_ANGLE, KNOB_TILT_MAGNITUDE};
pub use crate::shutdown::REQUEST_POWER_DOWN;
pub use smartknob_core::haptic_core::MOTOR_COMMAND_SIGNAL;

pub static DISPLAY_BRIGHTNESS_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();
