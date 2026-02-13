pub mod log_toggles;
use embassy_sync::{blocking_mutex::raw::RawMutex, signal::Signal};

use crate::{haptic_core::CalibrationData, system_settings::log_toggles::LogChannelToggles};

pub type HapticSystemStoreSignal<M> = Signal<M, CalibrationData>;
pub type LogTogglesStoreSignal<M> = Signal<M, LogChannelToggles>;

pub struct StoreSignals<M: RawMutex + 'static> {
    pub haptic_core: HapticSystemStoreSignal<M>,
    pub log_toggles: LogTogglesStoreSignal<M>,
}

impl<M: RawMutex> StoreSignals<M> {
    pub fn new() -> Self {
        Self {
            haptic_core: Signal::new(),
            log_toggles: Signal::new(),
        }
    }
}

impl<M: RawMutex> Default for StoreSignals<M> {
    fn default() -> Self {
        Self::new()
    }
}
