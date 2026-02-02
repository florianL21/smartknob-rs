use embassy_executor::SpawnError;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use thiserror::Error;

pub static DISPLAY_BRIGHTNESS_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();
pub static UI_READY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[derive(Error, Debug)]
pub enum DisplayTaskError {
    #[error("Log receiver has no more capacity. Increase the max number of log receivers")]
    LogReceiverOutOfCapacity,
    #[error("Failed to spawn at least one required task: {0}")]
    FailedToSpawnTask(#[from] SpawnError),
}

pub trait BrightnessSensor {
    fn sample(&mut self) -> Option<u16>;
}
