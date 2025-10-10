use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use ufmt::derive::uDebug;

const NUM_LOG_TOGGLE_WATCHER_RECEIVERS: usize = 2;

pub type LogToggleSender =
    watch::Sender<'static, CriticalSectionRawMutex, LogToggles, NUM_LOG_TOGGLE_WATCHER_RECEIVERS>;
pub type LogToggleReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, LogToggles, NUM_LOG_TOGGLE_WATCHER_RECEIVERS>;
pub type LogToggleWatcher =
    watch::Watch<CriticalSectionRawMutex, LogToggles, NUM_LOG_TOGGLE_WATCHER_RECEIVERS>;

pub enum LogChannel {
    Encoder,
    PushEvents,
}

#[derive(Clone, Debug, uDebug, Default, Serialize, Deserialize, MaxSize)]
pub struct LogToggles {
    pub active: bool,
    pub encoder: bool,
    pub push_events: bool,
}

pub async fn may_log<C>(receiver: &mut LogToggleReceiver, channel: LogChannel, c: C)
where
    C: FnOnce(),
{
    let toggles = receiver.get().await;
    let status = match channel {
        LogChannel::Encoder => toggles.encoder,
        LogChannel::PushEvents => toggles.push_events,
    };
    if toggles.active && status {
        c();
    }
}
