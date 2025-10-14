use alloc::string::{String, ToString};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use ufmt::{derive::uDebug, uDisplay, uwrite, uwriteln};

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

#[derive(Error, Debug)]
pub enum ConfigError {
    #[error("There is no logging channel called {0}")]
    InvalidLogChannelError(String),
}

#[derive(Clone, Debug, uDebug, Default, Serialize, Deserialize, MaxSize)]
pub struct LogChannelToggles {
    encoder: bool,
    push_events: bool,
}

impl uDisplay for LogChannelToggles {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        uwriteln!(f, "Channel:       | Enabled:")?;
        uwriteln!(f, "---------------|---------")?;
        uwriteln!(f, "encoder        | {}", self.encoder)?;
        uwrite!(f, "push_events    | {}", self.push_events)
    }
}

impl LogChannelToggles {
    fn should_log(&self, channel: LogChannel) -> bool {
        match channel {
            LogChannel::Encoder => self.encoder,
            LogChannel::PushEvents => self.push_events,
        }
    }

    pub fn set_from_str(&mut self, channel: &str, state: bool) -> Result<(), ConfigError> {
        match channel {
            "encoder" => {
                self.encoder = state;
            }
            "push_events" => {
                self.push_events = state;
            }
            c => {
                return Err(ConfigError::InvalidLogChannelError(c.to_string()));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, uDebug, Default)]
pub struct LogToggles {
    pub active: bool,
    pub config: LogChannelToggles,
}

impl LogToggles {
    fn should_log(&self, channel: LogChannel) -> bool {
        self.active && self.config.should_log(channel)
    }
}

pub async fn may_log<C>(receiver: &mut LogToggleReceiver, channel: LogChannel, c: C)
where
    C: FnOnce(),
{
    let toggles = receiver.get().await;
    if toggles.should_log(channel) {
        c();
    }
}
