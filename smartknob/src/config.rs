use alloc::string::{String, ToString};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{self, Watch},
};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use ufmt::{derive::uDebug, uDisplay, uwriteln};

const NUM_LOG_TOGGLE_WATCHER_RECEIVERS: usize = 5;

pub type LogToggleSender =
    watch::Sender<'static, CriticalSectionRawMutex, LogToggles, NUM_LOG_TOGGLE_WATCHER_RECEIVERS>;
pub type LogToggleReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, LogToggles, NUM_LOG_TOGGLE_WATCHER_RECEIVERS>;

pub static LOG_TOGGLES: Watch<
    CriticalSectionRawMutex,
    LogToggles,
    NUM_LOG_TOGGLE_WATCHER_RECEIVERS,
> = Watch::new();

#[derive(Error, Debug)]
pub enum ConfigError {
    #[error("There is no logging channel called {0}")]
    InvalidLogChannelError(String),
}

macro_rules! log_toggles {
    ($($variant:ident, $enum:ident),*) => {
        #[derive(Clone, Debug, uDebug, Default, Serialize, Deserialize, MaxSize)]
        pub struct LogChannelToggles {
            $($variant: bool),+
        }

        pub enum LogChannel {
            $($enum),+
        }


        impl uDisplay for LogChannelToggles {
            fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
            where
                W: ufmt::uWrite + ?Sized,
            {
                uwriteln!(f, "Channel:       | Enabled:")?;
                uwriteln!(f, "---------------|---------")?;
                $(uwriteln!(f, "{} | {}", stringify!($variant), self.$variant)?;)+
                Ok(())
            }
        }
        impl LogChannelToggles {
            fn should_log(&self, channel: LogChannel) -> bool {
                match channel {
                    $(LogChannel::$enum => self.$variant),+
                }
            }

            pub fn set_from_str(&mut self, channel: &str, state: bool) -> Result<(), ConfigError> {
                match channel {
                    $(stringify!($variant) => self.$variant = state,)+
                    c => {
                        return Err(ConfigError::InvalidLogChannelError(c.to_string()));
                    }
                }
                Ok(())
            }
        }
    };
}

log_toggles!(
    encoder,
    Encoder,
    push,
    Push,
    brightness,
    Brightness,
    render,
    Render,
    display_transfer,
    DisplayTransfer
);

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
