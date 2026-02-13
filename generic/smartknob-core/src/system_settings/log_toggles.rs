use core::str::FromStr;

use embassy_sync::watch::{self, Watch};
use embedded_cli::arguments::FromArgumentError;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use ufmt::{derive::uDebug, uDisplay, uwriteln};

pub type LogToggleSender = watch::DynSender<'static, LogToggles>;
pub type LogToggleReceiver = watch::DynReceiver<'static, LogToggles>;
pub type LogToggleWatcher<M, const N: usize> = Watch<M, LogToggles, N>;

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

            pub fn set(&mut self, channel: &LogChannel, state: bool) {
                match channel {
                    $(LogChannel::$enum => self.$variant = state,)+
                }
            }
        }

        impl FromStr for LogChannel {
            type Err = ();
            fn from_str(s: &str) -> Result<Self, Self::Err> {
                Ok(match s {
                    $(stringify!($variant) => LogChannel::$enum,)+
                    $(stringify!($enum) => LogChannel::$enum,)+
                    _ => {
                        return Err(());
                    }
                })
            }
        }

        impl uDisplay for LogChannel {
            fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
            where
                W: ufmt::uWrite + ?Sized,
            {
                match self {
                    $(LogChannel::$enum => f.write_str(stringify!($enum)),)+
                }
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
    DisplayTransfer,
    foc_loop,
    FOCLoop
);

impl<'a> embedded_cli::arguments::FromArgument<'a> for LogChannel {
    fn from_arg(arg: &'a str) -> Result<Self, FromArgumentError<'a>>
    where
        Self: Sized,
    {
        arg.parse().map_err(|_| FromArgumentError {
            value: arg,
            expected: "a valid LogChannel enum variant",
        })
    }
}

#[derive(Clone, Debug, uDebug, Default)]
pub struct LogToggles {
    pub active: bool,
    pub config: LogChannelToggles,
}

impl LogToggles {
    pub fn should_log(&self, channel: LogChannel) -> bool {
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
