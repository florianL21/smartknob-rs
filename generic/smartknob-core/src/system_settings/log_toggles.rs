#[cfg(feature = "host")]
use clap::ValueEnum;
use embassy_sync::watch::{self, Watch};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

pub type LogToggleSender = watch::DynSender<'static, LogChannelToggles>;
pub type LogToggleReceiver = watch::DynReceiver<'static, LogChannelToggles>;
pub type LogToggleWatcher<M, const N: usize> = Watch<M, LogChannelToggles, N>;

macro_rules! log_toggles {
    ($($variant:ident, $enum:ident),*) => {
        #[derive(Clone, Debug, Default, Serialize, Deserialize, MaxSize)]
        pub struct LogChannelToggles {
            $($variant: bool),+
        }

        #[derive(Deserialize, Serialize, Debug, Clone, MaxSize)]
        #[cfg_attr(feature = "host", derive(ValueEnum))]
        pub enum LogChannel {
            $($enum),+
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

pub async fn may_log<C>(receiver: &mut LogToggleReceiver, channel: LogChannel, c: C)
where
    C: FnOnce(),
{
    let toggles = receiver.get().await;
    if toggles.should_log(channel) {
        c();
    }
}
