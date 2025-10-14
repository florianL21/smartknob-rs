use alloc::{
    format,
    string::{String, ToString},
};
use core::{convert::Infallible, fmt::Debug, str::Utf8Error};
use embedded_cli::{cli::CliBuilder, Command, CommandGroup};
use embedded_io_async::Read;
use esp_backtrace as _;
use esp_hal::{usb_serial_jtag::UsbSerialJtag, Async};
use postcard::experimental::max_size::MaxSize;
use thiserror::Error;
use ufmt::{uDebug, uwrite};

use crate::{
    config::{ConfigError, LogChannelToggles, LogToggleSender, LogToggles},
    flash::{FlashError, FlashKeys, FlashType},
};

#[derive(Error, Debug)]
pub enum HandlerError {
    #[error("Flash read failed: {0:#?}")]
    FlashError(#[from] FlashError),
    #[error("Infallible error")]
    Infallible(#[from] Infallible),
    #[error("Conversion to UTF-8 failed: {0:#?}")]
    StringConversionError(#[from] Utf8Error),
    #[error("Failed to set config {0}")]
    InvalidConfigError(#[from] ConfigError),
    #[error("Could not serialize {0} to postcard format")]
    PostcardSerializationError(String),
}

impl uDebug for HandlerError {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        f.write_str(format!("{self:?}").as_str())
    }
}

struct Context {
    sender: LogToggleSender,
    logging_config: LogToggles,
    interface_open: bool,
    flash: FlashType<'static>,
}

#[derive(CommandGroup)]
enum RootGroup<'a> {
    Base(Base),
    Logging(Logging<'a>),
    Storage(Flash<'a>),
}

#[derive(Command)]
#[command(help_title = "Basic commands")]
enum Base {
    /// Exit the CLI. This will start logging out all enabled channels. To start the CLI again simply press any button
    Exit,
}

#[derive(Command)]
#[command(help_title = "Manage Logging output")]
enum Logging<'a> {
    /// Enable
    LogEnable {
        /// Which logging channel to enable
        channel: &'a str,
    },
    ///
    LogDisable {
        /// Which logging channel to disable
        channel: &'a str,
    },
    /// List all available logging channels
    LogList,
}

#[derive(Command)]
#[command(help_title = "Manage values stored in flash")]
enum Flash<'a> {
    /// Store a value to flash
    FlashStore {
        /// What value to store
        value: &'a str,
    },

    /// Load a value from flash
    FlashLoad,

    /// Format the flash
    FlashFormat,
}

async fn set_log(
    toggles: &mut LogChannelToggles,
    flash: &FlashType<'static>,
    channel: &str,
    state: bool,
) -> Result<(), HandlerError> {
    toggles.set_from_str(channel, state)?;
    let mut wt = flash.write_transaction().await;
    let mut buffer = [0u8; LogChannelToggles::POSTCARD_MAX_SIZE];
    postcard::to_slice(toggles, &mut buffer)
        .map_err(|_| HandlerError::PostcardSerializationError("LogToggles".to_string()))?;
    wt.write(&FlashKeys::LogChannels.key(), &buffer)
        .await
        .map_err(FlashError::from)?;
    wt.commit().await.map_err(FlashError::from)?;
    Ok(())
}

impl Logging<'_> {
    async fn handle(
        self: &Self,
        cli: &mut embedded_cli::cli::CliHandle<
            '_,
            esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>,
            core::convert::Infallible,
        >,
        context: &mut Context,
    ) -> Result<(), HandlerError> {
        match self {
            Logging::LogList => {
                uwrite!(
                    cli.writer(),
                    "Available logging channels: \n{}",
                    context.logging_config.config
                )?;
            }
            Logging::LogEnable { channel } => {
                set_log(
                    &mut context.logging_config.config,
                    &context.flash,
                    channel,
                    true,
                )
                .await?;
                uwrite!(cli.writer(), "Enabled logging for channel {}", channel)?;
                context.sender.send(context.logging_config.clone());
            }
            Logging::LogDisable { channel } => {
                set_log(
                    &mut context.logging_config.config,
                    &context.flash,
                    channel,
                    false,
                )
                .await?;
                uwrite!(cli.writer(), "Disabled logging for channel {}", channel)?;
                context.sender.send(context.logging_config.clone());
            }
        }
        Ok(())
    }
}

impl Flash<'_> {
    async fn handle(
        self: &Self,
        cli: &mut embedded_cli::cli::CliHandle<
            '_,
            esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>,
            core::convert::Infallible,
        >,
        context: &mut Context,
    ) -> Result<(), HandlerError> {
        match self {
            Flash::FlashLoad => {
                let wt = context.flash.read_transaction().await;
                let mut data = [0u8; 255];
                wt.read(&FlashKeys::Test.key(), &mut data)
                    .await
                    .map_err(FlashError::from)?;
                let s = str::from_utf8(&data)?;
                uwrite!(cli.writer(), "Stored value is: {}", s)?;
            }
            Flash::FlashStore { value } => {
                let mut wt = context.flash.write_transaction().await;
                wt.write(&FlashKeys::Test.key(), value.as_bytes())
                    .await
                    .map_err(FlashError::from)?;
                wt.commit().await.map_err(FlashError::from)?;
                uwrite!(cli.writer(), "Saved value to flash: {}", value)?;
            }
            Flash::FlashFormat => {
                context.flash.format().await.map_err(FlashError::from)?;
                uwrite!(cli.writer(), "Formatted flash")?;
            }
        }
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn menu_handler(
    serial: UsbSerialJtag<'static, Async>,
    sender: LogToggleSender,
    flash: FlashType<'static>,
    initial_log_toggles: LogChannelToggles,
) {
    let mut context = Context {
        sender,
        logging_config: LogToggles {
            active: true,
            config: initial_log_toggles,
        },
        interface_open: false,
        flash,
    };

    let command_buffer = [0u8; 255];
    let history_buffer = [0u8; 255];
    let (mut rx, tx) = serial.split();
    let builder = CliBuilder::default()
        .writer(tx)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build();
    let mut cli = match builder {
        Ok(cli) => cli,
        _ => {
            return;
        }
    };

    context.sender.send(context.logging_config.clone());

    let mut buf: [u8; 1] = [0; 1];
    loop {
        // read byte from somewhere (for example, uart)
        let _ = rx.read(&mut buf).await;

        // if the interface is not open open it and skip processing the CLI
        if context.interface_open == false {
            if buf[0] != 13 {
                continue;
            }
            context.interface_open = true;
            context.logging_config.active = false;
            context.sender.send(context.logging_config.clone());
        }

        let _ = cli
            .process_byte::<RootGroup, _>(
                buf[0],
                &mut RootGroup::processor(async |cli, command| {
                    let res = match command {
                        RootGroup::Logging(logging) => logging.handle(cli, &mut context).await,
                        RootGroup::Storage(storage) => storage.handle(cli, &mut context).await,
                        RootGroup::Base(base) => match base {
                            Base::Exit => {
                                context.interface_open = false;
                                context.logging_config.active = true;
                                context.sender.send(context.logging_config.clone());
                                Ok(())
                            }
                        },
                    };
                    if let Err(e) = res {
                        let _ = uwrite!(cli.writer(), "Failed to handle command: {:?}", e);
                    }
                    Ok(())
                })
                .await,
            )
            .await;
    }
}
