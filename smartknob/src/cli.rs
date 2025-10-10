use alloc::format;
use core::{convert::Infallible, fmt::Debug, str::Utf8Error};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_cli::{cli::CliBuilder, Command, CommandGroup};
use embedded_io_async::Read;
use esp_backtrace as _;
use esp_hal::{usb_serial_jtag::UsbSerialJtag, Async};
use log::info;
use thiserror::Error;
use ufmt::{derive::uDebug, uDisplay, uwrite};

use crate::flash::{FlashError, FlashType};

pub enum LogChannel {
    Encoder,
    PushEvents,
}

#[derive(Clone, uDebug, Default)]
pub struct LogToggles {
    active: bool,
    pub encoder: bool,
    pub push_events: bool,
}

pub async fn may_log<C>(
    receiver: &mut embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, LogToggles, 4>,
    channel: LogChannel,
    c: C,
) where
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

struct Context {
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, LogToggles, 4>,
    logging_config: LogToggles,
    interface_open: bool,
    flash: FlashType<'static>,
}

fn set_log(toggles: &mut LogToggles, channel: &str, state: bool) -> Result<(), ()> {
    match channel {
        "encoder" => {
            toggles.encoder = state;
        }
        "push_events" => {
            toggles.push_events = state;
        }
        _ => {
            return Err(());
        }
    }
    Ok(())
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

#[derive(Error, Debug)]
pub enum HandlerError {
    #[error("Flash read failed: {0:#?}")]
    FlashReadError(ekv::ReadError<FlashError>),
    #[error("Flash write failed: {0:#?}")]
    FlashWriteError(ekv::WriteError<FlashError>),
    #[error("Flash format failed: {0:#?}")]
    FlashFormatError(ekv::FormatError<FlashError>),
    #[error("Flash commit failed: {0:#?}")]
    FlashCommitError(ekv::CommitError<FlashError>),
    #[error("Infallible error")]
    Infallible(#[from] Infallible),
    #[error("Conversion to UTF-8 failed: {0:#?}")]
    StringConversionError(#[from] Utf8Error),
}

impl uDisplay for HandlerError {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        f.write_str(format!("{self:?}").as_str())
    }
}

impl From<ekv::ReadError<FlashError>> for HandlerError {
    fn from(value: ekv::ReadError<FlashError>) -> Self {
        HandlerError::FlashReadError(value.into())
    }
}

impl From<ekv::WriteError<FlashError>> for HandlerError {
    fn from(value: ekv::WriteError<FlashError>) -> Self {
        HandlerError::FlashWriteError(value.into())
    }
}

impl From<ekv::FormatError<FlashError>> for HandlerError {
    fn from(value: ekv::FormatError<FlashError>) -> Self {
        HandlerError::FlashFormatError(value.into())
    }
}

impl From<ekv::CommitError<FlashError>> for HandlerError {
    fn from(value: ekv::CommitError<FlashError>) -> Self {
        HandlerError::FlashCommitError(value.into())
    }
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
                    "Available logging channels: {:?}",
                    context.logging_config
                )?;
            }
            Logging::LogEnable { channel } => {
                if set_log(&mut context.logging_config, channel, true).is_ok() {
                    uwrite!(cli.writer(), "Enabled logging for channel {}", channel)?;
                    context.sender.send(context.logging_config.clone());
                } else {
                    uwrite!(cli.writer(), "Invalid channel {}", channel)?;
                }
            }
            Logging::LogDisable { channel } => {
                if set_log(&mut context.logging_config, channel, false).is_ok() {
                    uwrite!(cli.writer(), "Disabled logging for channel {}", channel)?;
                    context.sender.send(context.logging_config.clone());
                } else {
                    uwrite!(cli.writer(), "Invalid channel {}", channel)?;
                }
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
                wt.read(&[0], &mut data).await?;
                let s = str::from_utf8(&data)?;
                uwrite!(cli.writer(), "Stored value is: {}", s)?;
            }
            Flash::FlashStore { value } => {
                let mut wt = context.flash.write_transaction().await;
                wt.write(&[0], value.as_bytes()).await?;
                wt.commit().await?;
                uwrite!(cli.writer(), "Saved value to flash: {}", value)?;
            }
            Flash::FlashFormat => {
                context.flash.format().await?;
                uwrite!(cli.writer(), "Formatted flash")?;
            }
        }
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn menu_handler(
    serial: UsbSerialJtag<'static, Async>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, LogToggles, 4>,
    flash: FlashType<'static>,
) {
    let mut context = Context {
        sender,
        logging_config: LogToggles::default(),
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
                        let _ = uwrite!(cli.writer(), "Failed to handle command: {}", e);
                    }
                    Ok(())
                })
                .await,
            )
            .await;
    }
}
