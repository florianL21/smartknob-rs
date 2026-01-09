use alloc::format;
use core::{convert::Infallible, fmt::Debug, str::Utf8Error};
use embassy_time::Duration;
use embedded_cli::{Command, CommandGroup, cli::CliBuilder};
use embedded_io_async::{Read, Write};
use esp_backtrace as _;
use esp_hal::{Async, usb_serial_jtag::UsbSerialJtag};
use fixed::types::I16F16;
use log::info;
use postcard::experimental::max_size::MaxSize;
use thiserror::Error;
use ufmt::{uDebug, uwrite};

use crate::{
    config::{ConfigError, LogChannelToggles, LogToggleSender, LogToggles},
    display::DISPLAY_BRIGHTNESS_SIGNAL,
    flash::{FlashError, FlashHandler, FlashKeys},
    motor_control::MotorCommand,
    signals::{MOTOR_COMMAND_SIGNAL, REQUEST_POWER_DOWN},
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
    #[error("Could not serialize LogToggles to postcard format")]
    PostcardLogToggleSerializationError,
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
    flash: &'static FlashHandler,
}

#[derive(CommandGroup)]
enum RootGroup<'a> {
    Base(Base),
    Logging(Logging<'a>),
    Storage(Flash),
    Motor(Motor),
    Display(Display),
}

#[derive(Command)]
#[command(help_title = "Basic commands")]
enum Base {
    /// Exit the CLI. This will start logging out all enabled channels. To start the CLI again simply press any button
    Exit,
    /// This will initiate a system shutdown
    Shutdown,
}

// This is okay because these names will be converted to CLI commands, and they need the additional context
#[allow(clippy::enum_variant_names)]
#[derive(Command)]
#[command(help_title = "Manage Logging output")]
enum Logging<'a> {
    /// Enable printing of a certain log cannel
    LogEnable {
        /// Which logging channel to enable
        channel: &'a str,
    },
    /// Disable printing of a certain log channel
    LogDisable {
        /// Which logging channel to disable
        channel: &'a str,
    },
    /// List all available logging channels
    LogList,
}

async fn set_log(
    toggles: &mut LogChannelToggles,
    flash: &FlashHandler,
    channel: &str,
    state: bool,
) -> Result<(), HandlerError> {
    toggles.set_from_str(channel, state)?;
    flash
        .store::<_, { LogChannelToggles::POSTCARD_MAX_SIZE }>(&FlashKeys::LogChannels, toggles)
        .await?;
    Ok(())
}

impl Logging<'_> {
    async fn handle(
        &self,
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
                    context.flash,
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
                    context.flash,
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

#[derive(Command)]
#[command(help_title = "Manage values stored in flash")]
enum Flash {
    /// Format the flash
    FlashFormat,
}

impl Flash {
    async fn handle(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<
            '_,
            esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>,
            core::convert::Infallible,
        >,
        context: &mut Context,
    ) -> Result<(), HandlerError> {
        match self {
            Flash::FlashFormat => {
                context.flash.format().await?;
                uwrite!(cli.writer(), "Formatted flash")?;
            }
        }
        Ok(())
    }
}

#[derive(Command)]
#[command(help_title = "Manage values stored in flash")]
enum Motor {
    /// Align the sensor and the motor to the electrical zero position
    Align,
    /// Measure the encoder for non-linearity and give a verdict if the error is too big for proper operation
    EncoderValidate,
    /// Increase the electrical zero offset value of the FOC system
    TuneUp {
        /// Value to change the electrical angle offset by
        value: f32,
    },
    /// Reduce the electrical zero offset value of the FOC system
    TuneDown {
        /// Value to change the electrical angle offset by
        value: f32,
    },
    /// Save the modified current electrical angle offset to flash
    TuneStore,
    /// Make a beep sound
    Beep {
        /// Frequency of the beep
        freq: f32,
        /// duration in ms
        duration: u64,
        /// volume in % between 0 and 100
        volume: f32,
        /// Note offsetfreq
        note_offset: u32,
    },
}

impl Motor {
    async fn handle(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<
            '_,
            esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>,
            core::convert::Infallible,
        >,
        _context: &mut Context,
    ) -> Result<(), HandlerError> {
        match self {
            Motor::Align => {
                uwrite!(cli.writer(), "Starting motor alignment")?;
                MOTOR_COMMAND_SIGNAL.signal(MotorCommand::StartAlignment);
            }
            Motor::EncoderValidate => {
                uwrite!(
                    cli.writer(),
                    "Validating encoder linearity. Do not touch the motor!"
                )?;
                MOTOR_COMMAND_SIGNAL.signal(MotorCommand::VerifyEncoder);
            }
            Motor::TuneUp { value } => {
                MOTOR_COMMAND_SIGNAL.signal(MotorCommand::TuneAlignment(I16F16::from_num(*value)));
            }
            Motor::TuneDown { value } => {
                MOTOR_COMMAND_SIGNAL.signal(MotorCommand::TuneAlignment(I16F16::from_num(-*value)));
            }
            Motor::TuneStore => {
                MOTOR_COMMAND_SIGNAL.signal(MotorCommand::TuneStore);
            }
            Motor::Beep {
                freq,
                duration,
                volume,
                note_offset,
            } => {
                MOTOR_COMMAND_SIGNAL.signal(MotorCommand::Beep {
                    freq: I16F16::from_num(*freq),
                    duration: Duration::from_millis(*duration),
                    volume: I16F16::from_num(*volume),
                    note_offset: *note_offset,
                });
            }
        }
        Ok(())
    }
}

#[derive(Command)]
#[command(help_title = "Manage settings related to the display")]
enum Display {
    /// Set the backlight brightness of the display
    Brightness {
        /// Brightness in percent 0-100
        percent: u8,
    },
}

impl Display {
    async fn handle(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<
            '_,
            esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>,
            core::convert::Infallible,
        >,
        _context: &mut Context,
    ) -> Result<(), HandlerError> {
        match self {
            Display::Brightness { percent } => {
                let percent = (*percent).clamp(0, 100);
                uwrite!(cli.writer(), "Setting display brightness to {}%", percent)?;
                DISPLAY_BRIGHTNESS_SIGNAL.signal(percent);
            }
        }
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn menu_handler(
    serial: UsbSerialJtag<'static, Async>,
    flash: &'static FlashHandler,
    log_toggle_sender: LogToggleSender,
) {
    let mut buffer = [0u8; LogChannelToggles::POSTCARD_MAX_SIZE];
    let initial_log_toggles =
        if let Ok(Some(t)) = flash.load(FlashKeys::LogChannels, &mut buffer).await {
            t
        } else {
            LogChannelToggles::default()
        };
    let mut context = Context {
        sender: log_toggle_sender,
        logging_config: LogToggles {
            active: true,
            config: initial_log_toggles,
        },
        interface_open: false,
        flash,
    };
    context.sender.send(context.logging_config.clone());

    let command_buffer = [0u8; 255];
    let history_buffer = [0u8; 255];
    let (mut rx, mut tx) = serial.split();

    // Very hacky workaround for the CliBuilder::build() function not being async.
    // These lines will attempt to write and flush _something_.
    // In theory this future should never complete in case no USB device is connected
    Write::write(&mut tx, b"\0").await.ok();
    Write::flush(&mut tx).await.ok();
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

    info!("CLI is ready and operational");

    let mut buf: [u8; 1] = [0; 1];
    loop {
        // read byte from somewhere (for example, uart)
        let _ = rx.read(&mut buf).await;

        // if the interface is not open open it and skip processing the CLI
        if !context.interface_open {
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
                        RootGroup::Motor(motor) => motor.handle(cli, &mut context).await,
                        RootGroup::Display(display) => display.handle(cli, &mut context).await,
                        RootGroup::Base(base) => match base {
                            Base::Exit => {
                                context.interface_open = false;
                                context.logging_config.active = true;
                                context.sender.send(context.logging_config.clone());
                                Ok(())
                            }
                            Base::Shutdown => {
                                REQUEST_POWER_DOWN.signal(true);
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
