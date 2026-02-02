use crate::display::DISPLAY_BRIGHTNESS_SIGNAL;
use crate::flash::{FlashError, FlashHandling, FlashKeys};
use crate::haptic_core::{MOTOR_COMMAND_SIGNAL, MotorCommand};
use crate::shutdown::REQUEST_POWER_DOWN;
use crate::system_settings::log_toggles::{
    LogChannel, LogChannelToggles, LogToggleSender, LogToggles,
};
use core::fmt::Debug;
use core::fmt::Write as _;
use embassy_time::Duration;
use embedded_cli::{Command, CommandGroup, cli::CliBuilder};
use embedded_io_async::{Read, Write};
use embedded_storage::nor_flash::NorFlash;
use fixed::types::I16F16;
use log::info;
use postcard::experimental::max_size::MaxSize;
use thiserror::Error;
use ufmt::uwrite;

#[derive(Error, Debug)]
pub enum HandlerError<FE, IO: embedded_io::Error> {
    #[error("Flash read failed: {0:#?}")]
    FlashError(#[from] FlashError<FE>),
    #[error("Failed to write to console {0:#?}")]
    ConsoleWriteError(#[from] IO),
    #[error("Could not serialize LogToggles to postcard format")]
    PostcardLogToggleSerializationError,
}

struct Context {
    sender: LogToggleSender,
    logging_config: LogToggles,
    interface_open: bool,
}

#[derive(CommandGroup)]
enum RootGroup {
    Base(Base),
    Logging(Logging),
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
enum Logging {
    /// Enable printing of a certain log cannel
    LogEnable {
        /// Which logging channel to enable
        channel: LogChannel,
    },
    /// Disable printing of a certain log channel
    LogDisable {
        /// Which logging channel to disable
        channel: LogChannel,
    },
    /// List all available logging channels
    LogList,
}

async fn set_log<F: NorFlash>(
    toggles: &mut LogChannelToggles,
    flash: &impl FlashHandling<F>,
    channel: &LogChannel,
    state: bool,
) -> Result<(), FlashError<F::Error>> {
    toggles.set(channel, state);
    flash
        .store::<_, { LogChannelToggles::POSTCARD_MAX_SIZE }>(&FlashKeys::LogChannels, toggles)
        .await?;
    Ok(())
}

impl Logging {
    async fn handle<TX: embedded_io::Write, F: FlashHandling<FT>, FT: NorFlash, FE>(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<'_, TX, TX::Error>,
        context: &mut Context,
        flash: &F,
    ) -> Result<(), HandlerError<FE, TX::Error>>
    where
        HandlerError<FE, <TX as embedded_io::ErrorType>::Error>:
            From<FlashError<<FT as embedded_storage::nor_flash::ErrorType>::Error>>,
    {
        match self {
            Logging::LogList => {
                uwrite!(
                    cli.writer(),
                    "Available logging channels: \n{}",
                    context.logging_config.config
                )?;
            }
            Logging::LogEnable { channel } => {
                set_log(&mut context.logging_config.config, flash, channel, true).await?;
                uwrite!(cli.writer(), "Enabled logging for channel {}", channel)?;
                context.sender.send(context.logging_config.clone());
            }
            Logging::LogDisable { channel } => {
                set_log(&mut context.logging_config.config, flash, channel, false).await?;
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
    async fn handle<TX: embedded_io::Write, F: FlashHandling<FT>, FT: NorFlash, FE>(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<'_, TX, TX::Error>,
        flash: &F,
    ) -> Result<(), HandlerError<FE, TX::Error>>
    where
        HandlerError<FE, <TX as embedded_io::ErrorType>::Error>:
            From<FlashError<<FT as embedded_storage::nor_flash::ErrorType>::Error>>,
    {
        match self {
            Flash::FlashFormat => {
                flash.format().await?;
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
    async fn handle<TX: embedded_io::Write, FE>(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<'_, TX, TX::Error>,
    ) -> Result<(), HandlerError<FE, TX::Error>> {
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
    async fn handle<TX: embedded_io::Write, FE>(
        &self,
        cli: &mut embedded_cli::cli::CliHandle<'_, TX, TX::Error>,
    ) -> Result<(), HandlerError<FE, TX::Error>> {
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

pub struct CLIHandler<TX: Write + embedded_io::Write> {
    context: Context,
    cli: embedded_cli::cli::Cli<TX, TX::Error, [u8; 255], [u8; 255]>,
}

impl<TX: Write + embedded_io::Write> CLIHandler<TX> {
    pub async fn new<F: NorFlash>(
        flash: &'static impl FlashHandling<F>,
        log_toggle_sender: LogToggleSender,
        mut tx: TX,
    ) -> Option<Self> {
        let mut buffer = [0u8; LogChannelToggles::POSTCARD_MAX_SIZE];
        let initial_log_toggles =
            if let Ok(Some(t)) = flash.load(FlashKeys::LogChannels, &mut buffer).await {
                t
            } else {
                LogChannelToggles::default()
            };
        let context = Context {
            sender: log_toggle_sender,
            logging_config: LogToggles {
                active: true,
                config: initial_log_toggles,
            },
            interface_open: false,
        };
        context.sender.send(context.logging_config.clone());

        let command_buffer = [0u8; 255];
        let history_buffer = [0u8; 255];

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
        let cli = match builder {
            Ok(cli) => cli,
            _ => {
                return None;
            }
        };

        info!("CLI is ready and operational");
        Some(Self { context, cli })
    }

    pub async fn run<F: NorFlash, RX: Read>(
        &mut self,
        rx: &mut RX,
        flash: &'static impl FlashHandling<F>,
    ) {
        let mut buf: [u8; 1] = [0; 1];
        // read byte from somewhere (for example, uart)
        let _ = rx.read(&mut buf).await;

        // if the interface is not open open it and skip processing the CLI
        if !self.context.interface_open {
            if buf[0] != 13 {
                return;
            }
            self.context.interface_open = true;
            self.context.logging_config.active = false;
            self.context
                .sender
                .send(self.context.logging_config.clone());
        }

        let _ = self
            .cli
            .process_byte::<RootGroup, _>(
                buf[0],
                &mut RootGroup::processor(async |cli, command| {
                    let res = match command {
                        RootGroup::Logging(logging) => {
                            logging.handle(cli, &mut self.context, flash).await
                        }
                        RootGroup::Storage(storage) => storage.handle(cli, flash).await,
                        RootGroup::Motor(motor) => motor.handle(cli).await,
                        RootGroup::Display(display) => display.handle(cli).await,
                        RootGroup::Base(base) => match base {
                            Base::Exit => {
                                self.context.interface_open = false;
                                self.context.logging_config.active = true;
                                self.context
                                    .sender
                                    .send(self.context.logging_config.clone());
                                Ok(())
                            }
                            Base::Shutdown => {
                                REQUEST_POWER_DOWN.signal(true);
                                Ok(())
                            }
                        },
                    };
                    if let Err(e) = res {
                        let _ = write!(cli.writer(), "Failed to handle command: {}", e);
                    }
                    Ok(())
                })
                .await,
            )
            .await;
    }
}
