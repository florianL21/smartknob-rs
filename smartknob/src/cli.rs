use core::{convert::Infallible};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_cli::{cli::CliBuilder, Command, CommandGroup};
use embedded_io_async::Read;
use esp_backtrace as _;
use esp_hal::{usb_serial_jtag::UsbSerialJtag, Async};
use ufmt::{derive::uDebug, uwrite};

#[derive(Clone, uDebug, Default)]
pub struct LogToggles {
    pub encoder: bool,
    pub push_events: bool,
}

struct Context {
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, LogToggles, 4>,
    logging_config: LogToggles,
    interface_open: bool
}

fn set_log(toggles: &mut LogToggles, channel: &str, state: bool) -> Result<(), ()> {
    match channel {
        "encoder" => {
            toggles.encoder = state;
        }
        "push_events" => {
            toggles.push_events = state;
        }
        _ => {return Err(());}
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
    Exit
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

impl Logging<'_> {
    fn handle(self: &Self, cli: &mut embedded_cli::cli::CliHandle<'_, esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>, core::convert::Infallible>, context: &mut Context) -> Result<(), Infallible> {
        match self {
            Logging::LogList => {
                uwrite!(cli.writer(), "Available logging channels: {:?}", context.logging_config)?;
            }
            Logging::LogEnable { channel } => {
                if set_log(&mut context.logging_config, channel, true).is_ok() {
                    uwrite!(cli.writer(), "Enabled logging for channel {}", channel)?;
                    context.sender.send(context.logging_config.clone());
                } else {
                    uwrite!(cli.writer(), "Invalid channel {}", channel)?;
                }
            }
            Logging::LogDisable{ channel } => {
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
}

impl Flash<'_> {
    fn handle(self: &Self, cli: &mut embedded_cli::cli::CliHandle<'_, esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, Async>, core::convert::Infallible>, context: &mut Context) -> Result<(), Infallible> {
        match self {
            Flash::FlashLoad => {
                uwrite!(cli.writer(), "Stored value is: ?")?;
            }
            Flash::FlashStore  { value } => {
                uwrite!(cli.writer(), "Saved value to flash: {}", value)?;
            }
        }
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn menu_handler(
    serial: UsbSerialJtag<'static, Async>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, LogToggles, 4>,
) {
    let mut context = Context { sender, logging_config: LogToggles::default(), interface_open: false };

    let command_buffer: [u8; 32] = [0; 32];
    let history_buffer: [u8; 32] = [0; 32];
    let (mut rx, tx) = serial.split();
    let builder = CliBuilder::default()
        .writer(tx)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build();
    let mut cli = if let Ok(cli) = builder {
        cli
    } else {
        return ;
    };

    let mut buf: [u8; 1] = [0; 1];
    loop {
        // read byte from somewhere (for example, uart)
        let _ = rx.read(&mut buf).await;

        // if the interface is not open open it and skip processing the CLI
        if context.interface_open == false {
            context.interface_open = true;
            continue;
        }

        let _ = cli.process_byte::<RootGroup, _>(
            buf[0],
            &mut RootGroup::processor(|cli, command| {
                match command {
                    RootGroup::Logging(logging) => {
                        let _ = logging.handle(cli, &mut context);
                    }
                    RootGroup::Storage(storage) => {
                        let _ = storage.handle(cli, &mut context);
                    }
                    RootGroup::Base(base) => match base {
                        Base::Exit => {
                            context.interface_open = false;
                        }
                    }
                }
                Ok(())
            }),
        );
    }
}
