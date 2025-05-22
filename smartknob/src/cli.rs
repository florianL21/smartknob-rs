use core::fmt::Write;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_backtrace as _;
use esp_hal::{usb_serial_jtag::UsbSerialJtag, Async};
use menu::asynchronous::{Item, ItemType, Menu, Parameter, Runner};
use noline::builder::EditorBuilder;

#[derive(Clone, Debug, Default)]
pub struct LogToggles {
    pub encoder: bool,
    pub push_events: bool,
}

struct Context {
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, LogToggles, 4>,
    logging_config: LogToggles,
}

fn enter_root(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    let _ = writeln!(interface, "In enter_root");
}

fn exit_root(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    let _ = writeln!(interface, "In exit_root");
}

fn select_store(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    let _ = writeln!(interface, "In select_store. Args = {:?}", args);
}

fn select_load(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    let _ = writeln!(interface, "In select_load. Args = {:?}", args);
    // info!("In select_load. Args = {:?}", args);
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

fn log_enable(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    context: &mut Context,
) {
    if set_log(&mut context.logging_config, args[0], true).is_ok() {
        let _ = writeln!(interface, "Enabled logging for channel {}", args[0]);
        context.sender.send(context.logging_config.clone());
    } else {
        let _ = writeln!(interface, "Invalid channel {}", args[0]);
    }
}

fn log_disable(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    context: &mut Context,
) {
    if set_log(&mut context.logging_config, args[0], false).is_ok() {
        let _ = writeln!(interface, "Disabled logging for channel {}", args[0]);
        context.sender.send(context.logging_config.clone());
    } else {
        let _ = writeln!(interface, "Invalid channel {}", args[0]);
    }
}

fn log_list_channels(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    _args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    context: &mut Context,
) {
    let _ = writeln!(interface, "Avaliable logging channels: {:?}", context.logging_config);
}

const ROOT_MENU: Menu<UsbSerialJtag<'_, Async>, Context> = Menu {
    label: "root",
    items: &[
        &Item {
            item_type: ItemType::Callback {
                function: select_store,
                parameters: &[Parameter::Mandatory {
                    parameter_name: "value",
                    help: Some("The value to store"),
                }],
            },
            command: "save",
            help: Some("Store a value to flash"),
        },
        &Item {
            item_type: ItemType::Callback {
                function: select_load,
                parameters: &[],
            },
            command: "load",
            help: Some("Displays the value saved in flash"),
        },
        &Item {
            item_type: ItemType::Menu(&Menu {
                label: "logging",
                items: &[
                    &Item {
                        item_type: ItemType::Callback {
                            function: log_enable,
                            parameters: &[Parameter::Mandatory {
                                parameter_name: "channel",
                                help: Some("Which information to log"),
                            }],
                        },
                        command: "enable",
                        help: Some("Enable logging of a certain channel"),
                    },
                    &Item {
                        item_type: ItemType::Callback {
                            function: log_disable,
                            parameters: &[],
                        },
                        command: "disable",
                        help: Some("Disable logging of a certain channel"),
                    },
                    &Item {
                        item_type: ItemType::Callback {
                            function: log_list_channels,
                            parameters: &[],
                        },
                        command: "list",
                        help: Some("List all avaliable logging channels"),
                    },
                ],
                entry: None,
                exit: None,
            }),
            command: "logging",
            help: Some("Configure which information should be logged to the serial console"),
        },
    ],
    entry: Some(enter_root),
    exit: Some(exit_root),
};

#[embassy_executor::task]
pub async fn menu_handler(
    mut serial: UsbSerialJtag<'static, Async>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, LogToggles, 4>,
) {
    let mut buffer = [0u8; 100];
    let mut history = [0u8; 200];
    let Ok(mut editor) = EditorBuilder::from_slice(&mut buffer)
        .with_slice_history(&mut history)
        .build_async(&mut serial)
        .await
    else {
        return;
    };

    let mut context = Context { sender: sender, logging_config: LogToggles::default() };
    let mut r = Runner::new(ROOT_MENU, &mut editor, serial, &mut context).await;
    loop {
        // Upon error assume that we are just not connected to serial and terminate this function
        if let Err(_) = r.input_line(&mut context).await {
            return;
        }
    }
}
