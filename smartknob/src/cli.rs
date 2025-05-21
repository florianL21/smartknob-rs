use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_backtrace as _;
use esp_hal::{usb_serial_jtag::UsbSerialJtag, Async};
use menu::asynchronous::{Item, ItemType, Menu, Parameter, Runner};
use noline::builder::EditorBuilder;
use core::fmt::Write;

#[derive(Clone)]
pub struct LogToggles {
    pub encoder: bool,
    pub push_events: bool,
}

#[derive(Default)]
struct Context {
    _inner: u32,
}

fn enter_root(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    writeln!(interface, "In enter_root").unwrap();
}

fn exit_root(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    writeln!(interface, "In exit_root").unwrap();
}

fn select_store(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    writeln!(interface, "In select_store. Args = {:?}", args).unwrap();
}

fn select_load(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    writeln!(interface, "In select_load. Args = {:?}", args).unwrap();
    // info!("In select_load. Args = {:?}", args);
}

const ROOT_MENU: Menu<UsbSerialJtag<'_, Async>, Context> = Menu {
    label: "root",
    items: &[&Item {
        item_type: ItemType::Callback {
            function: select_store,
            parameters: &[
                Parameter::Mandatory {
                        parameter_name: "value",
                        help: Some("The value to store"),
                    },
            ],
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
    }],
    entry: Some(enter_root),
    exit: Some(exit_root),
};

#[embassy_executor::task]
pub async fn menu_handler(mut serial: UsbSerialJtag<'static, Async>, mut receiver: embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, LogToggles, 4>,) {
    let mut buffer = [0u8; 100];
    let mut history = [0u8; 200];
    let Ok(mut editor) = EditorBuilder::from_slice(&mut buffer)
        .with_slice_history(&mut history)
        .build_async(&mut serial)
        .await
    else {
        return;
    };

    let mut context = Context::default();
    let mut r = Runner::new(ROOT_MENU, &mut editor, serial, &mut context).await;
    loop {
        // Upon error assume that we are just not connected to serial and terminate this function
        if let Err(_) = r.input_line(&mut context).await {
            return;
        }
    }
}
