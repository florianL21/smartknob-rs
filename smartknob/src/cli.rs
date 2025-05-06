use esp_backtrace as _;
use esp_hal::{usb_serial_jtag::UsbSerialJtag, Async};
use menu::asynchronous::{Item, ItemType, Menu, Runner};
use noline::builder::EditorBuilder;

use log::info;

#[derive(Default)]
struct Context {
    _inner: u32,
}

fn enter_root(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    // writeln!(interface, "In enter_root").unwrap();
    info!("In enter_root");
}

fn exit_root(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    // writeln!(interface, "In exit_root").unwrap();
    info!("In exit_root");
}

fn select_bar(
    _menu: &Menu<UsbSerialJtag<'_, Async>, Context>,
    _item: &Item<UsbSerialJtag<'_, Async>, Context>,
    args: &[&str],
    interface: &mut UsbSerialJtag<'_, Async>,
    _context: &mut Context,
) {
    // writeln!(interface, "In select_bar. Args = {:?}", args).unwrap();
    info!("In select_bar. Args = {:?}", args);
}

const ROOT_MENU: Menu<UsbSerialJtag<'_, Async>, Context> = Menu {
    label: "root",
    items: &[&Item {
        item_type: ItemType::Callback {
            function: select_bar,
            parameters: &[],
        },
        command: "bar",
        help: Some("fandoggles a bar"),
    }],
    entry: Some(enter_root),
    exit: Some(exit_root),
};

#[embassy_executor::task]
pub async fn menu_handler(mut serial: UsbSerialJtag<'static, Async>) {
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
        // ignore any errors and assume that we are just not connected to serial
        if let Err(_) = r.input_line(&mut context).await {
            return;
        }
    }
}
