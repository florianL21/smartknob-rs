use core::cell::RefCell;

use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_sync::signal::Signal;
use esp_hal::gpio::DriveMode;
use esp_hal::time::Instant;
use esp_hal::{gpio::Output, time::Rate};
use log::{error, info, warn};
use slint::platform::software_renderer::{MinimalSoftwareWindow, Rgb565Pixel};
use slint::platform::WindowEvent;
use slint::platform::{Platform, PointerEventButton};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Ticker, Timer};
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, LSGlobalClkSource, Ledc, LowSpeed};
// use esp_hal::psram::{FlashFreq, PsramConfig, SpiRamFreq, SpiTimingConfigCoreClock};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use mipidsi::asynchronous::{
    interface::SpiInterface,
    models::{Model, GC9A01},
    options::{ColorInversion, ColorOrder},
    Builder,
};

slint::include_modules!();

pub type DisplaySpiBus =
    Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;
type FBType = [Rgb565Pixel; (GC9A01::FRAMEBUFFER_SIZE.0 * GC9A01::FRAMEBUFFER_SIZE.1) as usize];

const TARGET_FPS: u64 = 60;

pub static SLINT_READY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
    start_instant: Instant,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(self.start_instant.elapsed().as_millis())
    }

    // fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
    //     self.run_event_loop()
    // }
}

struct MyPlatform {
    window: Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    start_instant: Instant,
}

impl Platform for MyPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(self.start_instant.elapsed().as_millis())
    }
}

fn init_fbs_heap() -> &'static mut FBType {
    // If the framebuffer is too large to fit in ram, we can allocate it on the
    // heap in PSRAM instead.
    // Allocate the framebuffer to PSRAM without ever putting it on the stack first
    use alloc::alloc::alloc;
    use alloc::boxed::Box;
    use core::alloc::Layout;

    let layout = Layout::new::<FBType>();

    let fb0 = unsafe {
        let ptr = alloc(layout) as *mut FBType;
        Box::from_raw(ptr)
    };
    // let fb1 = unsafe {
    //     let ptr = alloc(layout) as *mut FBType;
    //     Box::from_raw(ptr)
    // };

    let fb0 = Box::leak(fb0);
    // let fb1 = Box::leak(fb1);
    fb0
}

#[embassy_executor::task]
pub async fn display_task(
    spi_bus: &'static DisplaySpiBus,
    lcd_cs: Output<'static>,
    dc_output: Output<'static>,
    backlight_output: Output<'static>,
    reset_output: Output<'static>,
    mut ledc: Ledc<'static>,
) {
    let device = SpiDevice::new(spi_bus, lcd_cs);
    const DISPLAY_SIZE: (u16, u16) = GC9A01::FRAMEBUFFER_SIZE;

    let mut delay = Delay {};
    let mut buffer = [0u8; DISPLAY_SIZE.0 as usize];
    let di = SpiInterface::new(device, dc_output, &mut buffer);
    let mut display = Builder::new(GC9A01, di)
        .reset_pin(reset_output)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .await
        .unwrap();

    // backlight control

    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty10Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(ledc::channel::Number::Channel0, backlight_output);
    channel0
        .configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    if let Err(e) = channel0.start_duty_fade(0, 50, 1000) {
        warn!("Display backlight fade failed: {e:?}");
    }
    // UI setup

    const TARGET_FRAME_DURATION: Duration = Duration::from_millis(1000 / TARGET_FPS);
    let mut ticker = Ticker::every(TARGET_FRAME_DURATION);

    let window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        start_instant: Instant::now(),
    }))
    .unwrap();

    window.as_ref().set_size(slint::PhysicalSize::new(
        DISPLAY_SIZE.0 as u32,
        DISPLAY_SIZE.1 as u32,
    ));

    let work_fb = init_fbs_heap();

    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);

    SLINT_READY_SIGNAL.signal(());

    loop {
        let t = Instant::now();

        slint::platform::update_timers_and_animations();

        // process and possibly dispatch events here
        // WindowEvent::PointerPressed {
        //         position: pos,
        //         button: PointerEventButton::Left,
        //     }
        // };
        // window.try_dispatch_event(event)?;

        let is_dirty = window.draw_if_needed(|renderer| {
            renderer.render(work_fb, DISPLAY_SIZE.0 as usize);
        });
        if is_dirty {
            display
                .set_pixels(
                    0,
                    0,
                    DISPLAY_SIZE.0 - 1,
                    DISPLAY_SIZE.1 - 1,
                    work_fb
                        .iter()
                        .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
                )
                .await
                .ok();
            info!("Frame took {} ms", t.elapsed().as_millis());
        } else {
            ticker.next().await;
        }
    }
}

#[embassy_executor::task]
pub async fn ui_task() {
    SLINT_READY_SIGNAL.wait().await;
    let ui = HelloWorld::new().unwrap();
    ui.show().expect("unable to show main window");
    let mut toggle = false;
    loop {
        info!("Switiching toggle!");
        ui.global::<State>().set_toggle(toggle);
        toggle = !toggle;
        Timer::after(Duration::from_secs(1)).await;
    }
}

impl Default for EspBackend {
    fn default() -> Self {
        EspBackend {
            window: RefCell::new(None),
            start_instant: Instant::now(),
        }
    }
}

impl EspBackend {
    // fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
    //         // Take and configure peripherals.
    //         let peripherals = self
    //             .peripherals
    //             .borrow_mut()
    //             .take()
    //             .expect("Peripherals already taken");
    //         let mut delay = Delay::new();

    //         // The following sequence is necessary to properly initialize touch on ESP32-S3-BOX-3
    //         // Based on issue from ESP-IDF: https://github.com/espressif/esp-bsp/issues/302#issuecomment-1971559689
    //         // Related code: https://github.com/espressif/esp-bsp/blob/30f0111a97b8fbe2efb7e58366fcf4d26b380f23/components/lcd_touch/esp_lcd_touch_gt911/esp_lcd_touch_gt911.c#L101-L133
    //         // --- Begin GT911 I²C Address Selection Sequence ---
    //         // Define constants for the two possible addresses.
    //         const ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS: u8 = 0x14;
    //         const ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP: u8 = 0x5D;

    //         // Our desired address.
    //         const DESIRED_ADDR: u8 = 0x14;
    //         // For desired address 0x14, assume the configuration’s reset level is false (i.e. 0 means active).
    //         let reset_level = Level::Low;

    //         // Configure the INT pin (GPIO3) as output; starting high because of internal pull-up.
    //         let mut int_pin = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());
    //         // Force INT low to prepare for address selection.
    //         int_pin.set_low();
    //         delay.delay_ms(10);
    //         int_pin.set_low();
    //         delay.delay_ms(1);

    //         // Configure the shared RESET pin (GPIO48) as output in open–drain mode.
    //         let mut rst = Output::new(
    //             peripherals.GPIO48,
    //             Level::Low, // start in active state
    //             OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    //         );

    //         // Set RESET to the reset-active level (here, false).
    //         rst.set_level(reset_level);
    //         // (Ensure INT remains low.)
    //         int_pin.set_low();
    //         delay.delay_ms(10);

    //         // Now, select the I²C address:
    //         // For GT911 address 0x14, the desired INT level is low; otherwise, for backup (0x5D) it would be high.
    //         let gpio_level = if DESIRED_ADDR == ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS {
    //             Level::Low
    //         } else if DESIRED_ADDR == ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP {
    //             Level::High
    //         } else {
    //             Level::Low
    //         };
    //         int_pin.set_level(gpio_level);
    //         delay.delay_ms(1);

    //         // Toggle the RESET pin:
    //         // Release RESET by setting it to the opposite of the reset level.
    //         rst.set_level(!reset_level);
    //         delay.delay_ms(10);
    //         delay.delay_ms(50);
    //         // --- End GT911 I²C Address Selection Sequence ---

    //         // --- Begin SPI and Display Initialization ---
    //         let spi = Spi::<esp_hal::Blocking>::new(
    //             peripherals.SPI2,
    //             SpiConfig::default()
    //                 .with_frequency(Rate::from_mhz(40))
    //                 .with_mode(SpiMode::_0),
    //         )
    //         .unwrap()
    //         .with_sck(peripherals.GPIO7)
    //         .with_mosi(peripherals.GPIO6);

    //         // Display control pins.
    //         let dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    //         let cs = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());

    //         // Wrap SPI into a bus.
    //         let spi_delay = Delay::new();
    //         let spi_device = ExclusiveDevice::new(spi, cs, spi_delay).unwrap();
    //         let mut buffer = [0u8; 512];
    //         let di = mipidsi::interface::SpiInterface::new(spi_device, dc, &mut buffer);

    //         // Initialize the display.
    //         let display = mipidsi::Builder::new(mipidsi::models::ILI9486Rgb565, di)
    //             .reset_pin(rst)
    //             .orientation(Orientation::new().rotate(Rotation::Deg180))
    //             .color_order(ColorOrder::Bgr)
    //             .init(&mut delay)
    //             .unwrap();

    //         // Set up the backlight pin.
    //         let mut backlight = Output::new(peripherals.GPIO47, Level::Low, OutputConfig::default());
    //         backlight.set_high();

    //         // Update the Slint window size from the display.
    //         let size = display.size();
    //         let size = slint::PhysicalSize::new(size.width, size.height);
    //         self.window.borrow().as_ref().unwrap().set_size(size);

    //         // --- End Display Initialization ---

    //         let mut i2c = I2c::new(
    //             peripherals.I2C0,
    //             esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    //         )
    //         .unwrap()
    //         .with_sda(peripherals.GPIO8)
    //         .with_scl(peripherals.GPIO18);

    //         // Initialize the touch driver.
    //         let mut touch = Gt911Blocking::new(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
    //         match touch.init(&mut i2c) {
    //             Ok(_) => info!("Touch initialized"),
    //             Err(e) => {
    //                 warn!("Touch initialization failed: {:?}", e);
    //                 let touch_fallback = Gt911Blocking::new(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
    //                 match touch_fallback.init(&mut i2c) {
    //                     Ok(_) => {
    //                         info!("Touch initialized with backup address");
    //                         touch = touch_fallback;
    //                     }
    //                     Err(e) => error!("Touch initialization failed with backup address: {:?}", e),
    //                 }
    //             }
    //         }

    //         // Prepare a draw buffer for the Slint software renderer.
    //         let mut buffer_provider = DrawBuffer {
    //             display,
    //             buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
    //         };

    //         // Variable to track the last touch position.
    //         let mut last_touch = None;

    //         // Main event loop.
    //         loop {
    //             slint::platform::update_timers_and_animations();

    //             if let Some(window) = self.window.borrow().clone() {
    //                 // Poll the GT911 for touch data.
    //                 match touch.get_touch(&mut i2c) {
    //                     // Active touch detected: Some(point) means a press or move.
    //                     Ok(Some(point)) => {
    //                         // Convert GT911 raw coordinates (assumed in pixels) into a PhysicalPosition.
    //                         let pos = slint::PhysicalPosition::new(point.x as i32, point.y as i32)
    //                             .to_logical(window.scale_factor());

    //                         let event = if let Some(previous_pos) = last_touch.replace(pos) {
    //                             // If the position changed, send a PointerMoved event.
    //                             if previous_pos != pos {
    //                                 WindowEvent::PointerMoved { position: pos }
    //                             } else {
    //                                 // If the position is unchanged, skip event generation.
    //                                 continue;
    //                             }
    //                         } else {
    //                             // No previous touch recorded, generate a PointerPressed event.
    //                             WindowEvent::PointerPressed {
    //                                 position: pos,
    //                                 button: PointerEventButton::Left,
    //                             }
    //                         };

    //                         // Dispatch the event to Slint.
    //                         window.try_dispatch_event(event)?;
    //                     }
    //                     // No active touch: if a previous touch existed, dispatch pointer release.
    //                     Ok(None) => {
    //                         if let Some(pos) = last_touch.take() {
    //                             window.try_dispatch_event(WindowEvent::PointerReleased {
    //                                 position: pos,
    //                                 button: PointerEventButton::Left,
    //                             })?;
    //                             window.try_dispatch_event(WindowEvent::PointerExited)?;
    //                         }
    //                     }
    //                     // On errors, you can log them if desired.
    //                     Err(_) => {
    //                         // Optionally log or ignore errors.
    //                     }
    //                 }

    //                 // Render the window if needed.
    //                 window.draw_if_needed(|renderer| {
    //                     renderer.render_by_line(&mut buffer_provider);
    //                 });

    //                 if window.has_active_animations() {
    //                     continue;
    //                 }
    //             }
    //         }
    // }
}

// /// Provides a draw buffer for the MinimalSoftwareWindow renderer.
// struct DrawBuffer<'a, Display> {
//     display: Display,
//     buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
// }

// impl<
//         DI: mipidsi::interface::Interface<Word = u8>,
//         RST: OutputPin<Error = core::convert::Infallible>,
//     > slint::platform::software_renderer::LineBufferProvider
//     for &mut DrawBuffer<'_, mipidsi::blockig::Display<DI, mipidsi::models::ILI9486Rgb565, RST>>
// {
//     type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

//     fn process_line(
//         &mut self,
//         line: usize,
//         range: core::ops::Range<usize>,
//         render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
//     ) {
//         let buffer = &mut self.buffer[range.clone()];
//         render_fn(buffer);

//         // Update the display with the rendered line.
//         self.display
//             .set_pixels(
//                 range.start as u16,
//                 line as u16,
//                 range.end as u16,
//                 line as u16,
//                 buffer
//                     .iter()
//                     .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
//             )
//             .unwrap();
//     }
// }
