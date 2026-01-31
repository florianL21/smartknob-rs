use super::{BacklightHandles, DisplayHandles, DisplayTaskError, brightness_task};
use crate::knob_tilt::KnobTiltEvent;
use crate::signals::{
    DISPLAY_BRIGHTNESS_SIGNAL, KNOB_EVENTS_CHANNEL, KNOB_TILT_ANGLE, KNOB_TILT_MAGNITUDE,
};
use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::pubsub::WaitResult;
use embassy_sync::signal::Signal;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{Point, Size};
use embedded_graphics::primitives::Rectangle;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use lcd_async::{
    Builder,
    interface::SpiInterface,
    models::GC9A01,
    models::Model,
    options::{ColorInversion, ColorOrder},
    raw_framebuf::RawFrameBuf,
};
use log::{error, info};
use slint::LogicalPosition;
use slint::platform::Platform;
use slint::platform::WindowEvent;
use slint::platform::software_renderer::{MinimalSoftwareWindow, Rgb565Pixel};
use smartknob_core::haptic_core::get_encoder_position;
use smartknob_core::system_settings::log_toggles::{
    LogChannel, LogToggleReceiver, LogToggleWatcher, may_log,
};

slint::include_modules!();

const TARGET_FPS: u64 = 60;
const INITIAL_DISPLAY_BRIGHTNESS: u8 = 100;

const DISPLAY_SIZE: (u16, u16) = GC9A01::FRAMEBUFFER_SIZE;
const FRAME_BUFFER_SIZE_U16: usize = (DISPLAY_SIZE.0 * DISPLAY_SIZE.1) as usize;
const FRAME_BUFFER_SIZE_U8: usize = FRAME_BUFFER_SIZE_U16 * 2;
const TARGET_FRAME_DURATION: Duration = Duration::from_millis(1000 / TARGET_FPS);

type FBType = [u8; FRAME_BUFFER_SIZE_U8];
type SlintFBType = [Rgb565Pixel; FRAME_BUFFER_SIZE_U16];
const DISPLAY_SPI_DMA_BUFFER_SIZE: usize = FRAME_BUFFER_SIZE_U8 / 4;

pub static SLINT_READY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub fn spawn_display_tasks<M: RawMutex, const N: usize>(
    spawner: Spawner,
    display_handles: DisplayHandles,
    backlight_handles: BacklightHandles,
    log_toggles: &'static LogToggleWatcher<M, N>,
) -> Result<(), DisplayTaskError> {
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();
    let (fb0, fb1) = init_fbs_heap();

    spawner.spawn(display_task(
        display_handles,
        &RX,
        &TX,
        fb1,
        log_toggles
            .dyn_receiver()
            .ok_or(DisplayTaskError::LogReceiverOutOfCapacity)?,
    ))?;
    spawner.spawn(render_task(
        &TX,
        &RX,
        fb0,
        log_toggles
            .dyn_receiver()
            .ok_or(DisplayTaskError::LogReceiverOutOfCapacity)?,
    ))?;
    spawner.spawn(ui_task())?;
    spawner.spawn(brightness_task(
        backlight_handles,
        log_toggles
            .dyn_receiver()
            .ok_or(DisplayTaskError::LogReceiverOutOfCapacity)?,
    ))?;
    Ok(())
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
        self.start_instant.elapsed().into()
    }
}

fn init_fbs_heap() -> (&'static mut FBType, &'static mut FBType) {
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
    let fb1 = unsafe {
        let ptr = alloc(layout) as *mut FBType;
        Box::from_raw(ptr)
    };

    let fb0 = Box::leak(fb0);
    let fb1 = Box::leak(fb1);
    fb0.fill(0);
    fb1.fill(0);
    (fb0, fb1)
}

fn init_slint_fb_heap() -> &'static mut SlintFBType {
    // If the framebuffer is too large to fit in ram, we can allocate it on the
    // heap in PSRAM instead.
    // Allocate the framebuffer to PSRAM without ever putting it on the stack first
    use alloc::alloc::alloc;
    use alloc::boxed::Box;
    use core::alloc::Layout;

    let layout = Layout::new::<SlintFBType>();

    let fb0 = unsafe {
        let ptr = alloc(layout) as *mut SlintFBType;
        Box::from_raw(ptr)
    };

    let fb0 = Box::leak(fb0);
    fb0.fill(Rgb565Pixel(0));
    fb0
}

#[embassy_executor::task]
pub async fn display_task(
    display_handles: DisplayHandles,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
    mut log_receiver: LogToggleReceiver,
) {
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_buffers!(DISPLAY_SPI_DMA_BUFFER_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let spi_bus = display_handles
        .spi_bus
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi_bus);
    let device = SpiDevice::new(&spi_bus, display_handles.lcd_cs);

    let mut delay = Delay {};
    let di = SpiInterface::new(device, display_handles.dc_output);
    let mut display = Builder::new(GC9A01, di)
        .reset_pin(display_handles.reset_output)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .orientation(display_handles.orientation)
        .init(&mut delay)
        .await
        .unwrap();

    DISPLAY_BRIGHTNESS_SIGNAL.signal(INITIAL_DISPLAY_BRIGHTNESS);

    let mut fb = fb;
    let mut ticker = Ticker::every(TARGET_FRAME_DURATION / 2);
    loop {
        let t = Instant::now();

        // Send the frame to the display
        display
            .show_raw_data(0, 0, DISPLAY_SIZE.0, DISPLAY_SIZE.1, fb)
            .await
            .ok();
        may_log(&mut log_receiver, LogChannel::DisplayTransfer, || {
            info!("Frame took {} ms", t.elapsed().as_millis());
        })
        .await;
        // wait for a new fb to transfer, once there is one send the old FB back and continue with transferring the just received one
        let new_fb = rx.wait().await;
        tx.signal(fb);
        fb = new_fb;
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn render_task(
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut FBType,
    mut log_receiver: LogToggleReceiver,
) {
    let mut knob_tilt = KNOB_EVENTS_CHANNEL.subscriber().expect(
        "Could not get knob event channel subscriber. Please increase number of maximal subs",
    );
    // UI setup
    let window = MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        start_instant: Instant::now(),
    }))
    .unwrap();
    let slint_buf = init_slint_fb_heap();

    window.as_ref().set_size(slint::PhysicalSize::new(
        DISPLAY_SIZE.0 as u32,
        DISPLAY_SIZE.1 as u32,
    ));
    let display_area = Rectangle::new(
        Point::zero(),
        Size::new(DISPLAY_SIZE.0 as u32, DISPLAY_SIZE.1 as u32),
    );

    SLINT_READY_SIGNAL.signal(());
    let mut ticker = Ticker::every(TARGET_FRAME_DURATION);
    let mut last_key = slint::platform::Key::Space;
    let mut last_encoder_position = get_encoder_position();
    let center = LogicalPosition::new(DISPLAY_SIZE.0 as f32 / 2.0, DISPLAY_SIZE.1 as f32 / 2.0);
    loop {
        slint::platform::update_timers_and_animations();

        // process and possibly dispatch events here
        match knob_tilt.try_next_message() {
            Some(WaitResult::Message(event)) => {
                let event = match event {
                    KnobTiltEvent::PressStart => Some(WindowEvent::KeyPressed {
                        text: slint::platform::Key::Return.into(),
                    }),
                    KnobTiltEvent::PressEnd => Some(WindowEvent::KeyReleased {
                        text: slint::platform::Key::Return.into(),
                    }),
                    KnobTiltEvent::TiltStart(dir) => Some(WindowEvent::KeyPressed {
                        text: match dir {
                            crate::knob_tilt::TiltDirection::Down => {
                                last_key = slint::platform::Key::DownArrow;
                                last_key
                            }
                            crate::knob_tilt::TiltDirection::Left => {
                                last_key = slint::platform::Key::LeftArrow;
                                last_key
                            }
                            crate::knob_tilt::TiltDirection::Right => {
                                last_key = slint::platform::Key::RightArrow;
                                last_key
                            }
                            crate::knob_tilt::TiltDirection::Up => {
                                last_key = slint::platform::Key::UpArrow;
                                last_key
                            }
                        }
                        .into(),
                    }),
                    KnobTiltEvent::TiltEnd => Some(WindowEvent::KeyReleased {
                        text: last_key.into(),
                    }),
                    _ => None,
                };
                if let Some(event) = event
                    && let Err(e) = window.try_dispatch_event(event)
                {
                    error!("Slint platform error: {e}");
                }
            }
            Some(WaitResult::Lagged(n)) => {
                error!("Lost {n} knob tilt/push events. UI state my be out of sync")
            }
            None => {}
        }
        let current_encoder_position = get_encoder_position();
        // Emulate the encoder position by sending scroll events to slint
        if last_encoder_position != current_encoder_position {
            let delta = current_encoder_position - last_encoder_position;
            last_encoder_position = current_encoder_position;
            let event = WindowEvent::PointerScrolled {
                position: center,
                delta_x: delta,
                delta_y: 0.0,
            };
            if let Err(e) = window.try_dispatch_event(event) {
                error!("Slint platform error: {e}");
            }
        }

        let t = Instant::now();
        let is_dirty = window.draw_if_needed(|renderer| {
            let area = renderer.render(slint_buf, DISPLAY_SIZE.0 as usize);
            let mut fbuf = RawFrameBuf::<Rgb565, _>::new(
                &mut fb[..],
                DISPLAY_SIZE.0 as usize,
                DISPLAY_SIZE.1 as usize,
            );
            let origin = area.bounding_box_origin();
            let size = area.bounding_box_size();
            let top_left = Point::new(origin.x, origin.y);
            let size = Size::new(size.width, size.height);
            let _area = Rectangle::new(top_left, size);

            // TODO: Look into only rendering the part of the framebuffer which has been updated
            fbuf.fill_contiguous(
                &display_area,
                slint_buf
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .expect("Cannot fill display");
        });
        if is_dirty {
            may_log(&mut log_receiver, LogChannel::Render, || {
                info!(
                    "New frame available. Rendering took {} ms",
                    t.elapsed().as_millis()
                )
            })
            .await;

            // send the frame buffer to be rendered
            tx.signal(fb);
            // get the next frame buffer
            fb = rx.wait().await;
        } else {
            ticker.next().await;
        }
    }
}

#[embassy_executor::task]
pub async fn ui_task() {
    SLINT_READY_SIGNAL.wait().await;
    let ui = MainWindow::new().unwrap();
    ui.show().expect("unable to show main window");
    loop {
        // info!("Switiching toggle!");
        let tilt_angle = KNOB_TILT_ANGLE.load(core::sync::atomic::Ordering::Relaxed);
        let magnitude = KNOB_TILT_MAGNITUDE.load(core::sync::atomic::Ordering::Relaxed);
        ui.global::<State>()
            .set_tilt_angle(tilt_angle * 180.0 / core::f32::consts::PI);
        ui.global::<State>().set_tilt_magnitude(magnitude);
        Timer::after_millis(30).await;
    }
}
