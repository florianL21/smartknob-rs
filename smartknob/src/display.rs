use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_executor::Spawner;
use embassy_sync::signal::Signal;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::DriveMode;
use esp_hal::{dma_buffers, spi};
use esp_hal::{gpio::Output, time::Rate};
use log::{info, warn};
use slint::platform::software_renderer::{MinimalSoftwareWindow, Rgb565Pixel};
use slint::platform::WindowEvent;
use slint::platform::{Platform, PointerEventButton};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
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

const TARGET_FPS: u64 = 60;

const DISPLAY_SIZE: (u16, u16) = GC9A01::FRAMEBUFFER_SIZE;
const FRAME_BUFFER_SIZE: usize = (DISPLAY_SIZE.0 * DISPLAY_SIZE.1) as usize;
const DISPLAY_BUFFER_SIZE: usize = FRAME_BUFFER_SIZE / 2;
const TARGET_FRAME_DURATION: Duration = Duration::from_millis(1000 / TARGET_FPS);

pub type DisplaySpiBus = spi::master::SpiDma<'static, esp_hal::Blocking>;
type FBType = [Rgb565Pixel; FRAME_BUFFER_SIZE];
const DISPLAY_SPI_DMA_BUFFER_SIZE: usize = DISPLAY_BUFFER_SIZE;
const SPI_TRANSFER_BUFFER_SIZE: usize = DISPLAY_BUFFER_SIZE;

pub static SLINT_READY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub struct DisplayHandles {
    pub spi_bus: DisplaySpiBus,
    pub lcd_cs: Output<'static>,
    pub dc_output: Output<'static>,
    pub backlight_output: Output<'static>,
    pub reset_output: Output<'static>,
    pub ledc: Ledc<'static>,
}

pub fn spawn_display_tasks(spawner: Spawner, display_handles: DisplayHandles) {
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();
    let (fb0, fb1) = init_fbs_heap();

    spawner.must_spawn(display_task(display_handles, &RX, &TX, fb1));
    spawner.must_spawn(render_task(&TX, &RX, fb0));
    spawner.must_spawn(ui_task());
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
    (fb0, fb1)
}

/// Get a buffer for holding one full spi transaction which is allocated on the heap because it is too big for the stack
fn get_spi_buffer() -> &'static mut [u8; SPI_TRANSFER_BUFFER_SIZE] {
    use alloc::alloc::alloc;
    use alloc::boxed::Box;
    use core::alloc::Layout;

    let layout = Layout::new::<[u8; SPI_TRANSFER_BUFFER_SIZE]>();

    let buf = unsafe {
        let ptr = alloc(layout) as *mut [u8; SPI_TRANSFER_BUFFER_SIZE];
        Box::from_raw(ptr)
    };

    Box::leak(buf)
}

#[embassy_executor::task]
pub async fn display_task(
    display_handles: DisplayHandles,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
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
    let buffer = get_spi_buffer();
    let di = SpiInterface::new(device, display_handles.dc_output, buffer);
    let mut display = Builder::new(GC9A01, di)
        .reset_pin(display_handles.reset_output)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .await
        .unwrap();

    // backlight control
    let mut ledc = display_handles.ledc;
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty10Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(
        ledc::channel::Number::Channel0,
        display_handles.backlight_output,
    );
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

    let mut fb = fb;
    loop {
        let t = Instant::now();

        // wait for a new fb to transfer, once there is one send the old FB back and continue with transferring the just received one
        let new_fb = rx.wait().await;
        tx.signal(fb);
        fb = new_fb;
        // Send the frame to the display
        display
            .set_pixels(
                0,
                0,
                DISPLAY_SIZE.0 - 1,
                DISPLAY_SIZE.1 - 1,
                fb.iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .await
            .ok();
        info!("Frame took {} ms", t.elapsed().as_millis(),);
    }
}

#[embassy_executor::task]
pub async fn render_task(
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut FBType,
) {
    // UI setup
    let window = MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::SwappedBuffers,
    );
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        start_instant: Instant::now(),
    }))
    .unwrap();

    window.as_ref().set_size(slint::PhysicalSize::new(
        DISPLAY_SIZE.0 as u32,
        DISPLAY_SIZE.1 as u32,
    ));

    SLINT_READY_SIGNAL.signal(());
    let mut ticker = Ticker::every(TARGET_FRAME_DURATION);
    loop {
        slint::platform::update_timers_and_animations();

        // process and possibly dispatch events here
        // WindowEvent::PointerPressed {
        //     position: pos,
        //     button: PointerEventButton::Left,
        // };

        // window.try_dispatch_event(event)?;

        let t = Instant::now();
        let is_dirty = window.draw_if_needed(|renderer| {
            renderer.render(fb, DISPLAY_SIZE.0 as usize);
        });
        if is_dirty {
            info!(
                "New frame available. Rendering took {} ms",
                t.elapsed().as_millis()
            );
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
