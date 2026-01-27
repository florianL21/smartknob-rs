use alloc::ffi::CString;
use alloc::string::ToString;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::signal::Signal;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::{dma_buffers, spi};
use esp_hal::{gpio::Output, time::Rate};
use lcd_async::Builder;
use lcd_async::interface::SpiInterface;
use lcd_async::models::GC9A01;
use lcd_async::options::{ColorInversion, ColorOrder};
use lcd_async::raw_framebuf::RawFrameBuf;
use log::{error, info, warn};
use lv_bevy_ecs::support::LabelLongMode;
use lv_bevy_ecs::{
    display::{Display, DrawBuffer},
    events::Event,
    functions::*,
    support::Align,
    widgets::{Arc, Label, LvglWorld},
};
use smartknob_core::haptic_core::get_encoder_position;
use smartknob_core::system_settings::log_toggles::{
    LogChannel, LogToggleReceiver, LogToggleWatcher, may_log,
};
use thiserror::Error;

const DISPLAY_RENDER_DELAY: Duration = Duration::from_millis(10);
const INITIAL_DISPLAY_BRIGHTNESS: u8 = 100;
const BRIGHTNESS_FADE_DURATION_MS: u16 = 1000;

const WIDTH: usize = 240;
const HEIGHT: usize = 240;
const LINE_HEIGHT: usize = HEIGHT / 20;
// Rgb565 uses 2 bytes per pixel
const FRAME_BUFFER_SIZE: usize = WIDTH * HEIGHT * 2;

pub type DisplaySpiBus = spi::master::SpiDma<'static, esp_hal::Blocking>;
type FBType = [u8; FRAME_BUFFER_SIZE];
const DISPLAY_SPI_DMA_BUFFER_SIZE: usize = FRAME_BUFFER_SIZE / 4;

/// Set the display brightness with a percentage between 0 - 100%
pub static DISPLAY_BRIGHTNESS_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

static LVGL_READY_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static FRAMEBUFFER_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct DisplayHandles {
    pub spi_bus: DisplaySpiBus,
    pub lcd_cs: Output<'static>,
    pub dc_output: Output<'static>,

    pub reset_output: Output<'static>,
}

pub struct BacklightHandles {
    pub ledc: Ledc<'static>,
    pub adc_instance: esp_hal::peripherals::ADC1<'static>,
    // pub brightness_sensor_pin: GPIO4<'static>,
    pub backlight_output: Output<'static>,
}

#[derive(Error, Debug)]
pub enum DisplayTaskError {
    #[error("Log receiver has no more capacity. Increase the max number of log receivers")]
    LogReceiverOutOfCapacity,
    #[error("Failed to spawn at least one required task: {0}")]
    FailedToSpawnTask(#[from] SpawnError),
}

pub fn spawn_display_tasks<M: RawMutex, const N: usize>(
    spawner: Spawner,
    display_handles: DisplayHandles,
    backlight_handles: BacklightHandles,
    log_toggles: &'static LogToggleWatcher<M, N>,
) -> Result<(), DisplayTaskError> {
    let fb0 = init_fbs_heap();

    let stats = esp_alloc::HEAP.stats();
    info!("Current Heap stats: {}", stats);

    spawner.spawn(display_task(
        display_handles,
        fb0,
        log_toggles
            .dyn_receiver()
            .ok_or(DisplayTaskError::LogReceiverOutOfCapacity)?,
    ))?;
    spawner.spawn(render_task(
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

fn init_fbs_heap() -> &'static mut FBType {
    // If the framebuffer is too large to fit in ram, we can allocate it on the
    // heap in PSRAM instead.
    // Allocate the framebuffer to PSRAM without ever putting it on the stack first
    use alloc::alloc::alloc;
    use alloc::boxed::Box;
    use core::alloc::Layout;

    let layout = Layout::new::<FBType>();

    let fb = unsafe {
        let ptr = alloc(layout) as *mut FBType;
        Box::from_raw(ptr)
    };

    Box::leak(fb)
}

#[embassy_executor::task]
pub async fn display_task(
    display_handles: DisplayHandles,
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
    let mut tft_display = Builder::new(GC9A01, di)
        .reset_pin(display_handles.reset_output)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .await
        .unwrap();

    lv_init();

    lv_bevy_ecs::logging::connect();

    let mutex: blocking_mutex::Mutex<NoopRawMutex, &'static mut FBType> =
        blocking_mutex::Mutex::new(fb);

    let buffer =
        DrawBuffer::<{ WIDTH * LINE_HEIGHT }, Rgb565>::create(WIDTH as u32, LINE_HEIGHT as u32);
    let mut display = Display::create(WIDTH as i32, HEIGHT as i32);

    display.register(buffer, |refresh| {
        let area = refresh.rectangle;
        let data = refresh.colors.iter().cloned();
        unsafe {
            mutex.lock_mut(|fb| {
                let mut fbuf = RawFrameBuf::<Rgb565, _>::new(&mut fb[..], WIDTH, HEIGHT);

                // This next call causes the stack to overflow. Maybe because of the cloned iterator in line 103?
                fbuf.fill_contiguous(&area, data)
                    .expect("Cannot fill display");
                FRAMEBUFFER_READY.signal(());
            })
        }
    });

    LVGL_READY_SIGNAL.signal(());

    DISPLAY_BRIGHTNESS_SIGNAL.signal(INITIAL_DISPLAY_BRIGHTNESS);

    loop {
        FRAMEBUFFER_READY.wait().await;
        let t = Instant::now();
        let fb = mutex.borrow();
        // Send the frame to the display
        tft_display
            .show_raw_data(0, 0, WIDTH as u16, HEIGHT as u16, *fb)
            .await
            .ok();
        may_log(&mut log_receiver, LogChannel::DisplayTransfer, || {
            info!("Frame took {}ms to transfer", t.elapsed().as_millis());
        })
        .await;
    }
}

#[embassy_executor::task]
pub async fn render_task(mut log_receiver: LogToggleReceiver) {
    // UI setup
    let mut prev_time = Instant::now();

    let mut ticker = Ticker::every(DISPLAY_RENDER_DELAY);
    loop {
        let t = Instant::now();
        let current_time = Instant::now();
        let diff = current_time.duration_since(prev_time);
        lv_tick_inc(diff.into());
        lv_timer_handler();
        may_log(&mut log_receiver, LogChannel::Render, || {
            info!("Render time: {}ms", t.elapsed().as_millis())
        })
        .await;
        prev_time = current_time;
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn ui_task() {
    let mut world = LvglWorld::default();

    LVGL_READY_SIGNAL.wait().await;

    // Add gradient background (dark blue to dark purple)
    unsafe {
        let screen = lv_bevy_ecs::sys::lv_screen_active();
        lv_bevy_ecs::sys::lv_obj_set_style_bg_color(
            screen,
            lv_bevy_ecs::sys::lv_color_hex(0x0A0A2E),  // Dark blue
            0,
        );
        lv_bevy_ecs::sys::lv_obj_set_style_bg_grad_color(
            screen,
            lv_bevy_ecs::sys::lv_color_hex(0x1A0A2E),  // Dark purple
            0,
        );
        lv_bevy_ecs::sys::lv_obj_set_style_bg_grad_dir(screen, 1, 0);  // 1 = vertical
        lv_bevy_ecs::sys::lv_obj_set_style_bg_opa(screen, 255, 0);
    }

    let mut arc = Arc::create_widget();
    lv_obj_set_size(&mut arc, 150, 150);
    lv_arc_set_rotation(&mut arc, 135);
    lv_arc_set_bg_angles(&mut arc, 0, 270);
    lv_arc_set_value(&mut arc, 10);
    lv_obj_set_align(&mut arc, Align::Center.into());

    let mut label = Label::create_widget();
    lv_label_set_long_mode(&mut label, LabelLongMode::Clip.into());
    lv_label_set_text_static(&mut label, c"asdasdasd");
    lv_obj_set_align(&mut label, Align::TopMid.into());

    lv_obj_add_event_cb(&mut arc, Event::ValueChanged, |mut event| {
        let Some(mut obj) = lv_event_get_target_obj(&mut event) else {
            lv_bevy_ecs::warn!("Target obj was null");
            return;
        };
        let value = lv_arc_get_value(&mut obj);
        let text = CString::new(value.to_string()).unwrap();
        lv_label_set_text(&mut label, text.as_c_str());
    });

    world.spawn(label);
    world.spawn(arc);

    loop {
        let _enc = get_encoder_position();
        Timer::after_millis(30).await;
    }
}

#[embassy_executor::task]
async fn brightness_task(handles: BacklightHandles, mut _log_receiver: LogToggleReceiver) {
    let mut current_brightness = 0;

    // backlight control
    let mut ledc = handles.ledc;
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut lstimer0 = ledc.timer::<LowSpeed>(ledc::timer::Number::Timer0);
    lstimer0
        .configure(ledc::timer::config::Config {
            duty: ledc::timer::config::Duty::Duty10Bit,
            clock_source: ledc::timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(ledc::channel::Number::Channel0, handles.backlight_output);
    channel0
        .configure(ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: current_brightness,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    loop {
        if !channel0.is_duty_fade_running()
            && let Some(new_brighness) = DISPLAY_BRIGHTNESS_SIGNAL.try_take()
        {
            if let Err(e) = channel0.start_duty_fade(
                current_brightness,
                new_brighness,
                BRIGHTNESS_FADE_DURATION_MS,
            ) {
                warn!("Display backlight fade failed: {e:?}");
            } else {
                current_brightness = new_brighness;
            }
        }
        Timer::after_millis(200).await;
    }
}
