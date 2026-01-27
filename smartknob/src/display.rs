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

    // Store scale, needle, and label as static pointers for the update loop
    static mut SCALE: *mut lv_bevy_ecs::sys::lv_obj_t = core::ptr::null_mut();
    static mut NEEDLE_LINE: *mut lv_bevy_ecs::sys::lv_obj_t = core::ptr::null_mut();
    static mut VALUE_LABEL: *mut lv_bevy_ecs::sys::lv_obj_t = core::ptr::null_mut();

    unsafe {
        let screen = lv_bevy_ecs::sys::lv_screen_active();

        // Create scale widget - 230° round gauge
        let scale = lv_bevy_ecs::sys::lv_scale_create(screen);
        lv_bevy_ecs::sys::lv_obj_set_size(scale, 200, 200);
        lv_bevy_ecs::sys::lv_obj_center(scale);
        
        // Configure scale: round inner mode (8), 230 degree arc
        lv_bevy_ecs::sys::lv_scale_set_mode(scale, 8);  // LV_SCALE_MODE_ROUND_INNER
        lv_bevy_ecs::sys::lv_scale_set_range(scale, 0, 100);
        lv_bevy_ecs::sys::lv_scale_set_total_tick_count(scale, 21);  // Ticks every 5 units
        lv_bevy_ecs::sys::lv_scale_set_major_tick_every(scale, 5);   // Major tick every 25 units
        lv_bevy_ecs::sys::lv_scale_set_angle_range(scale, 230);
        lv_bevy_ecs::sys::lv_scale_set_rotation(scale, 155);  // Start from bottom-left
        lv_bevy_ecs::sys::lv_scale_set_label_show(scale, false);  // No labels
        
        // Hide built-in ticks - we'll draw custom gradient-colored ticks
        lv_bevy_ecs::sys::lv_obj_set_style_length(scale, 0, 0x00050000);  // Hide minor ticks
        lv_bevy_ecs::sys::lv_obj_set_style_length(scale, 0, 0x00020000);  // Hide major ticks
        
        SCALE = scale;

        // Draw custom tick lines with gradient colors (blue to red)
        // 21 ticks: 0, 5, 10, ... 100
        let center_x: f32 = 120.0;
        let center_y: f32 = 120.0;
        let tick_outer_radius: f32 = 95.0;
        let tick_inner_radius: f32 = 80.0;  // 15px tick length
        
        // Static storage for tick line points
        static mut TICK_POINTS: [[lv_bevy_ecs::sys::lv_point_precise_t; 2]; 21] = 
            [[lv_bevy_ecs::sys::lv_point_precise_t { x: 0, y: 0 }; 2]; 21];
        
        for tick_idx in 0..=20 {
            let tick_value = tick_idx * 5;  // 0, 5, 10, ... 100
            let progress = tick_value as f32 / 100.0;
            
            // Calculate angle: gauge 0 = 155°, gauge 100 = 155° + 230° = 385°
            let angle_deg = 155.0 + progress * 230.0;
            let angle_rad = angle_deg * core::f32::consts::PI / 180.0;
            
            let cos_val = libm::cosf(angle_rad);
            let sin_val = libm::sinf(angle_rad);
            
            let outer_x = (center_x + tick_outer_radius * cos_val) as i32;
            let outer_y = (center_y + tick_outer_radius * sin_val) as i32;
            let inner_x = (center_x + tick_inner_radius * cos_val) as i32;
            let inner_y = (center_y + tick_inner_radius * sin_val) as i32;
            
            // Gradient color: blue (cold) to red (hot)
            let r = (100.0 + progress * 80.0) as u8;
            let g = (180.0 - progress * 150.0) as u8;
            let b = (255.0 - progress * 225.0) as u8;
            let tick_color = ((r as u32) << 16) | ((g as u32) << 8) | (b as u32);
            
            // Create tick line
            let line = lv_bevy_ecs::sys::lv_line_create(screen);
            
            TICK_POINTS[tick_idx][0] = lv_bevy_ecs::sys::lv_point_precise_t { x: outer_x, y: outer_y };
            TICK_POINTS[tick_idx][1] = lv_bevy_ecs::sys::lv_point_precise_t { x: inner_x, y: inner_y };
            
            lv_bevy_ecs::sys::lv_line_set_points(line, TICK_POINTS[tick_idx].as_ptr(), 2);
            lv_bevy_ecs::sys::lv_obj_set_style_line_color(line, lv_bevy_ecs::sys::lv_color_hex(tick_color), 0);
            lv_bevy_ecs::sys::lv_obj_set_style_line_width(line, 2, 0);
        }

        // Create needle line
        let needle = lv_bevy_ecs::sys::lv_line_create(scale);
        lv_bevy_ecs::sys::lv_obj_set_style_line_color(needle, lv_bevy_ecs::sys::lv_color_hex(0x64B4FF), 0);
        lv_bevy_ecs::sys::lv_obj_set_style_line_width(needle, 3, 0);
        lv_bevy_ecs::sys::lv_obj_set_style_line_rounded(needle, true, 0);
        
        // Set initial needle position
        lv_bevy_ecs::sys::lv_scale_set_line_needle_value(scale, needle, 60, 0);
        
        NEEDLE_LINE = needle;

        // Create center circle to cover inner 20px of needle
        // This makes the needle appear to start 20px from center
        let center_circle = lv_bevy_ecs::sys::lv_obj_create(screen);
        lv_bevy_ecs::sys::lv_obj_set_size(center_circle, 40, 40);  // 20px radius = 40px diameter
        lv_bevy_ecs::sys::lv_obj_center(center_circle);
        lv_bevy_ecs::sys::lv_obj_set_style_radius(center_circle, 20, 0);  // Make it circular
        lv_bevy_ecs::sys::lv_obj_set_style_bg_color(center_circle, lv_bevy_ecs::sys::lv_color_hex(0x0A0A2E), 0);
        lv_bevy_ecs::sys::lv_obj_set_style_bg_opa(center_circle, 255, 0);
        lv_bevy_ecs::sys::lv_obj_set_style_border_width(center_circle, 0, 0);

        // Create center value label (on top of circle)
        let label = lv_bevy_ecs::sys::lv_label_create(screen);
        lv_bevy_ecs::sys::lv_obj_center(label);
        lv_bevy_ecs::sys::lv_obj_set_style_text_color(label, lv_bevy_ecs::sys::lv_color_hex(0xFFFFFF), 0);
        lv_bevy_ecs::sys::lv_label_set_text(label, c"0".as_ptr());
        
        VALUE_LABEL = label;
    }

    // Encoder controls gauge - snap to 25 detent positions
    // Each detent = 0.4 encoder units, gauge shows 0, 4, 8, ... 100
    loop {
        let enc = get_encoder_position();
        let detent_float = enc / 0.4;
        let detent_index = ((detent_float + 0.5) as i32).clamp(0, 25);
        let gauge_value = (detent_index * 4).min(100);

        // Calculate gradient color: blue (cold) to red (hot)
        let progress = gauge_value as f32 / 100.0;
        let r = (100.0 + progress * 80.0) as u8;   // 100 -> 180
        let g = (180.0 - progress * 150.0) as u8;  // 180 -> 30
        let b = (255.0 - progress * 225.0) as u8;  // 255 -> 30
        let needle_color = ((r as u32) << 16) | ((g as u32) << 8) | (b as u32);

        unsafe {
            // Update needle position and color
            lv_bevy_ecs::sys::lv_scale_set_line_needle_value(SCALE, NEEDLE_LINE, 60, gauge_value);
            lv_bevy_ecs::sys::lv_obj_set_style_line_color(
                NEEDLE_LINE,
                lv_bevy_ecs::sys::lv_color_hex(needle_color),
                0,
            );
            
            let text = CString::new(alloc::format!("{}", gauge_value)).unwrap();
            lv_bevy_ecs::sys::lv_label_set_text(VALUE_LABEL, text.as_ptr());
        }
        
        Timer::after_millis(16).await;  // ~60fps
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
