use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_executor::Spawner;
use embassy_sync::pubsub::WaitResult;
use embassy_sync::signal::Signal;
use esp_hal::analog::adc::{Adc, AdcCalBasic, AdcCalScheme, AdcConfig, Attenuation};
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::DriveMode;
use esp_hal::peripherals::GPIO4;
use esp_hal::{dma_buffers, spi};
use esp_hal::{gpio::Output, time::Rate};
use log::{error, info, warn};
use slint::platform::software_renderer::{MinimalSoftwareWindow, Rgb565Pixel};
use slint::platform::Platform;
use slint::platform::WindowEvent;
use slint::LogicalPosition;

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

use crate::config::{may_log, LogChannel, LOG_TOGGLES};
use crate::knob_tilt::KnobTiltEvent;
use crate::signals::{
    ENCODER_ANGLE, ENCODER_POSITION, KNOB_EVENTS_CHANNEL, KNOB_TILT_ANGLE, KNOB_TILT_MAGNITUDE,
};

slint::include_modules!();

const TARGET_FPS: u64 = 60;
const INITIAL_DISPLAY_BRIGHTNESS: u8 = 100;
const BRIGHTNESS_FADE_DURATION_MS: u16 = 1000;

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

/// Set the display brightness with a percentage between 0 - 100%
pub static DISPLAY_BRIGHTNESS_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

pub struct DisplayHandles {
    pub spi_bus: DisplaySpiBus,
    pub lcd_cs: Output<'static>,
    pub dc_output: Output<'static>,

    pub reset_output: Output<'static>,
}

pub struct BacklightHandles {
    pub ledc: Ledc<'static>,
    pub adc_instance: esp_hal::peripherals::ADC1<'static>,
    pub brightness_sensor_pin: GPIO4<'static>,
    pub backlight_output: Output<'static>,
}

pub fn spawn_display_tasks(
    spawner: Spawner,
    display_handles: DisplayHandles,
    backlight_handles: BacklightHandles,
) {
    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();
    let (fb0, fb1) = init_fbs_heap();

    spawner.must_spawn(display_task(display_handles, &RX, &TX, fb1));
    spawner.must_spawn(render_task(&TX, &RX, fb0));
    spawner.must_spawn(ui_task());
    spawner.must_spawn(brightness_task(backlight_handles));
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
    let mut log_receiver = LOG_TOGGLES
        .receiver()
        .expect("Could not create log receiver. Increase the receiver count");
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

    DISPLAY_BRIGHTNESS_SIGNAL.signal(INITIAL_DISPLAY_BRIGHTNESS);

    let mut fb = fb;
    let mut ticker = Ticker::every(TARGET_FRAME_DURATION / 2);
    loop {
        let t = Instant::now();

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
        may_log(&mut log_receiver, LogChannel::display_transfer, || {
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
) {
    let mut log_receiver = LOG_TOGGLES
        .receiver()
        .expect("Could not create log receiver. Increase the receiver count");
    let mut knob_tilt = KNOB_EVENTS_CHANNEL.subscriber().expect(
        "Could not get knob event channel subscriber. Please increase number of maximal subs",
    );
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
    let mut last_key = slint::platform::Key::Space;
    let mut last_encoder_position = ENCODER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
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
                if let Some(event) = event {
                    if let Err(e) = window.try_dispatch_event(event) {
                        error!("Slint platform error: {e}");
                    }
                }
            }
            Some(WaitResult::Lagged(n)) => {
                error!("Lost {n} knob tilt/push events. UI state my be out of sync")
            }
            None => {}
        }
        let current_encoder_position = ENCODER_POSITION.load(core::sync::atomic::Ordering::Relaxed);
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
            renderer.render(fb, DISPLAY_SIZE.0 as usize);
        });
        if is_dirty {
            may_log(&mut log_receiver, LogChannel::render, || {
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
        let angle = ENCODER_ANGLE.load(core::sync::atomic::Ordering::Relaxed);
        let tilt_angle = KNOB_TILT_ANGLE.load(core::sync::atomic::Ordering::Relaxed);
        let magnitude = KNOB_TILT_MAGNITUDE.load(core::sync::atomic::Ordering::Relaxed);
        ui.set_encoder_angle(angle);
        ui.global::<State>().set_encoder_angle(angle);
        ui.global::<State>()
            .set_tilt_angle(tilt_angle * 180.0 / core::f32::consts::PI);
        ui.global::<State>().set_tilt_magnitude(magnitude);
        Timer::after_millis(30).await;
    }
}

#[embassy_executor::task]
async fn brightness_task(handles: BacklightHandles) {
    let mut log_receiver = LOG_TOGGLES
        .receiver()
        .expect("Log toggle receiver had not enough capacity");

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

    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config
        .enable_pin_with_cal::<_, AdcCalBasic<_>>(handles.brightness_sensor_pin, Attenuation::_0dB);
    let mut adc1 = Adc::new(handles.adc_instance, adc1_config);
    loop {
        if !channel0.is_duty_fade_running() {
            if let Some(new_brighness) = DISPLAY_BRIGHTNESS_SIGNAL.try_take() {
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
        }

        if let Ok(val) = adc1.read_oneshot(&mut pin) {
            may_log(&mut log_receiver, LogChannel::brightness, || {
                info!("Brightness: {val}")
            })
            .await;
        }
        Timer::after_millis(200).await;
    }
}
