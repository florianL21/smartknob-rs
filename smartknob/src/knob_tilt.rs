use core::usize;

use average::Mean;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Timer};
use esp_hal::{config, gpio::Input};
use ldc1x1x::{AutoScanSequence, Channel};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use libm::sqrtf;
use log::info;

type I2cBus1 = Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;
use nalgebra::{Rotation2, Vector2};

const ORIGIN: Vector2<f32> = Vector2::new(0.0f32, -1.0f32);
const CHANNELS: [Channel; 3] = [
    ldc1x1x::Channel::Zero,
    ldc1x1x::Channel::One,
    ldc1x1x::Channel::Two,
];

#[derive(Debug, Clone)]
pub struct TiltInfo {
    /// angle in radians from -PI to PI
    pub angle: f32,
    pub magnitude: u32,
}

#[derive(Debug, Clone)]
pub enum KnobTiltEvent {
    TiltStart(TiltInfo),
    TiltAdjust(TiltInfo),
    TiltEnd,
    PressStart,
    PressEnd,
}

enum KnobTiltState {
    Pressed,
    Idle,
    Tilted(TiltInfo),
}

struct KnobTiltConfig {
    tilt_trigger: u32,
    tilt_release: u32,
    press_trigger: u32,
    press_release: u32,
}

impl KnobTiltConfig {
    fn default() -> Self {
        KnobTiltConfig {
            tilt_trigger: 10000,
            tilt_release: 8000,
            press_trigger: 10000,
            press_release: 8000,
        }
    }
}

impl TiltInfo {
    fn new(angle: f32, magnitude: u32) -> Self {
        TiltInfo { angle, magnitude }
    }
}

struct KnobTilt {
    directions: [Vector2<f32>; 3],
    idle_positions: [u32; 3],
    coil_vecs: [Vector2<f32>; 3],
    raw_coil_values: [u32; 3],
    compensated_coil_values: [u32; 3],
    direction: Vector2<f32>,
    angle: f32,
    rotation: Rotation2<f32>,
    magnitude: f32,
    config: KnobTiltConfig,
    state: KnobTiltState,
}

impl KnobTilt {
    fn new(config: KnobTiltConfig) -> Self {
        let x_120 = sqrtf(3.0) / 2.0;

        KnobTilt {
            directions: [
                Vector2::new(0.0f32, -1.0f32),
                Vector2::new(-x_120, 0.5f32),
                Vector2::new(x_120, 1.0f32),
            ],
            idle_positions: [0; 3],
            coil_vecs: [Vector2::default(); 3],
            raw_coil_values: [0; 3],
            compensated_coil_values: [0; 3],
            angle: 0.0,
            direction: Vector2::default(),
            rotation: Rotation2::default(),
            magnitude: 0.0,
            config,
            state: KnobTiltState::Idle,
        }
    }

    fn calibrate(&mut self, idle_positions: [u32; 3]) {
        self.idle_positions = idle_positions;
    }

    fn update(&mut self, raw_coil_values: [u32; 3]) -> Option<KnobTiltEvent> {
        self.raw_coil_values = raw_coil_values;
        for (i, reading) in self.raw_coil_values.iter().enumerate() {
            // compensate idle positions
            let reading = reading.saturating_sub(self.idle_positions[i]);
            self.compensated_coil_values[i] = reading;
            self.coil_vecs[i] = self.directions[i] * (reading as f32);
        }
        self.direction = self.coil_vecs.iter().sum();
        self.rotation = Rotation2::rotation_between(&ORIGIN, &self.direction);
        self.angle = self.rotation.angle();
        self.magnitude = self.direction.magnitude();

        match &self.state {
            KnobTiltState::Idle => {
                if (self.magnitude as u32) > self.config.tilt_trigger {
                    self.state =
                        KnobTiltState::Tilted(TiltInfo::new(self.angle, self.magnitude as u32));
                    Some(KnobTiltEvent::TiltStart(TiltInfo::new(
                        self.angle,
                        self.magnitude as u32,
                    )))
                } else if self.compensated_coil_values[0] > self.config.press_trigger {
                    self.state = KnobTiltState::Pressed;
                    Some(KnobTiltEvent::PressStart)
                } else {
                    None
                }
            }
            KnobTiltState::Pressed => {
                if self.compensated_coil_values[0] < self.config.press_release {
                    self.state = KnobTiltState::Idle;
                    Some(KnobTiltEvent::PressEnd)
                } else {
                    None
                }
            }
            KnobTiltState::Tilted(tilt) => {
                if (self.magnitude as u32) < self.config.tilt_release {
                    self.state = KnobTiltState::Idle;
                    Some(KnobTiltEvent::TiltEnd)
                } else if tilt.angle != self.angle || tilt.magnitude != (self.magnitude as u32) {
                    self.state =
                        KnobTiltState::Tilted(TiltInfo::new(self.angle, self.magnitude as u32));
                    Some(KnobTiltEvent::TiltAdjust(TiltInfo::new(
                        self.angle,
                        self.magnitude as u32,
                    )))
                } else {
                    None
                }
            }
        }
    }
}

async fn take_mean_measurement(
    ldc: &mut ldc1x1x::Ldc<
        I2cDevice<'static, NoopRawMutex, esp_hal::i2c::master::I2c<'_, esp_hal::Async>>,
    >,
    int_pin: &mut Input<'static>,
) -> [u32; 3] {
    const NUM_AVERAGES: usize = 10;
    let mut measurements: [[u32; 3]; NUM_AVERAGES] = [[0; 3]; 10];
    // calculate average over 10 measurements
    for i in 0..NUM_AVERAGES {
        let mut channels: [u32; 3] = [0; 3];
        for (j, ch) in CHANNELS.iter().enumerate() {
            let reading = ldc.read_data_24bit(*ch).await.unwrap();
            channels[j] = reading;
        }
        measurements[i] = channels;
        // int_pin.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(20)).await;
    }

    let mut means: [u32; 3] = [0; 3];
    for (i, channel) in CHANNELS.iter().enumerate() {
        means[i] = measurements
            .iter()
            .map(|v| f64::from(v[*channel as usize]))
            .collect::<Mean>()
            .mean() as u32;
    }
    means
}

#[embassy_executor::task]
pub async fn read_ldc_task(
    i2c: &'static I2cBus1,
    mut int_pin: Input<'static>,
    sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, KnobTiltEvent, 2>,
) {
    let i2c_device = I2cDevice::new(i2c);
    let mut ldc = ldc1x1x::Ldc::new(i2c_device, 0x2A);
    let mut init_ok = false;
    for _ in 0..3 {
        if ldc.reset().await.is_err() {
            Timer::after_millis(100).await;
            info!("LDC init failed, retrying...");
        } else {
            init_ok = true;
            break;
        }
    }
    if !init_ok {
        panic!("LDC init failed!");
    }
    let div = ldc1x1x::Fsensor::from_inductance_capacitance(5.267, 330.0).to_clock_dividers(None);
    for ch in CHANNELS {
        ldc.set_clock_dividers(ch, div).await.unwrap();
        ldc.set_conv_settling_time(ch, 15).await.unwrap();
        ldc.set_ref_count_conv_interval(ch, 0x0546).await.unwrap();
        ldc.set_sensor_drive_current(ch, 0x19).await.unwrap();
    }
    ldc.set_mux_config(
        ldc1x1x::MuxConfig::default()
            .with_auto_scan(true)
            .with_deglitch_filter_bandwidth(ldc1x1x::Deglitch::TenMHz)
            .with_auto_scan_sequence(AutoScanSequence::ZeroOneTwo),
    )
    .await
    .unwrap();
    ldc.set_config(ldc1x1x::Config::default().with_interrupt_on_status_update(true))
        .await
        .unwrap();
    ldc.set_error_config(
        ldc1x1x::ErrorConfig::default().with_amplitude_high_error_to_data_register(true),
    )
    .await
    .unwrap();
    info!("LDC init done!");
    Timer::after(Duration::from_millis(20)).await;
    let idle_positions = take_mean_measurement(&mut ldc, &mut int_pin).await;
    info!("Average idle positions: {:#?}", idle_positions);

    let mut kt = KnobTilt::new(KnobTiltConfig::default());
    kt.calibrate(idle_positions);

    loop {
        let mut raw_coil_values: [u32; 3] = [0; 3];
        for (i, ch) in CHANNELS.iter().enumerate() {
            let reading = ldc.read_data_24bit(*ch).await.unwrap();
            raw_coil_values[i] = reading;
        }
        if let Some(t) = kt.update(raw_coil_values) {
            info!("Event: {:#?}", &t);
            sender.send(t);
        }
        // int_pin.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(100)).await;
    }
}
