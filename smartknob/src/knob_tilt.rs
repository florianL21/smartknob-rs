use average::Mean;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use heapless::Vec;
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

enum Event {
    TiltStart(f64),
    TiltAdjust(f64),
    TiltEnd,
    PressStart,
    PressEnd,
}

struct KnobTilt {
    directions: Vec<Vector2<f32>, 3>,
    idle_positions: Vec<u32, 3>,
    coil_vecs: Vec<Vector2<f32>, 3>,
    raw_coil_values: Vec<u32, 3>,
    direction: Vector2<f32>,
    angle: f32,
}

impl KnobTilt {
    fn new() -> Self {
        let mut directions = Vec::new();
        let idle_positions = Vec::new();
        let coil_vecs = Vec::new();
        let raw_coil_values = Vec::new();
        let x_120 = sqrtf(3.0) / 2.0;

        directions.push(Vector2::new(0.0f32, -1.0f32)).unwrap();
        directions.push(Vector2::new(-x_120, 0.5f32)).unwrap();
        directions.push(Vector2::new(x_120, 1.0f32)).unwrap();

        KnobTilt {
            directions,
            idle_positions,
            coil_vecs,
            raw_coil_values,
            angle: 0.0,
            direction: Vector2::default(),
        }
    }

    fn calibrate(&mut self, idle_positions: Vec<u32, 3>) {
        self.idle_positions = idle_positions;
    }

    fn update(&mut self, raw_coil_values: Vec<u32, 3>) {
        self.raw_coil_values = raw_coil_values;
        for (i, reading) in self.raw_coil_values.iter().enumerate() {
            self.coil_vecs
                .push(self.directions[i] * (*reading as f32))
                .unwrap();
        }
        let dir = self.coil_vecs[0] + self.coil_vecs[1] + self.coil_vecs[2];
        let angle = Rotation2::rotation_between(&ORIGIN, &dir);
        self.angle = angle.angle().to_degrees();
    }
}

async fn take_mean_measurement(
    ldc: &mut ldc1x1x::Ldc<
        I2cDevice<'static, NoopRawMutex, esp_hal::i2c::master::I2c<'_, esp_hal::Async>>,
    >,
) -> Vec<u32, 3> {
    const NUM_AVERAGES: usize = 10;
    let mut measurements: Vec<_, 10> = Vec::new();
    // calculate average over 10 measurements
    for _ in [0..NUM_AVERAGES] {
        let mut channels: Vec<u32, 3> = heapless::Vec::new();
        for ch in [
            ldc1x1x::Channel::Zero,
            ldc1x1x::Channel::One,
            ldc1x1x::Channel::Two,
        ] {
            let reading = ldc.read_data_24bit(ch).await.unwrap();
            channels.push(reading).unwrap();
        }
        measurements.push(channels).unwrap();
    }
    let mut means: Vec<u32, 3> = Vec::new();
    for channel in CHANNELS {
        let mean: Mean = measurements
            .iter()
            .map(|v| f64::from(v[channel as usize]))
            .collect();
        means.push(mean.mean() as u32).unwrap();
    }

    means
}

#[embassy_executor::task]
pub async fn read_ldc_task(i2c: &'static I2cBus1) {
    let i2c_device = I2cDevice::new(i2c);
    let mut ldc = ldc1x1x::Ldc::new(i2c_device, 0x2A);
    let mut init_ok = false;
    for _ in [0..3] {
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
    ldc.set_config(ldc1x1x::Config::default()).await.unwrap();
    ldc.set_error_config(
        ldc1x1x::ErrorConfig::default().with_amplitude_high_error_to_data_register(true),
    )
    .await
    .unwrap();
    info!("LDC init done!");
    let idle_positions = take_mean_measurement(&mut ldc).await;

    let mut kt = KnobTilt::new();
    kt.calibrate(idle_positions);

    loop {
        let mut raw_coil_values: Vec<u32, 3> = Vec::new();
        for ch in CHANNELS {
            let reading = ldc.read_data_24bit(ch).await.unwrap();
            raw_coil_values.push(reading).unwrap();
        }
        kt.update(raw_coil_values);
        info!("angle: {}", kt.angle);
        // let angle = Rotation2::rotation_between(&ORIGIN, &dir);
        // Vector2::new(x, y)
        Timer::after(Duration::from_millis(100)).await;
    }
}
