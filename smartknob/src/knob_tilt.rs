use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use ldc1x1x::AutoScanSequence;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use libm::sqrtf;
use log::info;

type I2cBus1 = Mutex<NoopRawMutex, esp_hal::i2c::master::I2c<'static, esp_hal::Async>>;
use nalgebra::{Rotation2, Vector2};

async fn take_mean_measurement(
    ldc: &mut ldc1x1x::Ldc<
        I2cDevice<'static, NoopRawMutex, esp_hal::i2c::master::I2c<'_, esp_hal::Async>>,
    >,
) -> Vec<u32, 3> {
    // const NUM_AVERAGES: usize = 10;
    // let mut measurements: Vec<_, 10> = Vec::new();
    // calculate average over 10 measurements
    // for _ in [0..NUM_AVERAGES] {
    //     let mut channels: Vec<u32, 3> = heapless::Vec::new();
    //     for ch in [ldc1x1x::Channel::Zero, ldc1x1x::Channel::One, ldc1x1x::Channel::Two] {
    //         let reading = ldc.read_data_24bit(ch).await.unwrap();
    //         channels.push(reading).unwrap();
    //     }
    //     measurements.push(channels).unwrap();
    // }
    let mut measurements: Vec<u32, 3> = heapless::Vec::new();
    for ch in [
        ldc1x1x::Channel::Zero,
        ldc1x1x::Channel::One,
        ldc1x1x::Channel::Two,
    ] {
        let reading = ldc.read_data_24bit(ch).await.unwrap();
        measurements.push(reading).unwrap();
    }
    measurements
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
    for ch in [
        ldc1x1x::Channel::Zero,
        ldc1x1x::Channel::One,
        ldc1x1x::Channel::Two,
    ] {
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
    let mut directions: Vec<Vector2<f32>, 3> = Vec::new();
    let x_120 = sqrtf(3.0) / 2.0;
    directions.push(Vector2::new(0.0f32, -1.0f32)).unwrap();
    directions.push(Vector2::new(-x_120, 0.5f32)).unwrap();
    directions.push(Vector2::new(x_120, 1.0f32)).unwrap();
    const ORIGIN: Vector2<f32> = Vector2::new(0.0f32, -1.0f32);

    loop {
        let mut coil_vecs: Vec<Vector2<f32>, 3> = Vec::new();
        let mut raw_coil_values: Vec<u32, 3> = Vec::new();
        for (i, ch) in [
            ldc1x1x::Channel::Zero,
            ldc1x1x::Channel::One,
            ldc1x1x::Channel::Two,
        ]
        .iter()
        .enumerate()
        {
            let reading = ldc.read_data_24bit(*ch).await.unwrap();
            raw_coil_values.push(reading).unwrap();
            coil_vecs.push(directions[i] * reading as f32).unwrap();
        }
        let dir = coil_vecs[0] + coil_vecs[1] + coil_vecs[2];
        // let angle = Rotation2::rotation_between(&ORIGIN, &dir);
        // Vector2::new(x, y)
        Timer::after(Duration::from_millis(20)).await;
    }
}
