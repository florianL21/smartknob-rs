use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Output},
    spi,
    time::Rate,
};
use fixed::types::I16F16;
use foc::pwm::SpaceVector;
use haptic_lib::{CurveBuilder, Easing, EasingType};
use log::info;
use smartknob_core::{
    haptic_core::{CalibrationData, DetailedSettings, SmartknobHapticCore, encoder::MT6701Spi},
    system_settings::{HapticSystemStoreSignal, log_toggles::LogToggleReceiver},
};
use smartknob_esp32::motor_driver::mcpwm::{MCPWM6, Pins6PWM};

pub type SpiBus1 = Mutex<NoopRawMutex, esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>>;

const ENCODER_SPI_DMA_BUFFER_SIZE: usize = 200;
const PWM_RESOLUTION: u16 = 1800;
const MOTOR_POLE_PAIRS: u8 = 7;

// about 2% dead time
const PWM_DEAD_TIME: u16 = 20;

#[embassy_executor::task]
pub async fn update_foc(
    spi_bus: spi::master::SpiDma<'static, esp_hal::Blocking>,
    mag_csn: Output<'static>,
    mcpwm0: esp_hal::peripherals::MCPWM0<'static>,
    pwm_pins: Pins6PWM<'static, AnyPin<'static>>,
    restored_state: Option<CalibrationData>,
    settings_store_signals: &'static HapticSystemStoreSignal<CriticalSectionRawMutex>,
    log_receiver: LogToggleReceiver,
) {
    // setup encoder SPI communication
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_buffers!(ENCODER_SPI_DMA_BUFFER_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi_bus = spi_bus.with_buffers(dma_rx_buf, dma_tx_buf).into_async();
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi_bus);
    let spi_device = SpiDevice::new(&spi_bus, mag_csn);
    let encoder = MT6701Spi::new(spi_device);
    info!("encoder init done!");

    let motor_driver: MCPWM6<_, PWM_RESOLUTION> = MCPWM6::new(
        mcpwm0,
        Rate::from_mhz(40),
        Rate::from_khz(20),
        pwm_pins,
        PWM_DEAD_TIME,
    )
    .unwrap();

    let mut haptic_core: SmartknobHapticCore<'_, _, _, SpaceVector, _, 52> =
        SmartknobHapticCore::new(
            encoder,
            motor_driver,
            MOTOR_POLE_PAIRS,
            None,
            DetailedSettings::default(),
            restored_state,
            log_receiver,
        )
        .await;

    // Create a curve with 25 identical detents for encoder positions 0-10
    // Each detent = 0.4 encoder units (10 / 25 = 0.4)
    // Gauge value = encoder * 10, so each detent = 4 on the gauge
    let detent_curve = CurveBuilder::<52>::new()
        // 25 detents, each with 2 segments (0.2 + 0.2 = 0.4 width)
        // All identical: 0.7 torque, cubic easing
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        // 5 detents done
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        // 10 detents done
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        // 15 detents done
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        // 20 detents done
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        .add_eased(0.2, 0.0, -0.7, Easing::Cubic(EasingType::In))
        .add_eased(0.2, 0.7, 0.0, Easing::Cubic(EasingType::Out))
        // 25 detents done - extend with no resistance
        .add_const(10.0, 0.0)
        .build()
        .unwrap()
        .make_absolute(I16F16::ZERO);
    let _ = haptic_core.set_curve(&detent_curve, 0.7).await;
    loop {
        haptic_core.run(settings_store_signals).await;
    }
}
