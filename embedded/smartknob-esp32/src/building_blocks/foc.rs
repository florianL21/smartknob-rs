use super::FOCBlock;

use crate::motor_driver::mcpwm::{MCPWM6, Pins6PWM};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex},
    mutex::Mutex,
};
use embassy_time::Duration;
use esp_hal::{
    Async,
    dma::{DmaChannelFor, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{
        AnyPin, Level, Output, OutputConfig, OutputPin,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    spi::{
        self, Mode,
        master::{Config as SpiConfig, Spi, SpiDmaBus},
    },
    time::Rate,
};
use foc::pwm::SpaceVector;
use log::info;
use smartknob_core::{
    haptics::{CalibrationData, DetailedSettings, encoder::MT6701Spi, run_haptic_core},
    system_settings::{
        HapticSystemStoreSignal,
        log_toggles::{LogToggleReceiver, LogToggleWatcher},
    },
};
use static_cell::StaticCell;

const ENCODER_SPI_DMA_BUFFER_SIZE: usize = 200;
const PWM_RESOLUTION: u16 = 1800;
const MOTOR_POLE_PAIRS: u8 = 7;
// about 2% dead time
const PWM_DEAD_TIME: u16 = 20;

pub struct EncoderPins<CLK: PeripheralOutput<'static>, DO: PeripheralInput<'static>, CS: OutputPin>
{
    /// GPIO pin the CLK of the MT6701 is connected to
    pub mag_clk: CLK,
    /// GPIO pin the DO of the MT6701 is connected to
    pub mag_do: DO,
    /// GPIO pin for the chip select of the MT6701
    pub mag_csn: CS,
}

/// Pre made building block for an FOC system using an MT6701 as an encoder and a 6PWM output
pub struct FOCMT6701With6PWM<
    SPI: spi::master::Instance,
    DMA: DmaChannelFor<spi::master::AnySpi<'static>>,
    CLK: PeripheralOutput<'static>,
    DO: PeripheralInput<'static>,
    CS: OutputPin,
> {
    /// SPI instance to use for the encoder
    pub spi: SPI,
    /// DMA channel to use for communication with the encoder
    pub dma_channel: DMA,
    /// MCPWM channel to use. Hardcoded to MWPC0 for now
    pub mcpwm_channel: esp_hal::peripherals::MCPWM0<'static>,
    /// Pins for the encoder
    pub encoder_pins: EncoderPins<CLK, DO, CS>,
    /// Pins for the motor driver
    pub pwm_pins: Pins6PWM<'static, AnyPin<'static>>,
    /// Frequency of the SPI bus of the encoder
    pub spi_frequency: Rate,
    /// Refresh rate of the FOC loop in case it should be throtteled to something
    /// If the FOC loop is running on its own core this can be None
    pub foc_refresh_rate: Option<Duration>,
    /// Settings for the haptic system. Exposed for fine tuning.
    /// Simply constructing a ::default() is good enough for most cases
    pub haptic_settings: DetailedSettings,
}

impl<
    SPI: spi::master::Instance + 'static,
    DMA: DmaChannelFor<spi::master::AnySpi<'static>>,
    CLK: PeripheralOutput<'static>,
    DO: PeripheralInput<'static>,
    CS: OutputPin + 'static,
> FOCBlock for FOCMT6701With6PWM<SPI, DMA, CLK, DO, CS>
{
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        restored_state: Option<CalibrationData>,
        settings_store_signals: &'static HapticSystemStoreSignal<CriticalSectionRawMutex>,
    ) {
        let encoder_pins = self.encoder_pins;
        let spi_bus = Spi::new(
            self.spi,
            SpiConfig::default()
                .with_frequency(self.spi_frequency)
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(encoder_pins.mag_clk)
        .with_miso(encoder_pins.mag_do)
        .with_dma(self.dma_channel);

        let mag_cs = Output::new(encoder_pins.mag_csn, Level::Low, OutputConfig::default());
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers!(ENCODER_SPI_DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let spi_bus = spi_bus.with_buffers(dma_rx_buf, dma_tx_buf).into_async();
        static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>> =
            StaticCell::new();
        let spi_bus = SPI_BUS.init(Mutex::new(spi_bus));
        let spi_device = SpiDevice::new(spi_bus, mag_cs);
        let encoder = MT6701Spi::new(spi_device);
        info!("encoder init done!");

        let motor_driver: MCPWM6<_, PWM_RESOLUTION> = MCPWM6::new(
            self.mcpwm_channel,
            Rate::from_mhz(40),
            Rate::from_khz(20),
            self.pwm_pins,
            PWM_DEAD_TIME,
        )
        .unwrap();

        spawner.spawn(
            foc_task(
                encoder,
                motor_driver,
                self.foc_refresh_rate,
                self.haptic_settings,
                restored_state,
                settings_store_signals,
                log_watcher
                    .dyn_receiver()
                    .expect("Could not get log_toggle receiver for shutdown subsystem"),
            )
            .unwrap(),
        );
    }
}

#[embassy_executor::task]
pub async fn foc_task(
    encoder: MT6701Spi<
        SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>,
    >,
    motor_driver: MCPWM6<'static, esp_hal::peripherals::MCPWM0<'static>, PWM_RESOLUTION>,
    refresh_rate: Option<Duration>,
    settings: DetailedSettings,
    restored_state: Option<CalibrationData>,
    settings_store_signals: &'static HapticSystemStoreSignal<CriticalSectionRawMutex>,
    log_receiver: LogToggleReceiver,
) {
    run_haptic_core::<_, _, SpaceVector, _, _>(
        encoder,
        motor_driver,
        MOTOR_POLE_PAIRS,
        refresh_rate,
        settings,
        restored_state,
        log_receiver,
        settings_store_signals,
    )
    .await;
}
