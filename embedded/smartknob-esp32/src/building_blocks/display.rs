use super::DisplayBlock;

use crate::display::{
    BacklightHandles, BrightnessSensor, DisplayHandles, brightness_task,
    slint::{spawn_display_tasks, ui_task},
};
use embassy_sync::{blocking_mutex::raw::RawMutex, pubsub::DynSubscriber};
use esp_hal::{
    dma::DmaChannelFor,
    gpio::{Level, Output, OutputConfig, OutputPin, interconnect::PeripheralOutput},
    ledc::Ledc,
    peripherals::LEDC,
    spi::{self, Mode, master::Spi, slave::AnySpi},
    time::Rate,
};
use lcd_async::options::Orientation;
use smartknob_core::{knob_tilt::KnobTiltEvent, system_settings::log_toggles::LogToggleWatcher};

pub struct DisplayPins<
    SCK: PeripheralOutput<'static>,
    MOSI: PeripheralOutput<'static>,
    CS: OutputPin + 'static,
    DC: OutputPin + 'static,
    RST: OutputPin + 'static,
    BL: OutputPin + 'static,
> {
    pub lcd_sck: SCK,
    pub lcd_mosi: MOSI,
    pub lcd_cs: CS,
    pub lcd_dc: DC,
    pub lcd_rst: RST,
    pub lcd_bl: BL,
}

/// Pre-made building block for rendering UI to a display attached via SPi
/// together with backlight dimming via LEDC and a brightness sensor
pub struct SPIDisplay<
    SPI: spi::master::Instance + 'static,
    DMA: DmaChannelFor<AnySpi<'static>>,
    SCK: PeripheralOutput<'static>,
    MOSI: PeripheralOutput<'static>,
    CS: OutputPin + 'static,
    DC: OutputPin + 'static,
    RST: OutputPin + 'static,
    BL: OutputPin + 'static,
> {
    /// SPI instance to use for the communication with the display
    pub spi_peripheral: SPI,
    /// LEDC peripheral for controlling the display backlight
    pub ledc_peripheral: LEDC<'static>,
    /// DMA channel for display transfers
    pub dma_channel: DMA,
    /// GPIO pin assignments for the display
    pub display_pins: DisplayPins<SCK, MOSI, CS, DC, RST, BL>,
    /// Brightness sensor for automatic backlight dimming
    pub brightness_sensor: Option<&'static mut (dyn BrightnessSensor + 'static)>,
    /// Orientation of the display. depending on the mechanical design this may need to be changed
    pub display_orientation: Orientation,
}

impl<
    SPI: spi::master::Instance + 'static,
    DMA: DmaChannelFor<AnySpi<'static>>,
    SCK: PeripheralOutput<'static>,
    MOSI: PeripheralOutput<'static>,
    CS: OutputPin + 'static,
    DC: OutputPin + 'static,
    RST: OutputPin + 'static,
    BL: OutputPin + 'static,
> DisplayBlock for SPIDisplay<SPI, DMA, SCK, MOSI, CS, DC, RST, BL>
{
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: embassy_executor::Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        knob_tilt: DynSubscriber<'static, KnobTiltEvent>,
    ) {
        let spi_bus: spi::master::SpiDma<'_, esp_hal::Blocking> = Spi::new(
            self.spi_peripheral,
            spi::master::Config::default()
                .with_frequency(Rate::from_mhz(80))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(self.display_pins.lcd_sck)
        .with_mosi(self.display_pins.lcd_mosi)
        .with_dma(self.dma_channel);

        let display_handles = DisplayHandles {
            spi_bus,
            lcd_cs: Output::new(
                self.display_pins.lcd_cs,
                Level::Low,
                OutputConfig::default(),
            ),
            dc_output: Output::new(
                self.display_pins.lcd_dc,
                Level::Low,
                OutputConfig::default(),
            ),
            reset_output: Output::new(
                self.display_pins.lcd_rst,
                Level::Low,
                OutputConfig::default(),
            ),
            orientation: self.display_orientation,
        };

        let backlight_stuff = BacklightHandles {
            backlight_output: Output::new(
                self.display_pins.lcd_bl,
                Level::Low,
                OutputConfig::default(),
            ),
            brightness_sensor: self.brightness_sensor,
            ledc: Ledc::new(self.ledc_peripheral),
        };

        spawn_display_tasks(spawner, display_handles, log_watcher, knob_tilt).unwrap();
        // Spawn the taks which sets the actual UI state
        spawner.spawn(ui_task().unwrap());
        spawner.spawn(
            brightness_task(
                backlight_stuff,
                log_watcher
                    .dyn_receiver()
                    .expect("Could not get log toggle receiver for brightness task"),
            )
            .unwrap(),
        );
    }
}
