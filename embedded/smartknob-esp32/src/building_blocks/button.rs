use super::ButtonBlock;

use crate::knob_tilt::{I2cBusLDC, ldc_knob_tilt_task};
use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use esp_hal::{
    gpio::{
        Input, InputConfig, InputPin,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    i2c::{self, master::I2c},
    time::Rate,
};
use smartknob_core::system_settings::log_toggles::LogToggleWatcher;
use static_cell::StaticCell;

pub struct LDCPins<
    SCL: PeripheralInput<'static> + PeripheralOutput<'static>,
    SDA: PeripheralInput<'static> + PeripheralOutput<'static>,
    INT: InputPin + 'static,
> {
    pub i2c_scl: SCL,
    pub i2c_sda: SDA,
    pub ldc_int: INT,
}

/// Pre-made building block for systems which use an LDC sensor for
/// detecting things like presses on the knob or even tilting it
pub struct LDCButtons<
    I2C: i2c::master::Instance + 'static,
    SCL: PeripheralInput<'static> + PeripheralOutput<'static>,
    SDA: PeripheralInput<'static> + PeripheralOutput<'static>,
    INT: InputPin + 'static,
> {
    /// I2C instance to use for the communication with the LDC sensor
    pub i2c_peripheral: I2C,
    /// I2C bus frequency for communicating with the LDC chip. 300kHz is known to work reliably
    pub i2c_freq: Rate,
    /// GPIO pins connected the the LDC
    pub pins: LDCPins<SCL, SDA, INT>,
}

impl<
    I2C: i2c::master::Instance + 'static,
    SCL: PeripheralInput<'static> + PeripheralOutput<'static>,
    SDA: PeripheralInput<'static> + PeripheralOutput<'static>,
    INT: InputPin + 'static,
> ButtonBlock for LDCButtons<I2C, SCL, SDA, INT>
{
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: embassy_executor::Spawner,
        _: &'static LogToggleWatcher<M, N>,
        event_sender: embassy_sync::pubsub::DynPublisher<
            'static,
            smartknob_core::knob_tilt::KnobTiltEvent,
        >,
    ) {
        // LDC sensor
        let i2c_bus: I2c<'_, esp_hal::Async> = I2c::new(
            self.i2c_peripheral,
            i2c::master::Config::default().with_frequency(self.i2c_freq),
        )
        .unwrap()
        .with_scl(self.pins.i2c_scl)
        .with_sda(self.pins.i2c_sda)
        .into_async();
        static I2C_BUS: StaticCell<I2cBusLDC> = StaticCell::new();
        let i2c_bus = I2C_BUS.init(Mutex::new(i2c_bus));

        let ldc_int_pin = Input::new(self.pins.ldc_int, InputConfig::default());

        spawner.spawn(ldc_knob_tilt_task(i2c_bus, ldc_int_pin, event_sender).unwrap());
    }
}
