use super::LEDBlock;

use embassy_sync::{blocking_mutex::raw::RawMutex, pubsub::DynSubscriber};
use embassy_time::Duration;
use esp_hal::{gpio::AnyPin, peripherals, rmt::Rmt, time::Rate};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use smartknob_core::{
    knob_tilt::KnobTiltEvent,
    led_ring::led_ring,
    system_settings::log_toggles::{LogToggleReceiver, LogToggleWatcher},
};

/// Pre-made building block for an LED ring which consists of WS2812 LEDs arranged in a circle
/// It reacts soley on push events from the button subsystam
pub struct WS2812LedRingForPushEvents {
    /// RMT peripheral
    pub rmt_peripheral: peripherals::RMT<'static>,
    /// Frequency for the RMT peripheral. For example 80Mhz seems to work well
    pub rmt_frequency: Rate,
    /// GPIO pin where the LEDs are connected to
    pub pin_leds: AnyPin<'static>,
}

impl LEDBlock for WS2812LedRingForPushEvents {
    fn init<M: RawMutex, const N: usize>(
        self,
        spawner: embassy_executor::Spawner,
        log_watcher: &'static LogToggleWatcher<M, N>,
        tilt_receiver: DynSubscriber<'static, KnobTiltEvent>,
    ) {
        let rmt = Rmt::new(self.rmt_peripheral, self.rmt_frequency).unwrap();
        spawner.spawn(
            led_ring_task(
                rmt.channel0,
                self.pin_leds,
                log_watcher
                    .dyn_receiver()
                    .expect("Could not get log toggle receiver for led task"),
                tilt_receiver,
            )
            .unwrap(),
        );
    }
}

#[embassy_executor::task]
pub async fn led_ring_task(
    rmt_channel: esp_hal::rmt::ChannelCreator<'static, esp_hal::Blocking, 0>,
    led_pin: AnyPin<'static>,
    log_receiver: LogToggleReceiver,
    tilt_receiver: DynSubscriber<'static, KnobTiltEvent>,
) {
    const NUM_LEDS: usize = 24;
    const LED_OFFSET: usize = 1;

    let mut rmt_buffer = smart_led_buffer!(NUM_LEDS);
    let led = SmartLedsAdapter::new(rmt_channel, led_pin, &mut rmt_buffer);
    led_ring::<NUM_LEDS, LED_OFFSET, _>(
        log_receiver,
        Duration::from_millis(20),
        led,
        tilt_receiver,
    )
    .await;
}
