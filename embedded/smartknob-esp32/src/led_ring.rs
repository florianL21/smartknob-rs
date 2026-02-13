use embassy_sync::pubsub::DynSubscriber;
use embassy_time::Duration;
use esp_hal::gpio::AnyPin;
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use smartknob_core::{
    knob_tilt::KnobTiltEvent, led_ring::led_ring, system_settings::log_toggles::LogToggleReceiver,
};

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
