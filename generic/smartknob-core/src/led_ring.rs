use crate::{
    knob_tilt::{KNOB_TILT_ANGLE, KnobTiltEvent},
    system_settings::log_toggles::{LogChannel, LogToggleReceiver, may_log},
};
use embassy_sync::pubsub::DynSubscriber;
use embassy_time::{Duration, Ticker};
use log::info;
use smart_leds::{
    RGB, SmartLedsWrite, brightness,
    colors::{BLACK, BLUE, RED},
    gamma,
};

fn map<
    T: core::ops::Sub<Output = T>
        + core::ops::Mul<Output = T>
        + core::ops::Div<Output = T>
        + core::ops::Add<Output = T>
        + Copy,
>(
    x: T,
    in_min: T,
    in_max: T,
    out_min: T,
    out_max: T,
) -> T {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

pub async fn led_ring<const NUM_LEDS: usize, const LED_OFFSET: usize, LED>(
    mut log_receiver: LogToggleReceiver,
    min_refresh: Duration,
    mut leds: LED,
    mut tilt_receiver: DynSubscriber<'static, KnobTiltEvent>,
) -> !
where
    LED: SmartLedsWrite,
    <LED as SmartLedsWrite>::Color: From<RGB<u8>>,
{
    let mut data = [RED; NUM_LEDS];
    let _step_size = (2.0f32 * core::f32::consts::PI) / NUM_LEDS as f32;

    let mut ticker = Ticker::every(min_refresh);
    loop {
        // Since this is only interested in displaying the current state we can ignore lag information
        let tilt_event = tilt_receiver.next_message_pure().await;
        may_log(&mut log_receiver, LogChannel::Push, || {
            info!("Event: {:#?}", &tilt_event)
        })
        .await;
        match tilt_event {
            KnobTiltEvent::PressEnd | KnobTiltEvent::TiltEnd => {
                for item in data.iter_mut() {
                    *item = BLACK;
                }
            }
            KnobTiltEvent::PressStart => {
                for item in data.iter_mut() {
                    *item = RED;
                }
            }
            KnobTiltEvent::TiltStart(_) | KnobTiltEvent::TiltAdjust => {
                let angle = KNOB_TILT_ANGLE.load(core::sync::atomic::Ordering::Relaxed);
                let angle = if angle < 0.0 {
                    angle + 2.0 * core::f32::consts::PI
                } else {
                    angle
                };
                let led_index = map(
                    angle,
                    2.0 * core::f32::consts::PI,
                    0.0,
                    0.0,
                    NUM_LEDS as f32,
                );
                // not yet working
                for (i, item) in data.iter_mut().enumerate() {
                    if i == led_index as usize {
                        *item = BLUE;
                    } else {
                        *item = BLACK;
                    }
                }
            }
        }
        let _ = leds.write(brightness(gamma(data.iter().cloned()), 10));
        ticker.next().await;
    }
}
