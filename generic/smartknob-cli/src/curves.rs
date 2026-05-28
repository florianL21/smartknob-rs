extern crate std;

use charming::{
    Chart,
    component::{
        Axis, DataZoom, DataZoomType, Feature, Restore, SaveAsImage, Title, Toolbox,
        ToolboxDataZoom,
    },
    element::{AxisType, Tooltip},
    series::Line,
};
use smartknob_core::haptics::base::{HapticCurveConfig, HapticPlayer, Playback};

pub fn create_graph(start: f32, curve: HapticCurveConfig, sample_step: f32) -> Chart {
    let curve_start_angle = curve.start_angle();
    let curve_inst = curve.without_pattern_layer().instantiate().unwrap();
    let width = curve_inst.width();
    let mut player = HapticPlayer::new(start, &curve_inst);
    let start_angle = start + curve_start_angle - 1.0;
    let end_anlge = start_angle + width + 2.0;

    let steps = ((end_anlge - start_angle) / sample_step) as u32;
    let mut x = start_angle;
    let mut x_data: Vec<f32> = Vec::new();
    let mut data: Vec<f32> = Vec::new();
    for _ in 0..steps {
        let playback = player.play(x);
        if let Playback::Torque(v) = playback {
            data.push(v);
            x_data.push(x);
        }
        x += sample_step;
    }

    Chart::new()
        .tooltip(Tooltip::new())
        .title(Title::new().text("Torque response"))
        .toolbox(
            Toolbox::new().feature(
                Feature::new()
                    .data_zoom(ToolboxDataZoom::new().y_axis_index("none"))
                    .restore(Restore::new())
                    .save_as_image(SaveAsImage::new()),
            ),
        )
        .x_axis(
            Axis::new()
                .name("Position (rad)")
                .type_(AxisType::Category)
                .data(x_data.iter().map(|&x| x.to_string()).collect()),
        )
        .y_axis(Axis::new().name("Torque").type_(AxisType::Value))
        .data_zoom(DataZoom::new().type_(DataZoomType::Inside))
        .data_zoom(DataZoom::new())
        .series(Line::new().data(data))
}
