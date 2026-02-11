extern crate std;

use charming::{
    Chart, HtmlRenderer,
    component::{
        Axis, DataZoom, DataZoomType, Feature, Restore, SaveAsImage, Title, Toolbox,
        ToolboxDataZoom,
    },
    element::{AxisType, Tooltip},
    series::Line,
};
use haptic_lib::{CurveBuilder, CurveSegment, HapticCurve, HapticPlayer, Playback};

fn create_graph(start: f32, curve: HapticCurve, sample_step: f32) -> Chart {
    let curve_start_angle = curve.start_angle();
    let curve_inst = curve.instantiate().unwrap();
    let width = curve_inst.width();
    println!("{curve_inst:#?}");
    let mut player = HapticPlayer::new(start, &curve_inst);
    let start_angle = start + curve_start_angle - 1.0;
    let end_anlge = start_angle + width + 2.0;

    let steps = ((end_anlge - start_angle) / sample_step) as u32;
    let mut x = start_angle;
    let mut x_data: Vec<f32> = Vec::new();
    let mut data: Vec<f32> = Vec::new();
    for _ in 0..steps {
        let playback = player.play(x);
        match playback {
            Playback::Torque(v) => {
                data.push(v);
                x_data.push(x);
            }
            _ => {}
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

fn main() {
    env_logger::init();
    let mut curve_builder = CurveBuilder::new();

    let seg1 = curve_builder.new_segment(
        CurveSegment::new()
            .add_bezier3(0.5, [0.0, -0.1, -1.0])
            .add_bezier3(0.5, [1.0, 0.1, 0.0]),
    );
    let endstop_left = curve_builder.new_segment(CurveSegment::new().add_linear(0.8, 1.0, 0.0));
    let endstop_right = curve_builder.new_segment(CurveSegment::new().add_linear(0.8, 0.0, -1.0));
    let test_curve = curve_builder
        .push(endstop_left)
        .push_repeated(seg1, 3)
        .push(endstop_right)
        .finish(-0.3);

    let chart = create_graph(0.0, test_curve, 0.01);

    let mut renderer = HtmlRenderer::new("Force graph", 1000, 800);
    renderer.save(&chart, "chart.html").unwrap();
}
