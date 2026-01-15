extern crate std;

use charming::{
    component::{
        Axis, DataZoom, DataZoomType, Feature, Restore, SaveAsImage, Title, Toolbox,
        ToolboxDataZoom,
    },
    element::{AxisType, Tooltip},
    series::Line,
    Chart, HtmlRenderer,
};
use haptic_lib::{CurveBuilder, HapticCurve, HapticPlayer, Playback};

fn create_graph<const N: usize>(start: f32, curve: HapticCurve<N>, sample_step: f32) -> Chart {
    let curve_start_angle = curve.start_angle();
    let absolut_curve = curve.instantiate().unwrap();
    println!("{absolut_curve:#?}");
    let mut player = HapticPlayer::new(start, &absolut_curve);
    let width = player.curve_width();
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
    let test_curve = CurveBuilder::<6>::new()
        .add_bezier3(0.3, [1.0, 0.0, 0.0])
        .add_bezier3(0.5, [0.0, -0.1, -1.0])
        .add_bezier3(0.5, [1.0, 0.1, 0.0])
        .add_bezier3(0.3, [0.0, 0.0, -1.0])
        .build(-0.3)
        .unwrap();

    let chart = create_graph(0.0, test_curve, 0.01);

    let mut renderer = HtmlRenderer::new("Force graph", 1000, 800);
    renderer.save(&chart, "chart.html").unwrap();
}
