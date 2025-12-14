extern crate std;

use charming::{
    Chart, HtmlRenderer,
    component::{
        Axis, DataZoom, DataZoomType, Feature, Legend, Restore, SaveAsImage, Title, Toolbox,
        ToolboxDataZoom,
    },
    element::{AxisLine, AxisType, ItemStyle, Tooltip},
    series::{Graph, Line},
};
use fixed::types::I16F16;
use haptic_lib::{AbsoluteCurve, CurveBuilder, HapticCurve, HapticPlayer, Easing, EasingType};

fn create_graph<const N: usize>(start: f32, curve: HapticCurve<N>, sample_step: f32) -> Chart {
    let absolut_curve = curve.make_absolute(I16F16::from_num(start));
    let mut player = HapticPlayer::new(I16F16::from_num(start), &absolut_curve);
    let width: f32 = player.curve_width().to_num();
    let start_angle = start - 1.0;
    let end_anlge = start_angle + width + 2.0;

    let steps = ((end_anlge - start_angle) / sample_step) as u32;
    let mut x = start_angle;
    let mut x_data: Vec<f32> = Vec::new();
    let mut data: Vec<f32> = Vec::new();
    for _ in 0..steps {
        data.push(player.play(I16F16::from_num(x)).to_num());
        x_data.push(x);
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
    let test_curve = CurveBuilder::<6>::new()
        .add_eased(0.3, 1.0, 0.0, Easing::Quadratic(EasingType::Out))
        .add_eased(0.5, 0.0, -1.0, Easing::Quadratic(EasingType::In))
        .add_eased(0.5, 1.0, 0.0, Easing::Quadratic(EasingType::Out))
        .add_eased(0.3, 0.0, -1.0, Easing::Quadratic(EasingType::In))
        .build()
        .unwrap();

    let chart = create_graph(0.0, test_curve, 0.01);

    let mut renderer = HtmlRenderer::new("Force graph", 1000, 800);
    renderer.save(&chart, "chart.html").unwrap();
}
