extern crate std;

use charming::{
    component::{Axis, Legend},
    element::{AxisLine, AxisType, ItemStyle},
    series::{Graph, Line},
    Chart, HtmlRenderer,
};
use fixed::types::I16F16;
use haptic_lib::{AbsoluteCurve, CurveBuilder, HapticCurve, HapticPlayer};

fn create_graph<const N: usize>(
    start: f32,
    start_value: I16F16,
    end_value: I16F16,
    curve: HapticCurve<N>,
    sample_step: f32,
) -> Chart {
    let absolut_curve = curve.make_absolute(start_value, end_value, I16F16::from_num(start));
    let player = HapticPlayer::new(I16F16::from_num(start), &absolut_curve);
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
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .axis_line(AxisLine::new().on_zero(false))
                .data(x_data.iter().map(|&x| x.to_string()).collect()),
        )
        .y_axis(Axis::new().type_(AxisType::Value))
        .series(Line::new().data(data))
}

fn main() {
    let test_curve = CurveBuilder::<6>::new()
        .add_linear(0.3, 2.0, 0.0)
        .add_const(0.05, 0.0)
        .add_linear(0.5, 0.0, -2.0)
        .add_linear(0.5, 2.0, 0.0)
        .add_const(0.05, 0.0)
        .add_linear(0.3, 0.0, -2.0)
        .build()
        .unwrap();

    let chart = create_graph(
        0.0,
        I16F16::from_num(2.0),
        I16F16::from_num(-2.0),
        test_curve,
        0.05,
    );

    let mut renderer = HtmlRenderer::new("Force graph", 1000, 800);
    renderer.save(&chart, "chart.html").unwrap();
}
