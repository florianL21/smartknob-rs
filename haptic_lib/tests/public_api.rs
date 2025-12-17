use haptic_lib::*;
extern crate std;

#[test]
fn test_empty_curve() {
    let test_curve = CurveBuilder::<5>::new().build();
    assert!(matches!(test_curve, Err(CurveError::EmptyCurve)));
}

#[test]
fn test_basic_curve() {
    let test_curve = CurveBuilder::<5>::new()
        .add_const(0.5, 0.0)
        .add_const(2.0, 1.0)
        .add_const(0.5, 0.0)
        .build()
        .unwrap()
        .make_absolute(Angle::ZERO);
    let mut player = HapticPlayer::new(Angle::ZERO, &test_curve);
    assert_eq!(player.play(Angle::from_num(-1)), Value::ZERO);
    assert_eq!(player.play(Angle::from_num(1)), Value::from_num(1.0));
    assert_eq!(player.play(Angle::from_num(3)), Value::ZERO);
}

#[test]
fn test_curve_scaling() {
    let test_curve = CurveBuilder::<1>::new()
        .add_const(0.5, 0.5)
        .build()
        .unwrap()
        .make_absolute(Angle::ZERO);
    let mut player = HapticPlayer::new(Angle::ZERO, &test_curve).with_scale(2.0);
    assert_eq!(player.play(Angle::from_num(0.1)), Value::from_num(1.0));
}
