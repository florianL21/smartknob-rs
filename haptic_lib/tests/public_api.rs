use haptic_lib::*;
extern crate std;
// use core::matches;

fn torque(p: Playback) -> Value {
    if let Playback::Torque(t) = p {
        t
    } else {
        panic!("This is not a torque value: {p:#?}")
    }
}

#[test]
fn test_empty_curve() {
    let test_curve = CurveBuilder::<5>::new().build(0.0);
    assert!(matches!(test_curve, Err(CurveError::EmptyCurve)));
}

#[test]
fn test_basic_curve() {
    let test_curve = CurveBuilder::<5>::new()
        .add_const(0.5, 0.0)
        .add_const(2.0, 1.0)
        .add_const(0.5, 0.0)
        .build(0.0)
        .unwrap()
        .instantiate()
        .unwrap();
    let mut player = HapticPlayer::new(0.0, &test_curve);
    assert_eq!(torque(player.play(-1.0)), 0.0);
    assert_eq!(torque(player.play(1.0)), 1.0);
    assert_eq!(torque(player.play(3.0)), 0.0);
}

#[test]
fn test_curve_scaling() {
    let test_curve = CurveBuilder::<1>::new()
        .add_const(0.5, 0.5)
        .build(0.0)
        .unwrap()
        .instantiate()
        .unwrap();
    let mut player = HapticPlayer::new(0.0, &test_curve).with_scale(2.0);
    assert_eq!(torque(player.play(0.1)), 1.0);
}

#[test]
fn test_curve_post_end() {
    let test_curve = CurveBuilder::<5>::new()
        .add_const(0.5, 0.4)
        .add_const(2.0, 1.0)
        .add_const(0.5, 0.2)
        .build(0.0)
        .unwrap()
        .instantiate()
        .unwrap();
    let mut player = HapticPlayer::new(0.0, &test_curve);
    assert_eq!(torque(player.play(5.0)), 0.2);
}

#[test]
fn test_curve_pre_start() {
    let test_curve = CurveBuilder::<5>::new()
        .add_const(0.5, 0.4)
        .add_const(2.0, 1.0)
        .add_const(0.5, 0.2)
        .build(0.0)
        .unwrap()
        .instantiate()
        .unwrap();
    let mut player = HapticPlayer::new(0.0, &test_curve);
    assert_eq!(torque(player.play(-1.0)), 0.4);
}

#[test]
fn test_curve_player_start_with_offset() {
    let test_curve = CurveBuilder::<5>::new()
        .add_const(0.5, 0.4)
        .add_const(2.0, 1.0)
        .add_const(0.5, 0.2)
        .build(0.0)
        .unwrap()
        .instantiate()
        .unwrap();
    let mut player = HapticPlayer::new(5.0, &test_curve);
    assert_eq!(torque(player.play(5.5)), 1.0);
    assert_eq!(torque(player.play(7.49)), 1.0);
    assert_eq!(torque(player.play(7.5)), 0.2);
}

#[test]
fn test_curve_with_offset() {
    let test_curve = CurveBuilder::<5>::new()
        .add_const(0.5, 0.4)
        .add_const(2.0, 1.0)
        .add_const(0.5, 0.2)
        .build(1.0)
        .unwrap()
        .instantiate()
        .unwrap();
    let mut player = HapticPlayer::new(0.0, &test_curve);
    assert_eq!(torque(player.play(0.99)), 0.4);
    assert_eq!(torque(player.play(1.5)), 1.0);
    assert_eq!(torque(player.play(3.5)), 0.2);
}
