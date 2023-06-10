use super::*;

#[test]
fn rotary_check() {
    let mut last_rotary = None;
    let event = update_last_rotary::<1>(&mut last_rotary, Direction::Clockwise, 14, 12);
    assert_eq!(event, Some(Event::Press(3, 14)));

    let event = update_last_rotary::<1>(&mut last_rotary, Direction::None, 14, 12);
    assert_eq!(event, None);

    let event = update_last_rotary::<1>(&mut last_rotary, Direction::None, 14, 12);
    assert_eq!(event, Some(Event::Release(3, 14)));

    let event = update_last_rotary::<1>(&mut last_rotary, Direction::None, 14, 12);
    assert_eq!(event, None);
}
