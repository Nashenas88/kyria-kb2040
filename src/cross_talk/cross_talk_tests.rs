use super::*;

#[test]
fn serialize_press() {
    for i in 0..8 {
        for j in 0..8 {
            let byte = serialize(Event::Press(i, j));
            assert_eq!(byte, Some((i << 3) | j | 0b1100_0000));
            let event = deserialize(byte.unwrap());
            assert_eq!(event, Some(Event::Press(i, j)));
        }
    }
}

#[test]
fn serialize_release() {
    for i in 0..8 {
        for j in 0..8 {
            let byte = serialize(Event::Release(i, j));
            assert_eq!(byte, Some((i << 3) | j | 0b1000_0000));
            let event = deserialize(byte.unwrap());
            assert_eq!(event, Some(Event::Release(i, j)));
        }
    }
}

#[test]
fn serialize_bad_i() {
    let byte = serialize(Event::Press(8, 0));
    assert_eq!(byte, None);
}

#[test]
fn serialize_bad_j() {
    let byte = serialize(Event::Press(0, 8));
    assert_eq!(byte, None);
}

#[test]
fn deserialize_empty() {
    let event = deserialize(0);
    assert_eq!(event, None);
}

#[test]
fn serialize_command() {
    let val: u32 = InterProcessCommand::ReadRotary.into();
    assert_eq!(
        InterProcessCommand::try_from(val),
        Ok(InterProcessCommand::ReadRotary)
    );
}

#[test]
fn serialize_response() {
    for ev in [
        Some(Event::Press(14, 4)),
        Some(Event::Press(12, 4)),
        Some(Event::Press(3, 4)),
        Some(Event::Press(1, 4)),
        Some(Event::Release(14, 4)),
        Some(Event::Release(12, 4)),
        Some(Event::Release(3, 4)),
        Some(Event::Release(1, 4)),
        None,
    ] {
        let val: u32 = InterProcessResponse::Rotary(ev).into();
        assert_eq!(
            InterProcessResponse::try_from(val),
            Ok(InterProcessResponse::Rotary(ev))
        );
    }
}
