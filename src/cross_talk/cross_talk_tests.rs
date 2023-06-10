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
