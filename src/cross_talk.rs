use crate::log;
use keyberon::layout::Event;

const EVENT_BIT_MASK: u8 = 0b1000_0000;
const PRESSED_BIT_MASK: u8 = 0b0100_0000;
const PRESSED_SHIFT: u8 = 6;
const ROW_BIT_MASK: u8 = 0b0011_1000;
const ROW_SHIFT: u8 = 3;
const COL_BIT_MASK: u8 = 0b0000_0111;
const COL_SHIFT: u8 = 0;

pub fn serialize(e: Event) -> Option<u8> {
    let (i, j) = e.coord();
    // 7 is the max value we can encode, and the highest row/col count on this board
    if i > 7 || j > 7 {
        return None;
    }

    let mut byte = (j << COL_SHIFT) & COL_BIT_MASK;
    byte |= (i << ROW_SHIFT) & ROW_BIT_MASK;
    byte |= (e.is_press() as u8) << PRESSED_SHIFT;
    byte |= EVENT_BIT_MASK;
    Some(byte)
}

pub fn deserialize(d: u8) -> Option<Event> {
    if (d & EVENT_BIT_MASK) == 0 {
        return None;
    }

    let i = (d & ROW_BIT_MASK) >> ROW_SHIFT;
    let j = (d & COL_BIT_MASK) >> COL_SHIFT;
    let is_pressed = (d & PRESSED_BIT_MASK) > 0;

    Some(if is_pressed {
        Event::Press(i, j)
    } else {
        Event::Release(i, j)
    })
}

#[cfg(test)]
mod cross_talk_tests;
