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

const COMMAND_MASK: u32 = 0x0000_00FF;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InterProcessCommand {
    ReadRotary,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InterProcessResponse {
    Rotary(Option<Event>),
    Error,
}

impl TryFrom<u32> for InterProcessCommand {
    type Error = u32;

    fn try_from(val: u32) -> Result<Self, Self::Error> {
        match val & COMMAND_MASK {
            1 => Ok(Self::ReadRotary),
            _ => Err(val),
        }
    }
}

impl From<InterProcessCommand> for u32 {
    fn from(command: InterProcessCommand) -> Self {
        match command {
            InterProcessCommand::ReadRotary => 1,
        }
    }
}

const PRESS_VAL: u32 = 1 << 16;
const RELEASE_VAL: u32 = 2 << 16;
const NONE_VAL: u32 = 4 << 16;

impl From<InterProcessResponse> for u32 {
    fn from(response: InterProcessResponse) -> Self {
        match response {
            InterProcessResponse::Rotary(event) => match event {
                Some(Event::Press(row, col)) => PRESS_VAL | (row as u32) << 8 | col as u32,
                Some(Event::Release(row, col)) => RELEASE_VAL | (row as u32) << 8 | col as u32,
                None => 3,
            },
            InterProcessResponse::Error => 0,
        }
    }
}

impl TryFrom<u32> for InterProcessResponse {
    type Error = u32;

    fn try_from(val: u32) -> Result<Self, <Self as TryFrom<u32>>::Error> {
        match val {
            0 => return Ok(Self::Error),
            3 => return Ok(Self::Rotary(None)),
            _ => {}
        }

        let row = (val & 0x0000_FF00) >> 8;
        let col = val & 0x0000_00FF;
        Ok(match val & 0x00FF_0000 {
            PRESS_VAL => InterProcessResponse::Rotary(Some(Event::Press(row as u8, col as u8))),
            RELEASE_VAL => InterProcessResponse::Rotary(Some(Event::Release(row as u8, col as u8))),
            NONE_VAL => InterProcessResponse::Rotary(None),
            _ => return Err(val),
        })
    }
}

#[cfg(test)]
mod cross_talk_tests;
