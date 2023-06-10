use keyberon::layout::Event;
use rotary_encoder_hal::Direction;

use crate::log;

fn new_rotary_event(event: Event) -> Option<(Event, u8)> {
    Some((event, 0))
}

pub fn update_last_rotary<const WAIT: u8>(
    last_rotary: &mut Option<(Event, u8)>,
    rotary_update: Direction,
    clockwise: u8,
    counterclockwise: u8,
) -> Option<Event> {
    let mut event = None;
    if let Some((revent, counter)) = last_rotary.take() {
        if counter < WAIT {
            *last_rotary = Some((revent, counter + 1));
        } else {
            match revent {
                Event::Press(x, y) => {
                    event = Some(Event::Release(x, y));
                    // Clear out the last rotary event now that it's released.
                    *last_rotary = None;
                }
                Event::Release(..) => log::unreachable!(),
            }
        }
    }

    let rotary_event = match rotary_update {
        Direction::Clockwise => new_rotary_event(Event::Press(3, clockwise)),
        Direction::CounterClockwise => new_rotary_event(Event::Press(3, counterclockwise)),
        Direction::None => None,
    };
    // Only override if there's a new event.
    if let e @ Some((rotary_event, _)) = rotary_event {
        *last_rotary = e;
        event = Some(rotary_event);
    }

    event
}

#[cfg(test)]
mod rotary_tests;
