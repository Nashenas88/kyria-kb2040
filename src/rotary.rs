use keyberon::layout::Event;

pub(crate) fn new_rotary_event(event: Event) -> Option<(Event, u8)> {
    Some((event, 0))
}
