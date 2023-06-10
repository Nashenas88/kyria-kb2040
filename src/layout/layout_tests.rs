use super::*;
use keyberon::layout::{CustomEvent, Event, Layout};
use test_case::test_case;

#[test_case(CustomAction::QwertyLed; "QwertyLed")]
#[test_case(CustomAction::ColemakLed; "ColemakLed")]
#[test_case(CustomAction::LayerSelectLed; "LayerSelectLed")]
#[test_case(CustomAction::NumpadLed; "NumpadLed")]
#[test_case(CustomAction::NavLed; "NavLed")]
#[test_case(CustomAction::SymLed; "SymLed")]
fn custom_action_serialize(action: CustomAction) {
    let byte = u8::from(action);
    let action2 = CustomAction::from(byte);
    assert_eq!(action2, action);
}

#[test_case(CustomAction::QwertyLed; "QwertyLed")]
#[test_case(CustomAction::ColemakLed; "ColemakLed")]
#[test_case(CustomAction::LayerSelectLed; "LayerSelectLed")]
#[test_case(CustomAction::NumpadLed; "NumpadLed")]
#[test_case(CustomAction::NavLed; "NavLed")]
#[test_case(CustomAction::SymLed; "SymLed")]
fn custom_action_is_led(action: CustomAction) {
    assert!(action.is_led());
}

#[test_case(CustomAction::MediaPlayPause; "MediaPlayPayse")]
#[test_case(CustomAction::MediaNext; "MediaNext")]
#[test_case(CustomAction::MediaBack; "MediaBack")]
#[test_case(CustomAction::MediaMute; "MediaMute")]
#[test_case(CustomAction::MediaVolumeUp; "MediaVolumeUp")]
#[test_case(CustomAction::MediaVolumeDown; "MediaVolumeDown")]
fn custom_action_is_not_led(action: CustomAction) {
    assert!(!action.is_led());
}

#[test]
fn rotary_keys() {
    let mut layout = Layout::new(&LAYERS);
    layout.event(Event::Press(3, 14));
    let event = layout.tick();
    assert_eq!(event, CustomEvent::Press(&CustomAction::MediaVolumeUp));

    layout.event(Event::Release(3, 14));
    let event = layout.tick();
    assert_eq!(event, CustomEvent::Release(&CustomAction::MediaVolumeUp));
    let mut layout = Layout::new(&LAYERS);

    layout.event(Event::Press(3, 12));
    let event = layout.tick();
    assert_eq!(event, CustomEvent::Press(&CustomAction::MediaVolumeDown));

    layout.event(Event::Release(3, 12));
    let event = layout.tick();
    assert_eq!(event, CustomEvent::Release(&CustomAction::MediaVolumeDown));
}
