use super::*;
use test_case::test_case;

#[test_case(CustomAction::MediaPlayPause, MediaKey::PlayPause; "MediaPlayPause")]
#[test_case(CustomAction::MediaNext, MediaKey::NextTrack; "MediaNext")]
#[test_case(CustomAction::MediaBack, MediaKey::PrevTrack; "MediaBack")]
#[test_case(CustomAction::MediaMute, MediaKey::Mute; "MediaMute")]
#[test_case(CustomAction::MediaVolumeUp, MediaKey::VolumeIncrement; "MediaVolumeUp")]
#[test_case(CustomAction::MediaVolumeDown, MediaKey::VolumeDecrement; "MediaVolumeDown")]
fn check_press_media(action: CustomAction, expected: MediaKey) {
    let consumer = media_report_for_action(action);
    if let Some(MediaKeyboardReport { usage_id }) = consumer {
        assert_eq!(usage_id, expected.into());
    } else {
        panic!("Expected MediaKeyboardReport");
    }
}
