use super::*;
use test_case::test_case;

#[test_case(CustomAction::MediaPlayPause, Consumer::PlayPause; "MediaPlayPause")]
#[test_case(CustomAction::MediaNext, Consumer::ScanNextTrack; "MediaNext")]
#[test_case(CustomAction::MediaBack, Consumer::ScanPreviousTrack; "MediaBack")]
#[test_case(CustomAction::MediaMute, Consumer::Mute; "MediaMute")]
#[test_case(CustomAction::MediaVolumeUp, Consumer::VolumeIncrement; "MediaVolumeUp")]
#[test_case(CustomAction::MediaVolumeDown, Consumer::VolumeDecrement; "MediaVolumeDown")]
fn check_press_media(action: CustomAction, expected: Consumer) {
    let consumer = media_report_for_action(action);
    assert_eq!(
        consumer,
        Some(MultipleConsumerReport {
            codes: [
                expected,
                Consumer::Unassigned,
                Consumer::Unassigned,
                Consumer::Unassigned,
            ]
        })
    );
}
