use usbd_hid::descriptor::{MediaKey, MediaKeyboardReport};

use crate::layout::CustomAction;

pub(crate) fn report_for_action(action: CustomAction) -> MediaKeyboardReport {
    let usage_id = match action {
        CustomAction::MediaPlayPause => MediaKey::PlayPause.into(),
        CustomAction::MediaNext => MediaKey::NextTrack.into(),
        CustomAction::MediaBack => MediaKey::PrevTrack.into(),
        CustomAction::MediaMute => MediaKey::Mute.into(),
        CustomAction::MediaVolumeUp => MediaKey::VolumeIncrement.into(),
        CustomAction::MediaVolumeDown => MediaKey::VolumeDecrement.into(),
        _ => unreachable!(),
    };

    MediaKeyboardReport { usage_id }
}

pub(crate) fn release_for_action(action: CustomAction) -> Option<MediaKeyboardReport> {
    if action.is_led() {
        None
    } else {
        Some(MediaKeyboardReport {
            usage_id: MediaKey::Zero.into(),
        })
    }
}
