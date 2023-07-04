use crate::layout::CustomAction;
use usbd_hid::descriptor::{MediaKey, MediaKeyboardReport};

pub fn media_report_for_action(action: CustomAction) -> Option<MediaKeyboardReport> {
    let usage_id = match action {
        CustomAction::MediaPlayPause => MediaKey::PlayPause,
        CustomAction::MediaNext => MediaKey::NextTrack,
        CustomAction::MediaBack => MediaKey::PrevTrack,
        CustomAction::MediaMute => MediaKey::Mute,
        CustomAction::MediaVolumeUp => MediaKey::VolumeIncrement,
        CustomAction::MediaVolumeDown => MediaKey::VolumeDecrement,
        _ => return None,
    }
    .into();

    Some(MediaKeyboardReport { usage_id })
}

pub fn release_for_media_action() -> MediaKeyboardReport {
    MediaKeyboardReport {
        usage_id: MediaKey::Zero.into(),
    }
}

#[cfg(test)]
mod media_tests;
