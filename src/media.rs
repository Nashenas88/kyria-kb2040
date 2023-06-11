use crate::layout::CustomAction;
use usbd_hid::descriptor::{MediaKey, MediaKeyboardReport};
// use usbd_human_interface_device::device::consumer::MultipleConsumerReport;
// use usbd_human_interface_device::page::Consumer;

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

// #[gen_hid_descriptor(
//     (collection = APPLICATION, usage_page = CONSUMER, usage = CONSUMER_CONTROL) = {
//         (usage_page = CONSUMER,) = {
//             (usage = 0xE0,) = {
//                 #[item_settings data,variable,relative] volume=input;
//             };
//             // (usage = 0xE9,) = {
//             //     #[item_settings data,variable,relative] volume_incr=input;
//             // };
//             // (usage = 0xEA,) = {
//             //     #[item_settings data,variable,relative] volume_decr=input;
//             // };
//         };
//     }
// )]
// pub(crate) struct ConsumerReport {
//     pub volume: i8,
// }

// pub(crate) fn consumer_report_for_action(action: CustomAction) -> Option<ConsumerReport> {
//     let volume = match action {
//         CustomAction::MediaVolumeUp => 1,
//         CustomAction::MediaVolumeDown => -1,
//         _ => return None,
//     };

//     Some(ConsumerReport { volume })
// }

pub fn release_for_media_action() -> MediaKeyboardReport {
    MediaKeyboardReport {
        usage_id: MediaKey::Zero.into(),
    }
}

// pub(crate) fn release_for_consumer_action(action: CustomAction) -> Option<ConsumerReport> {
//     if action.is_led() {
//         None
//     } else {
//         Some(ConsumerReport { volume: 0 })
//     }
// }

#[cfg(test)]
mod media_tests;
