use crate::layout::CustomAction;
use usbd_human_interface_device::device::consumer::MultipleConsumerReport;
use usbd_human_interface_device::page::Consumer;

pub fn media_report_for_action(action: CustomAction) -> Option<MultipleConsumerReport> {
    let usage_id = match action {
        CustomAction::MediaPlayPause => Consumer::PlayPause,
        CustomAction::MediaNext => Consumer::ScanNextTrack,
        CustomAction::MediaBack => Consumer::ScanPreviousTrack,
        CustomAction::MediaMute => Consumer::Mute,
        CustomAction::MediaVolumeUp => Consumer::VolumeIncrement,
        CustomAction::MediaVolumeDown => Consumer::VolumeDecrement,
        _ => return None,
    };

    Some(MultipleConsumerReport {
        codes: [
            usage_id,
            Consumer::Unassigned,
            Consumer::Unassigned,
            Consumer::Unassigned,
        ],
    })
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

pub fn release_for_media_action() -> MultipleConsumerReport {
    MultipleConsumerReport {
        codes: [Consumer::Unassigned; 4],
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
