use usbd_human_interface_device::page::Consumer;

use crate::layout::LAYERS;

use super::*;

struct MockEventHandler {
    report: KbHidReport,
    written_report: Option<KbHidReport>,
    written_media_report: Option<MultipleConsumerReport>,
    serialized: Option<CustomAction>,
}

impl MockEventHandler {
    fn new() -> Self {
        Self {
            report: KbHidReport::default(),
            written_report: None,
            written_media_report: None,
            serialized: None,
        }
    }

    fn clear(&mut self) {
        self.written_report = None;
        self.written_media_report = None;
        self.serialized = None;
    }
}

impl EventHandler for MockEventHandler {
    fn set_keyboard_report(&mut self, report: KbHidReport) -> bool {
        if report == self.report {
            false
        } else {
            self.report = report;
            true
        }
    }

    fn write_keyboard_report(&mut self, report: KbHidReport) {
        self.written_report = Some(report);
    }

    fn write_media_report(&mut self, report: MultipleConsumerReport) {
        self.written_media_report = Some(report);
    }

    fn write_ui(&mut self, serialized: u8) {
        self.serialized = Some(serialized.into());
    }

    fn usb_configured(&self) -> bool {
        true
    }
}

#[test]
fn test_rotary() {
    let event_handler = MockEventHandler::new();
    let layout = Layout::new(&LAYERS);
    let mut handler = Handler {
        event_handler,
        layout,
        last_media_report: MultipleConsumerReport::default(),
    };

    handler.handle_event(Some(Event::Press(3, 14)));
    assert_eq!(handler.event_handler.written_report, None);
    assert_eq!(handler.event_handler.written_media_report, None);
    assert_eq!(handler.event_handler.serialized, None);

    handler.handle_event(None);
    assert_eq!(handler.event_handler.written_report, None);
    assert_eq!(
        handler.event_handler.written_media_report,
        Some(MultipleConsumerReport {
            codes: [
                Consumer::VolumeIncrement,
                Consumer::Unassigned,
                Consumer::Unassigned,
                Consumer::Unassigned
            ]
        })
    );
    assert_eq!(handler.event_handler.serialized, None);
    handler.event_handler.clear();

    handler.handle_event(Some(Event::Release(3, 14)));
    assert_eq!(handler.event_handler.written_report, None);
    assert_eq!(handler.event_handler.written_media_report, None);
    assert_eq!(handler.event_handler.serialized, None);

    handler.handle_event(None);
    assert_eq!(handler.event_handler.written_report, None);
    assert_eq!(
        handler.event_handler.written_media_report,
        Some(MultipleConsumerReport {
            codes: [
                Consumer::Unassigned,
                Consumer::Unassigned,
                Consumer::Unassigned,
                Consumer::Unassigned
            ]
        })
    );
    assert_eq!(handler.event_handler.serialized, None);
}
