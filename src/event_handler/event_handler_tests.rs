use crate::layout::{CustomAction, LAYERS};
use assert_matches::assert_matches;
use usbd_hid::descriptor::MediaKey;

use super::*;

struct MockEventHandler {
    report: KbHidReport,
    written_report: Option<KbHidReport>,
    written_media_report: Option<MediaKeyboardReport>,
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

    fn write_media_report(&mut self, report: MediaKeyboardReport) {
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
        last_media_report: MediaKeyboardReport {
            usage_id: MediaKey::Zero.into(),
        },
    };

    handler.handle_event(Some(Event::Press(3, 14)));
    assert_eq!(handler.event_handler.written_report, None);
    assert_matches!(handler.event_handler.written_media_report, None);
    assert_eq!(handler.event_handler.serialized, None);

    handler.handle_event(None);
    assert_eq!(handler.event_handler.written_report, None);
    if let Some(MediaKeyboardReport { usage_id }) =
        handler.event_handler.written_media_report.take()
    {
        assert_eq!(usage_id, MediaKey::VolumeIncrement.into());
    } else {
        panic!("Expected MediaKeyboardReport");
    }

    assert_eq!(handler.event_handler.serialized, None);
    handler.event_handler.clear();

    handler.handle_event(Some(Event::Release(3, 14)));
    assert_eq!(handler.event_handler.written_report, None);
    assert_matches!(handler.event_handler.written_media_report, None);
    assert_eq!(handler.event_handler.serialized, None);

    handler.handle_event(None);
    assert_eq!(handler.event_handler.written_report, None);
    if let Some(MediaKeyboardReport { usage_id }) =
        handler.event_handler.written_media_report.take()
    {
        assert_eq!(usage_id, MediaKey::Zero.into());
    } else {
        panic!("Expected MediaKeyboardReport");
    }
    assert_eq!(handler.event_handler.serialized, None);
}
