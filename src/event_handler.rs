use crate::layout::{CustomAction, Layout};
use keyberon::key_code::{self, KbHidReport};
use keyberon::layout::Event;
use usbd_human_interface_device::device::consumer::MultipleConsumerReport;

use crate::{log, media};

pub trait EventHandler {
    fn set_keyboard_report(&mut self, report: KbHidReport) -> bool;
    fn write_keyboard_report(&mut self, report: KbHidReport);
    fn write_media_report(&mut self, report: MultipleConsumerReport);
    fn write_ui(&mut self, serialized: u8);
    fn usb_configured(&self) -> bool;
}

struct Handler<T> {
    event_handler: T,
    layout: Layout,
    last_media_report: MultipleConsumerReport,
}

impl<T> Handler<T>
where
    T: EventHandler,
{
    fn handle_event(&mut self, event: Option<Event>)
    where
        T: EventHandler,
    {
        // If there's an event, process it with the layout and return early. We use `None`
        // to signify that the current scan is done with its events.
        if let Some(e) = event {
            match e {
                Event::Press(x, y) => log::info!("p {} {}", x, y),
                Event::Release(x, y) => log::info!("r {} {}", x, y),
            };
            self.layout.event(e);
            return;
        }

        // "Tick" the layout so that it gets to a consistent state, then read the keycodes into
        // a Keyboard HID Report.
        let (report, media_report): (key_code::KbHidReport, Option<MultipleConsumerReport>) = {
            let media_report = match self.layout.tick() {
                keyberon::layout::CustomEvent::Press(&custom_action) => {
                    if custom_action.is_led() {
                        let serialized = custom_action.into();
                        self.event_handler.write_ui(serialized);
                        None
                    } else {
                        media::media_report_for_action(custom_action)
                    }
                }
                keyberon::layout::CustomEvent::Release(&custom_action) => {
                    if custom_action.is_led() {
                        None
                    } else {
                        Some(media::release_for_media_action())
                    }
                }
                _ => None, // no-op for no-event
            };
            (self.layout.keycodes().collect(), media_report)
        };

        // Set the keyboard report on the usb class.
        let was_report_modified = self.event_handler.set_keyboard_report(report.clone());
        let media_report_modified = {
            if let Some(media_report) = media_report {
                if self.last_media_report == media_report {
                    false
                } else {
                    self.last_media_report = media_report;
                    true
                }
            } else {
                false
            }
        };
        if !(was_report_modified || media_report_modified) {
            return;
        }

        // If the device is not configured yet, we need to bail out.
        if !self.event_handler.usb_configured() {
            return;
        }

        if was_report_modified {
            self.event_handler.write_keyboard_report(report);
        }

        if media_report_modified {
            self.event_handler
                .write_media_report(self.last_media_report);
        }
    }
}

#[cfg(test)]
mod event_handler_tests;
