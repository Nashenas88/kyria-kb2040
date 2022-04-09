use keyberon::keyboard::Leds;

pub struct UsualLeds;

impl Leds for UsualLeds {
    fn num_lock(&mut self, _status: bool) {}

    fn caps_lock(&mut self, _status: bool) {}

    fn scroll_lock(&mut self, _status: bool) {}

    fn compose(&mut self, _status: bool) {}

    fn kana(&mut self, _status: bool) {}
}
