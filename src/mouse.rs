use pimoroni_trackball_driver::TrackballData;

pub(crate) trait MouseReportExt {
    fn is_dead_and_same(&self, last: &Self) -> bool;
}

impl MouseReportExt for TrackballData {
    fn is_dead_and_same(&self, last: &Self) -> bool {
        if self.up != 0
            || self.down != 0
            || self.right != 0
            || self.left != 0
            || self.switch_changed
        {
            return false;
        }

        self.up == last.up
            && self.down == last.down
            && self.right == last.right
            && self.left == last.left
            && self.switch_changed == last.switch_changed
            && self.switch_pressed == last.switch_pressed
    }
}
