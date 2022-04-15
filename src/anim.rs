use smart_leds::{RGB, RGB8};

#[derive(Copy, Clone)]
pub enum AnimState {
    Boot,
    Colemak,
    Qwerty,
    LayerSelect,
    Sym,
}

const NUM_LEDS: usize = 10;

impl Default for AnimState {
    fn default() -> Self {
        Self::new()
    }
}

impl AnimState {
    fn new() -> AnimState {
        AnimState::Boot
    }
}

#[derive(Default)]
pub struct AnimationController {
    state: AnimState,
    anim_count: u16,
}

impl AnimationController {
    pub fn new() -> Self {
        Self {
            state: AnimState::new(),
            anim_count: 0,
        }
    }

    pub fn leds(&self) -> [RGB8; NUM_LEDS] {
        match self.state {
            AnimState::Boot => self.boot_leds(),
            AnimState::Colemak => self.colemak_leds(),
            AnimState::Qwerty => self.qwerty_leds(),
            AnimState::LayerSelect => self.layer_leds(),
            AnimState::Sym => self.sym_leds(),
        }
    }

    pub fn tick(&mut self) -> bool {
        self.anim_count = self.anim_count.saturating_add(1);
        if self.anim_count > self.max_anim_count() {
            self.anim_count = 0;
            if let AnimState::Boot = self.state {
                return true;
            }
        }

        false
    }

    pub fn state(&self) -> AnimState {
        self.state
    }

    pub fn set_state(&mut self, state: AnimState) {
        self.state = state;
    }

    fn max_anim_count(&self) -> u16 {
        match self.state {
            AnimState::Boot => 1000,
            AnimState::Colemak => 1000,
            AnimState::Qwerty => 1000,
            AnimState::LayerSelect => 1000,
            AnimState::Sym => 1000,
        }
    }

    fn for_pos(idx: u8, pos: u8) -> u8 {
        if idx == pos {
            255
        } else if idx == pos + 1 || idx == pos - 1 {
            127
        } else {
            0
        }
    }

    fn for_slow_pos(idx: u8, pos: u16) -> u8 {
        if idx as u16 * 100 == pos {
            255
        } else if idx as u16 * 100 == pos + 1 || idx as u16 * 100 == pos - 1 {
            127
        } else {
            0
        }
    }

    fn red(c: u8) -> RGB8 {
        RGB { r: c, g: 0, b: 0 }
    }

    fn green(c: u8) -> RGB8 {
        RGB { r: 0, g: c, b: 0 }
    }

    fn blue(c: u8) -> RGB8 {
        RGB { r: 0, g: 0, b: c }
    }

    fn boot_leds(&self) -> [RGB8; NUM_LEDS] {
        if self.anim_count <= 333 {
            let pos = (EASE_OUT[self.anim_count as usize] / 100) as u8;
            [
                Self::red(Self::for_pos(0, pos)),
                Self::red(Self::for_pos(1, pos)),
                Self::red(Self::for_pos(2, pos)),
                Self::red(Self::for_pos(3, pos)),
                Self::red(Self::for_pos(4, pos)),
                Self::red(Self::for_pos(5, pos)),
                Self::red(Self::for_pos(6, pos)),
                Self::red(Self::for_pos(7, pos)),
                Self::red(Self::for_pos(8, pos)),
                Self::red(Self::for_pos(9, pos)),
            ]
        } else if self.anim_count <= 666 {
            let pos = NUM_LEDS as u8 - (EASE_OUT[(self.anim_count - 334) as usize] / 100) as u8;
            [
                Self::red(Self::for_pos(0, pos)),
                Self::red(Self::for_pos(1, pos)),
                Self::red(Self::for_pos(2, pos)),
                Self::red(Self::for_pos(3, pos)),
                Self::red(Self::for_pos(4, pos)),
                Self::red(Self::for_pos(5, pos)),
                Self::red(Self::for_pos(6, pos)),
                Self::red(Self::for_pos(7, pos)),
                Self::red(Self::for_pos(8, pos)),
                Self::red(Self::for_pos(9, pos)),
            ]
        } else {
            let intensity = SIN[(self.anim_count - 667) as usize];
            [RGB {
                r: intensity,
                g: 0,
                b: 0,
            }; NUM_LEDS]
        }
    }

    fn colemak_leds(&self) -> [RGB8; NUM_LEDS] {
        let pos = EASE_OUT[(self.anim_count / 3) as usize];
        [
            Self::red(255 - Self::for_slow_pos(0, pos)),
            Self::red(255 - Self::for_slow_pos(1, pos)),
            Self::red(255 - Self::for_slow_pos(2, pos)),
            Self::red(255 - Self::for_slow_pos(3, pos)),
            Self::red(255 - Self::for_slow_pos(4, pos)),
            Self::red(255 - Self::for_slow_pos(5, pos)),
            Self::red(255 - Self::for_slow_pos(6, pos)),
            Self::red(255 - Self::for_slow_pos(7, pos)),
            Self::red(255 - Self::for_slow_pos(8, pos)),
            Self::red(255 - Self::for_slow_pos(9, pos)),
        ]
    }

    fn qwerty_leds(&self) -> [RGB8; NUM_LEDS] {
        let pos = EASE_OUT[(self.anim_count / 3) as usize];
        [
            Self::blue(255 - Self::for_slow_pos(0, pos)),
            Self::blue(255 - Self::for_slow_pos(1, pos)),
            Self::blue(255 - Self::for_slow_pos(2, pos)),
            Self::blue(255 - Self::for_slow_pos(3, pos)),
            Self::blue(255 - Self::for_slow_pos(4, pos)),
            Self::blue(255 - Self::for_slow_pos(5, pos)),
            Self::blue(255 - Self::for_slow_pos(6, pos)),
            Self::blue(255 - Self::for_slow_pos(7, pos)),
            Self::blue(255 - Self::for_slow_pos(8, pos)),
            Self::blue(255 - Self::for_slow_pos(9, pos)),
        ]
    }

    fn layer_leds(&self) -> [RGB8; NUM_LEDS] {
        [RGB8::default(); NUM_LEDS]
    }

    fn sym_leds(&self) -> [RGB8; NUM_LEDS] {
        let pos = EASE_OUT[(self.anim_count / 3) as usize];
        [
            Self::green(255 - Self::for_slow_pos(0, pos)),
            Self::green(255 - Self::for_slow_pos(1, pos)),
            Self::green(255 - Self::for_slow_pos(2, pos)),
            Self::green(255 - Self::for_slow_pos(3, pos)),
            Self::green(255 - Self::for_slow_pos(4, pos)),
            Self::green(255 - Self::for_slow_pos(5, pos)),
            Self::green(255 - Self::for_slow_pos(6, pos)),
            Self::green(255 - Self::for_slow_pos(7, pos)),
            Self::green(255 - Self::for_slow_pos(8, pos)),
            Self::green(255 - Self::for_slow_pos(9, pos)),
        ]
    }
}

// Lookup table where SIN[i] == sin(i) (where i is in degrees).
const SIN: [u8; 360] = [
    0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 15, 17, 18, 20, 22, 24, 26, 28, 30,
    32, 35, 37, 39, 42, 44, 47, 49, 52, 55, 58, 60, 63, 66, 69, 72, 75, 78, 81, 85, 88, 91, 94, 97,
    101, 104, 107, 111, 114, 117, 121, 124, 127, 131, 134, 137, 141, 144, 147, 150, 154, 157, 160,
    163, 167, 170, 173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 202, 205, 208, 210, 213, 215,
    217, 220, 222, 224, 226, 229, 231, 232, 234, 236, 238, 239, 241, 242, 244, 245, 246, 248, 249,
    250, 251, 251, 252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253,
    252, 251, 251, 250, 249, 248, 246, 245, 244, 242, 241, 239, 238, 236, 234, 232, 231, 229, 226,
    224, 222, 220, 217, 215, 213, 210, 208, 205, 202, 200, 197, 194, 191, 188, 185, 182, 179, 176,
    173, 170, 167, 163, 160, 157, 154, 150, 147, 144, 141, 137, 134, 131, 127, 124, 121, 117, 114,
    111, 107, 104, 101, 97, 94, 91, 88, 85, 81, 78, 75, 72, 69, 66, 63, 60, 58, 55, 52, 49, 47, 44,
    42, 39, 37, 35, 32, 30, 28, 26, 24, 22, 20, 18, 17, 15, 13, 12, 11, 9, 8, 7, 6, 5, 4, 3, 2, 2,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];

const EASE_OUT: [u16; 334] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 7, 7, 8, 9, 9,
    10, 11, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 26, 27, 28, 30, 31, 33, 34, 36,
    37, 39, 40, 42, 44, 46, 47, 49, 51, 53, 55, 57, 59, 61, 63, 66, 68, 70, 73, 75, 77, 80, 82, 85,
    88, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120, 123, 127, 130, 133, 137, 140, 144, 148,
    151, 155, 159, 163, 167, 170, 174, 179, 183, 187, 191, 195, 200, 204, 208, 213, 217, 222, 226,
    231, 236, 240, 245, 250, 255, 260, 265, 270, 275, 280, 285, 290, 296, 301, 306, 311, 317, 322,
    328, 333, 339, 344, 350, 355, 361, 367, 372, 378, 384, 390, 396, 401, 407, 413, 419, 425, 431,
    437, 443, 449, 455, 461, 467, 472, 478, 484, 490, 496, 503, 509, 515, 521, 527, 532, 538, 544,
    550, 556, 562, 568, 574, 580, 586, 592, 598, 603, 609, 615, 621, 627, 632, 638, 644, 649, 655,
    660, 666, 671, 677, 682, 688, 693, 698, 703, 709, 714, 719, 724, 729, 734, 739, 744, 749, 754,
    759, 763, 768, 773, 777, 782, 786, 791, 795, 800, 804, 808, 812, 816, 820, 825, 829, 832, 836,
    840, 844, 848, 851, 855, 859, 862, 866, 869, 872, 876, 879, 882, 885, 888, 891, 894, 897, 900,
    903, 906, 909, 911, 914, 917, 919, 922, 924, 926, 929, 931, 933, 936, 938, 940, 942, 944, 946,
    948, 950, 952, 953, 955, 957, 959, 960, 962, 963, 965, 966, 968, 969, 971, 972, 973, 974, 976,
    977, 978, 979, 980, 981, 982, 983, 984, 985, 986, 987, 988, 988, 989, 990, 990, 991, 992, 992,
    993, 994, 994, 995, 995, 995, 996, 996, 997, 997, 997, 998, 998, 998, 998, 999, 999, 999, 999,
    999, 999, 999, 999, 999, 999, 1000,
];
