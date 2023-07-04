use smart_leds::{RGB, RGB8};

const NUM_LEDS: usize = 10;

pub enum SwitchKind {
    Momentary,
    Toggle,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AnimKind {
    Boot,
    Colemak,
    Qwerty,
    LayerSelect,
    Num,
    Nav,
    Sym,
    Function,
    Rainbow,
    PongStart,
    PongScoreLeft,
    PongScoreRight,
    Error(u8),
}

const BOOT_ANIM_MS: u32 = 2000;
const COLEMAK_ANIM_MS: u32 = 3000;
const QWERTY_ANIM_MS: u32 = 3000;
const LAYER_SELECT_ANIM_MS: u32 = 1000;
const NUM_ANIM_MS: u32 = 3000;
const NAV_ANIM_MS: u32 = 3000;
const SYM_ANIM_MS: u32 = 3000;
const FUNCTION_ANIM_MS: u32 = 3000;
const RAINBOW_ANIM_MS: u32 = 1000;
const TRANSITION_MS: u32 = 750;
const GAUSS_STD_DEV: u32 = 75;
const PONG_START_MS: u32 = 4000;
const PONG_SCORE_MS: u32 = 4000;

impl AnimKind {
    fn new() -> AnimKind {
        AnimKind::Boot
    }

    fn anim_time_ms(&self) -> u32 {
        match self {
            AnimKind::Boot => BOOT_ANIM_MS,
            AnimKind::Colemak => COLEMAK_ANIM_MS,
            AnimKind::Qwerty => QWERTY_ANIM_MS,
            AnimKind::LayerSelect => LAYER_SELECT_ANIM_MS,
            AnimKind::Num => NUM_ANIM_MS,
            AnimKind::Nav => NAV_ANIM_MS,
            AnimKind::Sym => SYM_ANIM_MS,
            AnimKind::Function => FUNCTION_ANIM_MS,
            AnimKind::Rainbow => RAINBOW_ANIM_MS,
            AnimKind::PongStart => PONG_START_MS,
            AnimKind::PongScoreLeft | AnimKind::PongScoreRight => PONG_SCORE_MS,
            AnimKind::Error(e) => (e + 1) as u32 * 1000,
        }
    }

    pub fn switch_kind(&self) -> SwitchKind {
        match self {
            AnimKind::Boot => SwitchKind::Momentary,
            AnimKind::Colemak => SwitchKind::Toggle,
            AnimKind::Qwerty => SwitchKind::Toggle,
            AnimKind::LayerSelect => SwitchKind::Momentary,
            AnimKind::Num => SwitchKind::Toggle,
            AnimKind::Nav => SwitchKind::Toggle,
            AnimKind::Sym => SwitchKind::Momentary,
            AnimKind::Function => SwitchKind::Toggle,
            AnimKind::Rainbow => SwitchKind::Toggle,
            AnimKind::PongStart => SwitchKind::Toggle,
            AnimKind::PongScoreLeft => SwitchKind::Toggle,
            AnimKind::PongScoreRight => SwitchKind::Toggle,
            AnimKind::Error(_) => SwitchKind::Toggle,
        }
    }
}

impl Default for AnimKind {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Default, Copy, Clone, Debug, PartialEq, Eq)]
pub struct AnimState {
    kind: AnimKind,
    /// Current number of milliseconds into the current animation.
    anim_ms: u32,
}

impl AnimState {
    pub fn new() -> Self {
        Self {
            kind: AnimKind::new(),
            anim_ms: 0,
        }
    }

    fn tick(&mut self, delta_ms: u32) -> bool {
        self.anim_ms = self.anim_ms.wrapping_add(delta_ms);
        if self.anim_ms >= self.kind.anim_time_ms() {
            self.anim_ms = 0;
            true
        } else {
            false
        }
    }

    pub fn leds(&self, is_right: bool) -> [RGB8; NUM_LEDS] {
        match self.kind {
            AnimKind::Boot => self.boot_leds(),
            AnimKind::Colemak => ease_out::<{ COLEMAK_ANIM_MS as usize }>(self.anim_ms, |i, p| {
                red(inverted(
                    gaussian::<GAUSS_STD_DEV>(i as u16 * 50 + 200, p) as u8
                ))
            }),
            AnimKind::Qwerty => ease_out::<{ QWERTY_ANIM_MS as usize }>(self.anim_ms, |i, p| {
                blue(inverted(
                    gaussian::<GAUSS_STD_DEV>(i as u16 * 50 + 200, p) as u8
                ))
            }),
            AnimKind::LayerSelect | AnimKind::Rainbow => {
                let mut leds = [RGB8::default(); NUM_LEDS];
                let pos = self.anim_ms as f32 / LAYER_SELECT_ANIM_MS as f32;
                for (led, idx) in leds.iter_mut().zip(BOARD_LED_ORDER.into_iter()) {
                    let mut t = idx as f32 / (NUM_LEDS - 1) as f32 + pos;
                    if t > 1.0 {
                        t -= 1.0;
                    }
                    *led = rainbow_wheel(t);
                }
                leds
            }
            AnimKind::Num => ease_out::<{ NUM_ANIM_MS as usize }>(self.anim_ms, |i, p| {
                yellow(inverted(
                    gaussian::<GAUSS_STD_DEV>(i as u16 * 50 + 200, p) as u8
                ))
            }),
            AnimKind::Nav => ease_out::<{ NAV_ANIM_MS as usize }>(self.anim_ms, |i, p| {
                orange(inverted(
                    gaussian::<GAUSS_STD_DEV>(i as u16 * 50 + 200, p) as u8
                ))
            }),
            AnimKind::Sym => ease_out::<{ SYM_ANIM_MS as usize }>(self.anim_ms, |i, p| {
                green(inverted(
                    gaussian::<GAUSS_STD_DEV>(i as u16 * 50 + 200, p) as u8
                ))
            }),
            AnimKind::Function => ease_out::<{ SYM_ANIM_MS as usize }>(self.anim_ms, |i, p| {
                purple(inverted(
                    gaussian::<GAUSS_STD_DEV>(i as u16 * 50 + 200, p) as u8
                ))
            }),
            AnimKind::PongStart => solid(RGB {
                r: 255,
                g: 255,
                b: 255,
            }),
            AnimKind::PongScoreLeft => {
                if is_right {
                    solid(RGB { r: 0, g: 0, b: 0 })
                } else {
                    solid(RGB {
                        r: 255,
                        g: 255,
                        b: 255,
                    })
                }
            }
            AnimKind::PongScoreRight => {
                if is_right {
                    solid(RGB {
                        r: 255,
                        g: 255,
                        b: 255,
                    })
                } else {
                    solid(RGB { r: 0, g: 0, b: 0 })
                }
            }
            AnimKind::Error(e) => error(e, self.anim_ms),
        }
    }

    fn boot_leds(&self) -> [RGB8; NUM_LEDS] {
        let mut leds = [RGB8::default(); NUM_LEDS];
        const SECTION_MS: usize = BOOT_ANIM_MS as usize / 3;
        if self.anim_ms <= SECTION_MS as u32 {
            let idx = (EASE_OUT.len() - 1) * self.anim_ms as usize / SECTION_MS;
            let pos = EASE_OUT[idx];
            for (led, idx) in leds.iter_mut().zip(BOARD_LED_ORDER.into_iter()) {
                *led = red(gaussian::<GAUSS_STD_DEV>(idx as u16 * 100, pos) as u8);
            }
            leds
        } else if self.anim_ms <= 2 * SECTION_MS as u32 {
            let idx = (EASE_OUT.len() - 1) * (self.anim_ms as usize - SECTION_MS) / SECTION_MS;
            let pos = NUM_LEDS as u16 * 100 - EASE_OUT[idx];
            for (led, idx) in leds.iter_mut().zip(BOARD_LED_ORDER.into_iter()) {
                *led = red(gaussian::<GAUSS_STD_DEV>(idx as u16 * 100, pos) as u8);
            }
            leds
        } else {
            let idx = (EASE_OUT.len() - 1) * (self.anim_ms as usize - 2 * SECTION_MS) / SECTION_MS;
            let intensity = SIN[idx];
            [RGB {
                r: intensity,
                g: 0,
                b: 0,
            }; NUM_LEDS]
        }
    }

    fn from_kind(kind: AnimKind) -> AnimState {
        Self { kind, anim_ms: 0 }
    }
}

fn inverted(val: u8) -> u8 {
    255 - val
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

fn purple(c: u8) -> RGB8 {
    RGB { r: c, g: 0, b: c }
}

fn yellow(c: u8) -> RGB8 {
    RGB { r: c, g: c, b: 0 }
}

fn orange(c: u8) -> RGB8 {
    RGB {
        r: c,
        g: c / 2,
        b: 0,
    }
}

fn rainbow_wheel(t: f32) -> RGB8 {
    const COLORS: [(u8, u8, u8); 7] = [
        (255, 0, 0),     // red
        (255, 165, 0),   // orange
        (255, 255, 0),   // yellow
        (0, 255, 0),     // green
        (0, 0, 255),     // blue
        (75, 0, 130),    // indigo
        (238, 130, 238), // violet
    ];
    const COLOR_WIDTH: f32 = 1.0 / COLORS.len() as f32;

    let idx = t / COLOR_WIDTH;
    let left = idx as usize;
    let right = (idx + 1.0) as usize % COLORS.len();

    let left_color = COLORS[left];
    let right_color = COLORS[right];
    let blend = idx - left as f32;
    let (r, g, b) = (
        (right_color.0 as f32 * blend + (left_color.0 as f32) * (1.0 - blend)) as u8,
        (right_color.1 as f32 * blend + (left_color.1 as f32) * (1.0 - blend)) as u8,
        (right_color.2 as f32 * blend + (left_color.2 as f32) * (1.0 - blend)) as u8,
    );

    RGB { r, g, b }
}

fn gaussian<const STD_DEV: u32>(x: u16, mean: u16) -> u16 {
    const PEAK: u8 = 255;
    let sqr_std_dev_2 = (2 * STD_DEV * STD_DEV) as f32;
    let diff = x as f32 - mean as f32;
    let exp = -(diff * diff / sqr_std_dev_2);
    (PEAK as f32 * exp_approx(exp)) as u16
}

#[cfg(feature = "std")]
const BOARD_LED_ORDER: [usize; NUM_LEDS] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
#[cfg(not(feature = "std"))]
const BOARD_LED_ORDER: [usize; NUM_LEDS] = [0, 1, 2, 5, 6, 7, 8, 9, 4, 3];

fn ease_out<const ANIM_MS: usize>(t: u32, color: impl Fn(u8, u16) -> RGB8) -> [RGB8; NUM_LEDS] {
    let mut leds = [RGB8::default(); NUM_LEDS];
    let idx = (EASE_OUT.len() - 1) * t as usize / ANIM_MS;
    let pos = EASE_OUT[idx];
    for (led, idx) in leds.iter_mut().zip(BOARD_LED_ORDER.into_iter()) {
        *led = color(idx as u8, pos);
    }
    leds
}

fn error(e: u8, anim_ms: u32) -> [RGB8; NUM_LEDS] {
    let half_secs = anim_ms / 250;
    if half_secs >= e as u32 * 2 || half_secs % 2 == 1 {
        [RGB8::default(); NUM_LEDS]
    } else {
        [RGB8 { r: 255, g: 0, b: 0 }; NUM_LEDS]
    }
}

// fn marquee<const ANIM_MS: usize>(t: u32, color: impl Fn(u8, u16) -> RGB8) -> [RGB8; NUM_LEDS] {
//     let mut leds = [RGB8::default(); NUM_LEDS];

//     leds
// }

fn solid(color: RGB8) -> [RGB8; NUM_LEDS] {
    [color; NUM_LEDS]
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InternalState {
    Steady(AnimState),
    Transition(AnimState, AnimState, u32),
}

impl InternalState {
    fn tick(&mut self, delta_ms: u32) -> bool {
        match self {
            InternalState::Steady(state) => state.tick(delta_ms),
            InternalState::Transition(from, to, count) => {
                from.tick(delta_ms);
                *count += 1;
                to.tick(delta_ms)
            }
        }
    }
}

impl Default for InternalState {
    fn default() -> Self {
        Self::Steady(AnimState::default())
    }
}

#[derive(Default, Clone, PartialEq)]
pub struct AnimationController {
    state: InternalState,
}

impl AnimationController {
    pub fn new() -> Self {
        Self {
            state: InternalState::default(),
        }
    }

    pub fn leds(&self, is_right: bool) -> [RGB8; NUM_LEDS] {
        match self.state {
            InternalState::Steady(state) => state.leds(is_right),
            InternalState::Transition(from, to, t) => {
                self.leds_for_transition(&from.leds(is_right), &to.leds(is_right), t)
            }
        }
    }

    pub fn tick(&mut self, delta_ms: u32) -> bool {
        if let InternalState::Transition(_, to, ref mut count) = self.state {
            *count += delta_ms;
            if *count >= TRANSITION_MS {
                self.state = InternalState::Steady(to);
            }
        }
        self.state.tick(delta_ms)
    }

    pub fn state(&self) -> AnimKind {
        let (InternalState::Steady(state) | InternalState::Transition(_, state, _)) = self.state;
        state.kind
    }

    pub fn set_state(&mut self, state: AnimKind) {
        self.state = match self.state {
            InternalState::Steady(from) => InternalState::Transition(
                from,
                AnimState {
                    kind: state,
                    anim_ms: from.anim_ms % state.anim_time_ms(),
                },
                0,
            ),
            InternalState::Transition(_, from, _) => InternalState::Transition(
                from,
                AnimState {
                    kind: state,
                    anim_ms: from.anim_ms % state.anim_time_ms(),
                },
                0,
            ),
        }
    }

    pub fn force_state(&mut self, state: AnimKind) {
        self.state = InternalState::Steady(AnimState::from_kind(state));
    }

    #[inline(never)]
    fn interp(c1: RGB8, c2: RGB8, t: u32) -> RGB8 {
        let r = (c1.r as u64 * (TRANSITION_MS.saturating_sub(t) as u64) / TRANSITION_MS as u64
            + c2.r as u64 * t as u64 / TRANSITION_MS as u64) as u8;
        let g = (c1.g as u64 * (TRANSITION_MS.saturating_sub(t) as u64) / TRANSITION_MS as u64
            + c2.g as u64 * t as u64 / TRANSITION_MS as u64) as u8;
        let b = (c1.b as u64 * (TRANSITION_MS.saturating_sub(t) as u64) / TRANSITION_MS as u64
            + c2.b as u64 * t as u64 / TRANSITION_MS as u64) as u8;
        RGB { r, g, b }
    }

    fn leds_for_transition<const NUM: usize>(
        &self,
        from: &[RGB<u8>; NUM],
        to: &[RGB<u8>; NUM],
        t: u32,
    ) -> [RGB<u8>; NUM] {
        let mut leds = [RGB::default(); NUM];
        for (led, (from, to)) in leds.iter_mut().zip(from.iter().zip(to.iter())) {
            *led = Self::interp(*from, *to, t)
        }
        leds
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

#[inline(always)]
fn exp_approx(mut x: f32) -> f32 {
    const A: f32 = (1 << 23) as f32 / core::f32::consts::LN_2;
    const B: f32 = (1 << 23) as f32 * (127.0 - 0.043677448);
    x = A * x + B;

    const C: f32 = (1 << 23) as f32;
    const D: f32 = (1 << 23) as f32 * 255.0;
    if x < C || x > D {
        x = if x < C { 0.0 } else { D };
    }

    let n: u32 = x as u32;
    x = unsafe { core::mem::transmute_copy(&n) };
    // memcpy(&x, &n, 4);
    return x;
}

#[cfg(test)]
mod anim_tests;
