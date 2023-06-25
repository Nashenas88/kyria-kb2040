use keyberon::action::{self, d, l, HoldTapAction};
use keyberon::key_code::KeyCode;

// TODO: split into media, led variants and new enums.
#[allow(clippy::enum_variant_names)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CustomAction {
    QwertyLed,
    ColemakLed,
    LayerSelectLed,
    NumpadLed,
    NavLed,
    SymLed,
    FunctionLed,
    MediaPlayPause,
    MediaNext,
    MediaBack,
    MediaMute,
    MediaVolumeUp,
    MediaVolumeDown,
}

impl CustomAction {
    pub fn is_led(&self) -> bool {
        match self {
            CustomAction::QwertyLed
            | CustomAction::ColemakLed
            | CustomAction::LayerSelectLed
            | CustomAction::NumpadLed
            | CustomAction::NavLed
            | CustomAction::SymLed
            | CustomAction::FunctionLed => true,
            CustomAction::MediaPlayPause
            | CustomAction::MediaNext
            | CustomAction::MediaBack
            | CustomAction::MediaMute
            | CustomAction::MediaVolumeUp
            | CustomAction::MediaVolumeDown => false,
        }
    }
}

impl TryFrom<u8> for CustomAction {
    type Error = ();
    fn try_from(a: u8) -> Result<Self, Self::Error> {
        Ok(match a {
            1 => CustomAction::QwertyLed,
            2 => CustomAction::ColemakLed,
            3 => CustomAction::LayerSelectLed,
            4 => CustomAction::NumpadLed,
            5 => CustomAction::NavLed,
            6 => CustomAction::SymLed,
            _ => return Err(()),
        })
    }
}

impl From<CustomAction> for u8 {
    fn from(action: CustomAction) -> Self {
        match action {
            CustomAction::QwertyLed => 1,
            CustomAction::ColemakLed => 2,
            CustomAction::LayerSelectLed => 3,
            CustomAction::NumpadLed => 4,
            CustomAction::NavLed => 5,
            CustomAction::SymLed => 6,
            _ => panic!("only led codes can serialize to integer"),
        }
    }
}

pub(crate) type Action = action::Action<CustomAction>;
macro_rules! ma {
    [$ca:expr, $($action:expr),+$(,)?] => {
        {
            const ARR: &'static [Action] = &[Action::Custom($ca), $($action,)+];
            Action::MultipleActions(&ARR)
        }
    };
}

// Colemak DH
const COLEMAK: Action = ma![CustomAction::ColemakLed, d(0)];
const QWERTY: Action = ma![CustomAction::QwertyLed, d(1)];
const LAYER: Action = ma![CustomAction::LayerSelectLed, l(2)];
const NUM_PAD: Action = ma![CustomAction::NumpadLed, d(3)];
const NAV: Action = ma![CustomAction::NavLed, d(4)];
const SYM: Action = ma![CustomAction::SymLed, l(5)];
const FUNC: Action = ma![CustomAction::FunctionLed, l(6)];

const MP: Action = Action::Custom(CustomAction::MediaPlayPause);
const MN: Action = Action::Custom(CustomAction::MediaNext);
const MB: Action = Action::Custom(CustomAction::MediaBack);
const VM: Action = Action::Custom(CustomAction::MediaMute);
const VU: Action = Action::Custom(CustomAction::MediaVolumeUp);
const VD: Action = Action::Custom(CustomAction::MediaVolumeDown);

/// The number of columns on the keyboard.
pub const NCOLS: usize = 16;

/// The number of rows on the keyboard.
pub const NROWS: usize = 4;

pub const NLAYERS: usize = 7;

pub type Layout = keyberon::layout::Layout<NCOLS, NROWS, NLAYERS, CustomAction>;

pub static LAYERS: keyberon::layout::Layers<NCOLS, NROWS, NLAYERS, CustomAction> = {
    use keyberon::action::{k, m};
    use keyberon::key_code::KeyCode::{
        Equal, Kb9, LAlt, LBracket, LCtrl, LGui, LShift, Minus, Quote, RAlt, RCtrl, RGui, RShift,
        SColon, A, D, E, F, I, J, K, L, N, O, R, S, T,
    };
    use keyberon::layout::{Event, StackedIter, WaitingAction};

    fn left_mod(stacked_iter: StackedIter) -> Option<WaitingAction> {
        match stacked_iter.map(|s| s.event()).find(|e| e.is_press()) {
            Some(Event::Press(_, j)) if j < (NCOLS as u8 / 2) => Some(WaitingAction::Tap),
            _ => None,
        }
    }

    fn right_mod(stacked_iter: StackedIter) -> Option<WaitingAction> {
        match stacked_iter.map(|s| s.event()).find(|e| e.is_press()) {
            Some(Event::Press(_, j)) if j >= (NCOLS as u8 / 2) => Some(WaitingAction::Tap),
            _ => None,
        }
    }

    macro_rules! home_mod {
        ($tap:expr, $hold:expr, $mod_func:expr) => {{
            Action::HoldTap(&HoldTapAction {
                timeout: 250,
                hold: $hold,
                tap: $tap,
                config: ::keyberon::action::HoldTapConfig::Custom($mod_func),
                tap_hold_interval: 0,
            })
        }};
    }

    // Colemak home row mods
    const A_SFT: Action = home_mod!(k(A), k(LShift), left_mod);
    const R_CTL: Action = home_mod!(k(R), k(LCtrl), left_mod);
    const S_ALT: Action = home_mod!(k(S), k(LAlt), left_mod);
    const T_GUI: Action = home_mod!(k(T), k(LGui), left_mod);

    const N_GUI: Action = home_mod!(k(N), k(RGui), right_mod);
    const E_ALT: Action = home_mod!(k(E), k(RAlt), right_mod);
    const I_CTL: Action = home_mod!(k(I), k(RCtrl), right_mod);
    const O_SFT: Action = home_mod!(k(O), k(RShift), right_mod);

    // Qwerty home row mods
    // const A_SFT: Action = home_mod!(k(A), k(LShift), left_mod);
    const S_CTL: Action = home_mod!(k(S), k(LCtrl), left_mod);
    const D_ALT: Action = home_mod!(k(D), k(LAlt), left_mod);
    const F_GUI: Action = home_mod!(k(F), k(LGui), left_mod);

    const J_GUI: Action = home_mod!(k(J), k(RGui), right_mod);
    const K_ALT: Action = home_mod!(k(K), k(RAlt), right_mod);
    const L_CTL: Action = home_mod!(k(L), k(RCtrl), right_mod);
    const SC_SFT: Action = home_mod!(k(SColon), k(RShift), right_mod);

    // Symbol home row mods
    const EQ_SFT: Action = home_mod!(k(Equal), k(LShift), left_mod);
    const LMIN: &[KeyCode] = &[LShift, Minus];
    const US_CTL: Action = home_mod!(m(&LMIN), k(LCtrl), left_mod);
    const MN_ALT: Action = home_mod!(k(Minus), k(LAlt), left_mod);
    const LEQ: &[KeyCode] = &[LShift, Equal];
    const PL_GUI: Action = home_mod!(m(&LEQ), k(LGui), left_mod);

    const L9: &[KeyCode] = &[LShift, Kb9];
    const LP_GUI: Action = home_mod!(m(&L9), k(RGui), right_mod);
    const LBRACK: &[KeyCode] = &[LShift, LBracket];
    const LB_ALT: Action = home_mod!(m(&LBRACK), k(RAlt), right_mod);
    const LS_CTL: Action = home_mod!(k(LBracket), k(RCtrl), right_mod);
    const QT_SFT: Action = home_mod!(k(Quote), k(RShift), right_mod);
    const C_LCK: Action = Action::KeyCode(KeyCode::CapsLock);
    const N_LCK: Action = Action::KeyCode(KeyCode::NumLock);
    const S_LCK: Action = Action::KeyCode(KeyCode::ScrollLock);

    // Physical layout
    // ******       ******
    // ******       ******
    // ********   ********
    //  ABC****   ****ABC
    // Where * is a key, A is rotaty AntiClockwise, C is rotary Clockwise and B is rotary button.

    keyberon::layout::layout! {
        { // (0) Colemak Dh
            [Escape Q      W      F      P      B      n      n          n      n      J       L      U      Y      ;      -     ]
            [Tab   {A_SFT}{R_CTL}{S_ALT}{T_GUI} G      n      n          n      n      M      {N_GUI}{E_ALT}{I_CTL}{O_SFT} Quote ]
            [Grave  Z      X      C      D      V     {C_LCK} LGui       RGui  {S_LCK} K       H      ,      .      /      Bslash]
            [n     {MB}   {MP}   {MN}   {SYM}   BSpace Space {LAYER}    {LAYER} Enter  Delete {SYM}  {VD}   {VM}   {VU}    n     ]
        }
        { // (1) Qwerty
            [Escape Q      W      E      R      T      n      n          n      n      Y       U      I      O      P       -     ]
            [Tab   {A_SFT}{S_CTL}{D_ALT}{F_GUI} G      n      n          n      n      H      {J_GUI}{K_ALT}{L_CTL}{SC_SFT} Quote ]
            [Grave  Z      X      C      V      B     {C_LCK} LGui       RGui  {S_LCK} N       M      ,      .      /       Bslash]
            [n     {MB}   {MP}   {MN}   {SYM}   BSpace Space {LAYER}    {LAYER} Enter  Delete {SYM}  {VD}   {VM}   {VU}     n     ]
        }
        { // (2) Layer Selector
            [t  t       t    t        t        t n  n          n      n  t  t        t        t    t       t]
            [t {QWERTY}{NAV}{NUM_PAD}{COLEMAK} t n  n          n      n  t {COLEMAK}{NUM_PAD}{NAV}{QWERTY} t]
            [t  t       t    t       {FUNC}    t t  t          t      t  t {FUNC}    t        t    t       t]
            [n  t       t    t       {SYM}     t t {LAYER}    {LAYER} t  t {SYM}     t        t    t       n]
        }
        { // (3) Numpad
            [Escape t      t       t    t    t      n      n          n      n      {N_LCK}     Kp7  Kp8 Kp9   KpMinus t      ]
            [Tab    LShift LCtrl   LAlt LGui t      n      n          n      n       KpSlash    Kp4  Kp5 Kp6   KpPlus  t      ]
            [t      t      t       t    t    t      t      t          t      t       KpAsterisk Kp1  Kp2 Kp3   KpDot   KpEnter]
            [n      t      NumLock t   {SYM} BSpace Space {LAYER}    {LAYER} KpEnter Kp0       {SYM}{VD}{VM}  {VU}     n      ]
        }
        { // (4) Nav
            [Escape Pause      PgUp  Up     PgDown t      n      n          n      n     t       t    t    t     t      t]
            [Tab    PScreen    Left  Down   Right  t      n      n          n      n     t       RGui RAlt RCtrl RShift t]
            [t      ScrollLock Home  Insert End    t      1      2          5      6     t       t    t    t     t      t]
            [n      Left       t     Right {SYM}   BSpace Space {LAYER}    {LAYER} Enter Delete {SYM}{VD} {VM}  {VU}    n]
        }
        { // (5) Symbol
            [Escape 1       2       3       4       5      n     n    n n      6      7       8       9       0       t]
            [Tab   {EQ_SFT}{US_CTL}{MN_ALT}{PL_GUI} t      n     n    n n      t     {LP_GUI}{LB_ALT}{LS_CTL}{QT_SFT} t]
            [n      ~       Grave   |       Bslash  t      t     t    t t      t     ')'     '}'     ']'     '"'      t]
            [n      t       t       t       t       BSpace Space t    t Enter  Delete t       t       t       t       n]
        }
        { // (6) Function
            [Escape F1     F2    F3   F4   F5     n     n    n n     F6     F7   F8   F9    F10    t]
            [Tab    LShift LCtrl LAlt LGui F11    n     n    n n     F12    RGui RAlt RCtrl RShift t]
            [t      t      t     t    t    t      t     t    t t     t      t    t    t     t      t]
            [n      t      t     t    t    BSpace Space t    t Enter Delete t    t    t     t      n]
        }
    }
};

#[cfg(test)]
mod layout_tests;
