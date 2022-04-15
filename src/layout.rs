use keyberon::action::{self, d, l};

#[allow(clippy::enum_variant_names)]
#[derive(Copy, Clone)]
pub enum CustomAction {
    QwertyLed,
    ColemakLed,
    LayerSelectLed,
    NumpadLed,
    NavLed,
    SymLed,
}

impl From<u8> for CustomAction {
    fn from(a: u8) -> Self {
        match a {
            0 => CustomAction::QwertyLed,
            1 => CustomAction::ColemakLed,
            2 => CustomAction::LayerSelectLed,
            3 => CustomAction::NumpadLed,
            4 => CustomAction::NavLed,
            5 => CustomAction::SymLed,
            _ => panic!("unexpected number {a}"),
        }
    }
}

impl From<CustomAction> for u8 {
    fn from(action: CustomAction) -> Self {
        match action {
            CustomAction::QwertyLed => 0,
            CustomAction::ColemakLed => 1,
            CustomAction::LayerSelectLed => 2,
            CustomAction::NumpadLed => 3,
            CustomAction::NavLed => 4,
            CustomAction::SymLed => 5,
        }
    }
}

pub(crate) type Action = action::Action<CustomAction>;
macro_rules! ma {
    [$ca:expr, $($action:expr),+$(,)?] => {
        Action::MultipleActions(&[Action::Custom($ca), $($action,)+])
    };
}

// Colemak DH
const COLEMAK: Action = ma![CustomAction::ColemakLed, d(0)];
const QWERTY: Action = ma![CustomAction::QwertyLed, d(1)];
const LAYER: Action = ma![CustomAction::LayerSelectLed, l(2)];
const NUM_PAD: Action = ma![CustomAction::NumpadLed, d(3)];
const NAV: Action = ma![CustomAction::NavLed, d(4)];
const SYM: Action = ma![CustomAction::SymLed, l(5)];

/// The number of columns on the keyboard.
pub(crate) const NCOLS: usize = 16;

/// The number of rows on the keyboard.
pub(crate) const NROWS: usize = 4;

pub(crate) const NLAYERS: usize = 6;

pub(crate) static LAYERS: keyberon::layout::Layers<NCOLS, NROWS, NLAYERS, CustomAction> = {
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
        ($tap:expr, $hold:expr, $mod_func:expr) => {
            Action::HoldTap {
                timeout: 250,
                hold: $hold,
                tap: $tap,
                config: ::keyberon::action::HoldTapConfig::Custom(
                    ::keyberon::action::CustomHandler($mod_func),
                ),
                tap_hold_interval: 0,
            }
        };
    }

    // Colemak home row mods
    const A_SFT: Action = home_mod!(&k(A), &k(LShift), left_mod);
    const R_CTL: Action = home_mod!(&k(R), &k(LCtrl), left_mod);
    const S_ALT: Action = home_mod!(&k(S), &k(LAlt), left_mod);
    const T_GUI: Action = home_mod!(&k(T), &k(LGui), left_mod);

    const N_GUI: Action = home_mod!(&k(N), &k(RGui), right_mod);
    const E_ALT: Action = home_mod!(&k(E), &k(RAlt), right_mod);
    const I_CTL: Action = home_mod!(&k(I), &k(RCtrl), right_mod);
    const O_SFT: Action = home_mod!(&k(O), &k(RShift), right_mod);

    // Qwerty home row mods
    // const A_SFT: Action = home_mod!(&k(A), &k(LShift), left_mod);
    const S_CTL: Action = home_mod!(&k(S), &k(LCtrl), left_mod);
    const D_ALT: Action = home_mod!(&k(D), &k(LAlt), left_mod);
    const F_GUI: Action = home_mod!(&k(F), &k(LGui), left_mod);

    const J_GUI: Action = home_mod!(&k(J), &k(RGui), right_mod);
    const K_ALT: Action = home_mod!(&k(K), &k(RAlt), right_mod);
    const L_CTL: Action = home_mod!(&k(L), &k(RCtrl), right_mod);
    const SC_SFT: Action = home_mod!(&k(SColon), &k(RShift), right_mod);

    // Symbol home row mods
    const EQ_SFT: Action = home_mod!(&k(Equal), &k(LShift), left_mod);
    const US_CTL: Action = home_mod!(&m(&[LShift, Minus]), &k(LCtrl), left_mod);
    const MN_ALT: Action = home_mod!(&k(Minus), &k(LAlt), left_mod);
    const PL_GUI: Action = home_mod!(&m(&[LShift, Equal]), &k(LGui), left_mod);

    const LP_GUI: Action = home_mod!(&m(&[LShift, Kb9]), &k(RGui), right_mod);
    const LB_ALT: Action = home_mod!(&m(&[LShift, LBracket]), &k(RAlt), right_mod);
    const LS_CTL: Action = home_mod!(&k(LBracket), &k(RCtrl), right_mod);
    const QT_SFT: Action = home_mod!(&k(Quote), &k(RShift), right_mod);

    keyberon::layout::layout! {
        { // (0) Colemak Dh
            [Escape Q      W        F        P      B      n    n          n      n    J      L      U      Y          ;      -    ]
            [Tab   {A_SFT}{R_CTL}  {S_ALT}  {T_GUI} G      n    n          n      n    M     {N_GUI}{E_ALT}{I_CTL}    {O_SFT} Quote]
            [t      Z      X        C        D      V      1    2          5      6    K      H      ,      .          /      t    ]
            [n      Left   CapsLock Right    BSpace Space {SYM}{LAYER}    {LAYER}{SYM} Enter  Delete Up     ScrollLock Down   n    ]
        }
        { // (1) Qwerty
            [Escape Q      W        E        R      T      n    n          n      n    Y      U      I      O          P       -    ]
            [Tab   {A_SFT}{S_CTL}  {D_ALT}  {F_GUI} G      n    n          n      n    H     {J_GUI}{K_ALT}{L_CTL}    {SC_SFT} Quote]
            [t      Z      X        C        V      B      1    2          5      6    N      M      ,      .          /       t    ]
            [n      Left   CapsLock Right    BSpace Space {SYM}{LAYER}    {LAYER}{SYM} Enter  Delete Up     ScrollLock Down    n    ]
        }
        { // (2) Left Layer Selector
            [ t  t       t    t        t        t   n    n          n      n    t  t        t        t    t       t]
            [ t {QWERTY}{NAV}{NUM_PAD}{COLEMAK} t   n    n          n      n    t {COLEMAK}{NUM_PAD}{NAV}{QWERTY} t]
            [ t  t       t    t        t        t   1    2          5      6    t  t        t        t    t       t]
            [ n  t       t    t        t        t  {SYM}{LAYER}    {LAYER}{SYM} t  t        t        t    t       n]
        }
        { // (3) Numpad
            [t t      t       t    t      t      t    t          t      t    t       Kp7     Kp8     Kp9        KpMinus t]
            [t LShift LCtrl   LAlt LGui   t      t    t          t      t    t       Kp4     Kp5     Kp6        KpPlus  t]
            [t t      t       t    t      t      1    2          5      6    t       Kp1     Kp2     Kp3        KpEnter t]
            [n t      NumLock t    BSpace Space {SYM}{LAYER}    {LAYER}{SYM} KpEnter Kp0     n       KpDot      n       n]
        }
        { // (4) Nav
            [t      Pause      PgUp  Up     PgDown t      t    t          t      t    t       t      t    t          t      t]
            [t      PScreen    Left  Down   Right  t      t    t          t      t    t       RGui   RAlt RCtrl      RShift t]
            [t      ScrollLock Home  Insert End    t      1    2          5      6    t       t      t    t          t      t]
            [n      t          t     t      BSpace Space {SYM}{LAYER}    {LAYER}{SYM} Enter   Delete t    ScrollLock t      n]
        }
        { // (5) Symbol
            [n     1       2       3       4       5     t t    t t 6      7       8       9       0       n]
            [n    {EQ_SFT}{US_CTL}{MN_ALT}{PL_GUI} n     t t    t t n     {LP_GUI}{LB_ALT}{LS_CTL}{QT_SFT} n]
            [n     ~      '`'      |       Bslash  n     t t    t t n     ')'     '}'     ']'     '"'      n]
            [n     t       t       t       BSpace  Space t t    t t Enter  Delete  t       t       t       n]
        }
    }
};
