use keyberon::action::{self, d, l};

pub enum CustomAction {
    BootReset,
    QwertyLed,
    ColemakLed,
    LayerSelectLed,
    NumpadLed,
    NavLed,
    SymLed,
    MmoLed,
}

pub(crate) type Action = action::Action<CustomAction>;
macro_rules! ma {
    [$ca:expr, $($action:expr),+$(,)?] => {
        Action::MultipleActions(&[Action::Custom($ca), $($action,)+])
    };
}

fn ma(actions: &'static [Action]) -> Action {
    Action::MultipleActions(actions)
}

const UF2: Action = Action::Custom(CustomAction::BootReset);

// Colemak DH
const COLEMAK: Action = ma![CustomAction::ColemakLed, d(0)];
const QWERTY: Action = ma![CustomAction::QwertyLed, d(1)];
const LAYER: Action = ma![CustomAction::LayerSelectLed, l(2)];
const NUM_PAD: Action = ma![CustomAction::NumpadLed, d(3)];
const NAV: Action = ma![CustomAction::NavLed, d(4)];
const SYM: Action = ma![CustomAction::SymLed, l(5)];
const MMO: Action = ma![CustomAction::MmoLed, d(6)];

#[cfg(feature = "home-mods")]
pub(crate) static LAYERS: keyberon::layout::Layers<CustomAction> = {
    use keyberon::action::{k, m};
    use keyberon::key_code::KeyCode::{
        Equal, Kb9, LAlt, LBracket, LCtrl, LGui, LShift, Minus, Quote, RAlt, RCtrl, RGui, RShift,
        SColon, A, D, E, F, I, J, K, L, N, O, R, S, T,
    };
    use keyberon::layout::{Event, Stack, WaitingAction};

    fn left_mod(stack: &Stack) -> Option<WaitingAction> {
        match stack.iter().map(|s| s.event()).find(|e| e.is_press()) {
            Some(Event::Press(_, j)) if j < 6 => Some(WaitingAction::Tap),
            _ => None,
        }
    }

    fn right_mod(stack: &Stack) -> Option<WaitingAction> {
        match stack.iter().map(|s| s.event()).find(|e| e.is_press()) {
            Some(Event::Press(_, j)) if j > 5 => Some(WaitingAction::Tap),
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
    const A_SFT: Action = home_mod!(&k(A), &k(LShift), &left_mod);
    const R_CTL: Action = home_mod!(&k(R), &k(LCtrl), &left_mod);
    const S_ALT: Action = home_mod!(&k(S), &k(LAlt), &left_mod);
    const T_GUI: Action = home_mod!(&k(T), &k(LGui), &left_mod);

    const N_GUI: Action = home_mod!(&k(N), &k(RGui), &right_mod);
    const E_ALT: Action = home_mod!(&k(E), &k(RAlt), &right_mod);
    const I_CTL: Action = home_mod!(&k(I), &k(RCtrl), &right_mod);
    const O_SFT: Action = home_mod!(&k(O), &k(RShift), &right_mod);

    // Qwerty home row mods
    // const A_SFT: Action = home_mod!(&k(A), &k(LShift), &left_mod);
    const S_CTL: Action = home_mod!(&k(S), &k(LCtrl), &left_mod);
    const D_ALT: Action = home_mod!(&k(D), &k(LAlt), &left_mod);
    const F_GUI: Action = home_mod!(&k(F), &k(LGui), &left_mod);

    const J_GUI: Action = home_mod!(&k(J), &k(RGui), &right_mod);
    const K_ALT: Action = home_mod!(&k(K), &k(RAlt), &right_mod);
    const L_CTL: Action = home_mod!(&k(L), &k(RCtrl), &right_mod);
    const SC_SFT: Action = home_mod!(&k(SColon), &k(RShift), &right_mod);

    // Symbol home row mods
    const EQ_SFT: Action = home_mod!(&k(Equal), &k(LShift), &left_mod);
    const US_CTL: Action = home_mod!(&m(&[LShift, Minus]), &k(LCtrl), &left_mod);
    const MN_ALT: Action = home_mod!(&k(Minus), &k(LAlt), &left_mod);
    const PL_GUI: Action = home_mod!(&m(&[LShift, Equal]), &k(LGui), &left_mod);

    const LP_GUI: Action = home_mod!(&m(&[LShift, Kb9]), &k(RGui), &right_mod);
    const LB_ALT: Action = home_mod!(&m(&[LShift, LBracket]), &k(RAlt), &right_mod);
    const LS_CTL: Action = home_mod!(&k(LBracket), &k(RCtrl), &right_mod);
    const QT_SFT: Action = home_mod!(&k(Quote), &k(RShift), &right_mod);

    keyberon::layout::layout! {
        { // (0) Colemak Dh
            [t t Escape Q      W      F        P      B       J      L      U      Y          ;      -     t t]
            [t t Tab   {A_SFT}{R_CTL}{S_ALT}  {T_GUI} G       M     {N_GUI}{E_ALT}{I_CTL}    {O_SFT} Quote t t]
            [t t t      Z      X      C        D      V       K      H      ,      .          /      t     t t]
            [t t n      n      t      CapsLock P      Space   Enter  Delete t      ScrollLock n      n     t t]
        }
        { // (1) Qwerty
            [t t Escape Q      W      E        R      T         Y      U      I      O          P       -     t t]
            [t t Tab   {A_SFT}{S_CTL}{D_ALT}  {F_GUI} G         H     {J_GUI}{K_ALT}{L_CTL}    {SC_SFT} Quote t t]
            [t t t      Z      X      C        V      B         N      M      ,      .          /       t     t t]
            [t t n      n      t      CapsLock BSpace Space     Enter  Delete t      ScrollLock n       n     t t]
        }
        { // (2) Left Layer Selector
            [t t  n        n     n      n         n        n        n  n         n         n      n     n       t t]
            [t t {QWERTY} {MMO} {NAV}  {NUM_PAD} {COLEMAK} n        n {COLEMAK} {NUM_PAD} {NAV}  {MMO} {QWERTY} t t]
            [t t  n        n     n      n         n        n        n  n         n         n      n     n       t t]
            [t t  n        n     n      n         n        n        n  n         n         n      n     n       t t]
        }
        { // (3) Numpad
            [t t t t      t       t    t      t         t       Kp7     Kp8     Kp9        KpMinus t t t]
            [t t t LShift LCtrl   LAlt LGui   t         t       Kp4     Kp5     Kp6        KpPlus  t t t]
            [t t t t      t       t    t      t         t       Kp1     Kp2     Kp3        KpEnter t t t]
            [t t n n      NumLock t    BSpace Space     KpEnter Kp0     Kp0     KpDot      n       n t t]
        }
        { // (4) Nav
            [t t t      Pause      PgUp  Up     PgDown t        t       t      t    t          t      t t t]
            [t t t      PScreen    Left  Down   Right  t        t       RGui   RAlt RCtrl      RShift t t t]
            [t t t      ScrollLock Home  Insert End    t        t       t      t    t          t      t t t]
            [t t n      n          t     t      BSpace Space    Enter   Delete t    ScrollLock n      n t t]
        }
        { // (5) Symbol
            [t t n     1       2       3       4       5       6      7       8       9       0       n t t]
            [t t n    {EQ_SFT}{US_CTL}{MN_ALT}{PL_GUI} n       n     {LP_GUI}{LB_ALT}{LS_CTL}{QT_SFT} n t t]
            [t t n     ~      '`'      |       Bslash  n       n     ')'     '}'     ']'     '"'      n t t]
            [t t n     n       t       t       BSpace  Space   Enter  Delete  t       t       n       n t t]
        }
        { // (6) MMO
            [t t N      1  2    3      4      5        n       Kp7   Kp8     Kp9        KpMinus n t t]
            [t t Tab    I  A    W      D      =        n       Kp4   Kp5     Kp6        KpPlus  n t t]
            [t t B      6  7    8      9      0        n       Kp1   Kp2     Kp3        KpEnter n t t]
            [t t n      n  LAlt LCtrl LShift Space     Kp0     RCtrl RAlt    RShift     n       n t t]
        }
    }
};

#[cfg(not(feature = "home-mods"))]
pub(crate) static LAYERS: keyberon::layout::Layers<CustomAction> = {
    keyberon::layout::layout! {
        { // (0) Colemak Dh
            [Escape   Q  W     F    P      B         J      L      U    Y     ;   -     ]
            [Tab      A  R     S    T      G         M      N      E    I     O   Quote ]
            [LShift   Z  X     C    D      V         K      H      ,    .     /   RShift]
            [n        n  LCtrl LAlt BSpace Space     Enter  Delete RAlt RCtrl n   n     ]
            [n        n  t    {SYM} LGui  {LAYER}   {LAYER} RGui  {SYM} t     n   n     ]
        }
        { // (1) Qwerty
            [Escape   Q  W     E    R      T         Y      U      I    O     P   -     ]
            [Tab      A  S     D    F      G         H      J      K    L     ;   Quote ]
            [LShift   Z  X     C    V      B         N      M      ,    .     /   RShift]
            [n        n  LCtrl LAlt BSpace Space     Enter  Delete RAlt RCtrl n   n     ]
            [n        n  t    {SYM} LGui  {LAYER}   {LAYER} RGui  {SYM} t     n   n     ]
        }
        { // (2) Left Layer Selector
            [ n        n     n      n         n        n        n  n         n         n      n     n      ]
            [{QWERTY} {MMO} {NAV}  {NUM_PAD} {COLEMAK} n        n {COLEMAK} {NUM_PAD} {NAV}  {MMO} {QWERTY}]
            [ n        n     n      n         n        n        n  n         n         n      n     n      ]
            [ n        n     n      n         n        n        n  n         n         n      n     n      ]
            [ n        n     n      n         n        n        n  n         n         n      n     n      ]
        }
        { // (3) Numpad
            [t      t t     t     t      t         t       Kp7     Kp8     Kp9        KpMinus t     ]
            [t      t t     t     t      t         t       Kp4     Kp5     Kp6        KpPlus  t     ]
            [LShift t t     t     t      t         t       Kp1     Kp2     Kp3        KpEnter RShift]
            [n      n LCtrl LAlt  BSpace Space     KpEnter Kp0     Kp0     KpDot      n       n     ]
            [n      n t    {SYM}  LGui  {LAYER}   {LAYER}  RGui   {SYM}    t          n       n     ]
        }
        { // (4) Nav
            [t      Pause      PgUp  Up     PgDown t        t       t      t    t     t t     ]
            [t      PScreen    Left  Down   Right  t        t       t      t    t     t t     ]
            [LShift ScrollLock Home  Insert End    t        t       t      t    t     t RShift]
            [n      n          LCtrl LAlt   BSpace Space    Enter   Delete RAlt RCtrl n n     ]
            [n      n          t    {SYM}   LGui  {LAYER}  {LAYER}  RGui  {SYM} t     n n     ]
        }
        { // (5) Symbol
            [n      1  2     3    4      5       6      7      8    9     0 n     ]
            [n      = '_'    -    +      n       n     '('    '{'  '['    n n     ]
            [LShift ~ '`'    |    Bslash n       n     ')'    '}'  ']'    n RShift]
            [n      n  LCtrl LAlt BSpace Space   Enter  Delete RAlt RCtrl n n     ]
            [n      n  t     n    LGui  {LAYER} {LAYER} RGui   n    t     n n     ]
        }
        { // (6) MMO
            [N      1  2    3      4      5        n       Kp7   Kp8  Kp9        KpMinus n]
            [Tab    I  A    W      D      =        n       Kp4   Kp5  Kp6        KpPlus  n]
            [B      6  7    8      9      0        n       Kp1   Kp2  Kp3        KpEnter n]
            [n      n  LAlt LCtrl LShift Space     Kp0     RCtrl RAlt RShift     n       n]
            [n      n  t   {SYM}  LGui  {LAYER}   {LAYER}  RGui {SYM} t          n       n]
        }
    }
};
