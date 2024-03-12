use rmk::action::KeyAction;
use rmk::{a, k, layer, mo};

pub(crate) const COL: usize = 12;
pub(crate) const ROW: usize = 4;
pub(crate) const NUM_LAYER: usize = 3;

#[rustfmt::skip]
pub(crate) const KEYMAP: [[[KeyAction; COL]; ROW]; NUM_LAYER] = [
    // Layer 0 (Main layer)
    layer![
        [k!(Tab), k!(Q), k!(W), k!(F), k!(P), k!(G), k!(J), k!(L), k!(U), k!(Y), k!(Semicolon), k!(Backspace)],
        [k!(Escape), k!(A), k!(R), k!(S), k!(T), k!(D), k!(H), k!(N), k!(E), k!(I), k!(O), k!(Quote)],
        [k!(LShift), k!(Z), k!(X), k!(C), k!(V), k!(B), k!(K), k!(M), k!(Comma), k!(Dot), k!(Slash), k!(Enter)]
        [a!(LCtrl), k!(LGui), k!(LAlt), k!(No), mo!(1), k!(Space), k!(Backspace), mo!(2), a!(Home), k!(PageUp), k!(PageDown), a!(End)],
    ],
    // Layer 1 (Top layer with Upper key)
    layer![
        [k!(Tilde), k!(Exclam), k!(At), k!(Hash), k!(Dollar), k!(Percent), k!(Circum), k!(Ampersand), k!(Asterisk), k!(Lparen), k!(Rparen), a!(Backspace)],
        [k!(Delete), k!(F1), k!(F2), k!(F3), k!(F4), k!(F5), k!(F6), k!(Underscore), k!(Plus), k!(LeftBracket), k!(RightBracket), k!(Pipe)],
        [k!(LShift), k!(F7), k!(F8), k!(F9), k!(F10), k!(F11), k!(F12), k!(Comma), k!(Dot), k!(Slash), k!(Enter)],
        [a!(LCtrl), k!(LGui), k!(LAlt), k!(No), k!(Space), k!(Backspace), k!(No), a!(No), a!(Home), k!(PageUp), k!(PageDown), a!(End)],
    ],
    // Layer 2 (Lower layer)
    layer![
        [k!(Grave), k!(Kp1), k!(Kp2), k!(Kp3), k!(Kp4), k!(Kp5), k!(Kp6), k!(Kp7), k!(Kp8), k!(Kp9), k!(Kp0), a!(Delete)],
        [k!(Escape), k!(Kp4), k!(Kp5), k!(Kp6), k!(No), k!(No), k!(No), k!(No), k!(No), k!(No), k!(No), a!(Backslash)],
        [k!(Delete), k!(Kp7), k!(Kp8), k!(Kp9), k!(Kp0), k!(No), k!(No), a!(No), a!(Comma), k!(Dot), k!(Slash), a!(Enter)],
        [a!(LCtrl), a!(LGui), a!(LAlt), a!(No), k!(Space), k!(Backspace), a!(No), k!(Home), k!(PageUp), k!(PageDown), k!(End)]
    ]
];

