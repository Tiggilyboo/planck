#![no_std]
#![no_main]

use ble::BleSystem;
use bt_hci::controller::ExternalController;
use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;
use static_cell::StaticCell;
use usb::UsbSystem;
use {defmt_rtt as _, panic_probe as _};

mod keyboard;
use keyboard::*;

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/*
Planck 12 column x 4 rows
Pinout:
    Columns (Right to left):
        PIN_4 (GP2),
        PIN_5 (GP3),
        PIN_6 (GP4),
        PIN_7 (GP5),
        PIN_9 (GP6),
        PIN_10 (GP7),
        PIN_11 (GP8),
        PIN_12 (GP9),
        PIN_14 (GP10),
        PIN_15 (GP11),
        PIN_16 (GP12),
        PIN_17 (GP13),

    Rows (Bottom to top):
        PIN_24 (GP18),
        PIN_25 (GP19),
        PIN_26 (GP20),
        PIN_27 (GP21),
*/
pub const NUM_COLS: usize = 12;
pub const NUM_ROWS: usize = 4;
pub const NUM_LAYERS: usize = 3;

type PlanckKeyboard<'a> = Keyboard<'a, NUM_COLS, NUM_ROWS, NUM_LAYERS>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let outputs = [
        Output::new(p.PIN_13, Level::Low),
        Output::new(p.PIN_12, Level::Low),
        Output::new(p.PIN_11, Level::Low),
        Output::new(p.PIN_10, Level::Low),
        Output::new(p.PIN_9, Level::Low),
        Output::new(p.PIN_8, Level::Low),
        Output::new(p.PIN_7, Level::Low),
        Output::new(p.PIN_6, Level::Low),
        Output::new(p.PIN_5, Level::Low),
        Output::new(p.PIN_4, Level::Low),
        Output::new(p.PIN_3, Level::Low),
        Output::new(p.PIN_2, Level::Low),
    ];

    let mut inputs = [
        Input::new(p.PIN_21, Pull::Down),
        Input::new(p.PIN_20, Pull::Down),
        Input::new(p.PIN_19, Pull::Down),
        Input::new(p.PIN_18, Pull::Down),
    ];
    for i in inputs.iter_mut() {
        // slightly debounce using schmitt trigger
        i.set_schmitt(true);
    }

    #[rustfmt::skip]
    let keymap = define_keymap!(NUM_COLS, NUM_ROWS, NUM_LAYERS, [ 
        // base
        [
            [Tab, Q, W, F, P, G, J, L, U, Y, Semicolon, Delete],
            [Escape, A, R, S, T, D, H, N, E, I, O, Quote],
            [LShift, Z, X, C, V, B, K, M, Comma, Dot, Slash, Enter],
            [LCtrl, LGui, LAlt, No, TriLayerLower, Space, Backspace, TriLayerUpper, Left, Down, UP, Right]
        ],
        // upper        
        [
            [Grave, Kc1, Kc2, Kc3, Kc4, Kc5, Kc6, Kc7, Kc8, Kc9, Kc0, Backspace],
            [Escape, Kc4, Kc5, Kc6, T, D, H, Minus, Equal, LeftBracket, RightBracket, Backslash],
            [LShift, Kc7, Kc8, Kc9, Kc0, B, K, M, Comma, Dot, Slash, Enter],
            [LCtrl, LGui, LAlt, No, TriLayerLower, Space, Backspace, TriLayerUpper, Home, PageDown, PageUp, End]
        ],
        // lower
        [
            [Grave, Kc1, Kc2, Kc3, Kc4, Kc5, Kc6, Kc7, Kc8, Kc9, Kc0, Backspace],
            [Escape, F1, F2, F3, F4, F5, F6, Minus, Equal, LeftBracket, RightBracket, Backslash],
            [LShift, F7, F8, F9, F10, F11, F12, F13, Comma, Dot, Slash, Enter],
            [LCtrl, LGui, LAlt, No, TriLayerLower, Space, Backspace, TriLayerUpper, Home, PageDown, PageUp, Right]
        ]
    ]);

    let mode = Mode::UsbHid;
    let mut keyboard = PlanckKeyboard::new(inputs, outputs, keymap, mode);

    if mode == Mode::Ble {
        let mut pwr = Output::new(p.PIN_23, Level::Low);
        let cs = Output::new(p.PIN_25, Level::High);
        let mut pio = Pio::new(p.PIO0, Irqs);
        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            pio.irq0,
            cs,
            p.PIN_24,
            p.PIN_29,
            p.DMA_CH0,
        );
        let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
        let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
        let btfw = include_bytes!("../cyw43-firmware/43439A0_btfw.bin");

        static STATE: StaticCell<cyw43::State> = StaticCell::new();
        let state = STATE.init(cyw43::State::new());
        let (_net_dev, bt_dev, mut control, runner) =
            cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;

        spawner.spawn(ble::cyw43_task(runner)).unwrap();
        control.init(clm).await;

        let bt_controller = ExternalController::<_, 10>::new(bt_dev);

        BleSystem::init(bt_controller).await;
    }
    UsbSystem::init(p.USB, true, &mut keyboard, Irqs).await;
}
