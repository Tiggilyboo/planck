use usbd_hid::descriptor::KeyboardReport;
use embassy_rp::gpio::{Input, Output};
use embassy_time::{Duration, Instant, Timer};
use num_enum::FromPrimitive;

pub mod keycodes;
use keycodes::*;

pub mod keystate;
use keystate::*;

pub mod ble;

#[macro_export]
macro_rules! define_keymap {
    ($cols:expr, $rows:expr, $layers:expr, [$([$([$($key: ident), +]), +]), +]) => {{
        use $crate::keyboard::keycodes::KeyCode as KeyCode;
        let mut matrix = [[[KeyCode::No; $cols]; $rows]; $layers];
        let mut i = 0;
        $(
            let mut ri = 0;
            $(
                let mut ci = 0;
                $(
                    matrix[i][ri][ci] = KeyCode::$key;
                    ci += 1;
                )*
                ri += 1;
            )*
            i += 1;
        )*

        matrix
    }}
}

#[repr(u8)]
#[derive(Copy, Clone, FromPrimitive)]
pub enum Layer {
    #[default]
    Base = 0b00,
    Lower = 0b01,
    Upper = 0b10,
    Both = 0b11,
}

pub struct Keyboard<'a, const COLS: usize, const ROWS: usize, const LAYERS: usize> {
    inputs: [Input<'a>; ROWS],
    outputs: [Output<'a>; COLS],
    keys: [[[KeyState; COLS]; ROWS]; LAYERS],
    report: KeyboardReport,
    current_layer: Layer,
    last_tick: u32,
}

impl <'a, const COLS: usize, const ROWS: usize, const LAYERS: usize> Keyboard<'a, COLS, ROWS, LAYERS> {
    pub fn new(
        inputs: [Input<'a>; ROWS], 
        outputs: [Output<'a>; COLS], 
        keymap: [[[KeyCode; COLS]; ROWS]; LAYERS]
    ) -> Self {
        let mut keys: [[[KeyState; COLS]; ROWS]; LAYERS] = [[[KeyState::default(); COLS]; ROWS]; LAYERS];
        for l in 0..LAYERS {
            for r in 0..ROWS {
                for c in 0..COLS {
                    let key_code = keymap[l][r][c];
                    keys[l][r][c].set_key_code(key_code);
                    log::info!("[{}, {}, {}] = {:?}", c, r, l, key_code);
                }
            }
        }

        Keyboard::<'a, COLS, ROWS, LAYERS> {
            inputs,
            outputs,
            keys,
            report: KeyboardReport {
                modifier: 0,
                reserved: 0,
                leds: 0,
                keycodes: [0; 6],
            },
            current_layer: Layer::Base,
            last_tick: 0,
        }
    }

    pub fn get_report(&self) -> &KeyboardReport {
        &self.report
    }

    pub fn reset_report(&mut self) {
        for bit in &mut self.report.keycodes {
            *bit = 0;
        }
    }

    fn process_key_change(pressed: bool, key_code: KeyCode, layer: &mut Layer, report: &mut KeyboardReport) -> bool {
        match key_code {
            KeyCode::TriLayerLower => if pressed {
                *layer = Layer::from(*layer as u8 | Layer::Lower as u8);
                false
            } else {
                *layer = Layer::from(*layer as u8 & !(Layer::Lower as u8));
                false
            },
            KeyCode::TriLayerUpper => if pressed {
                *layer = Layer::from(*layer as u8 | Layer::Upper as u8);
                false
            } else {
                *layer = Layer::from(*layer as u8 & !(Layer::Upper as u8));
                false
            },

            // process as normal
            _ => {
                if pressed {
                    //log::info!("{:?} is down", key_code);
                    if key_code.is_modifier() {
                        report.modifier |= key_code.as_modifier_bit();
                    } else if let Some(i) = report.keycodes.iter().position(|&k| k == 0) {
                        report.keycodes[i] = key_code as u8
                    }
                } else {
                    if key_code.is_modifier() {
                        report.modifier &= !key_code.as_modifier_bit();
                    } else if let Some(i) = report.keycodes.iter().position(|&k| k == key_code as u8) {
                        report.keycodes[i] = 0;
                    }
                }
                true
            }
        }
    }

    pub async fn scan(&mut self) -> bool {
        log::info!("scan {}", self.last_tick);

        let mut send_report = false;
        let layer = &mut self.current_layer;

        // Update matrix state
        for (out_index, output) in self.outputs.iter_mut().enumerate() {

            // Pull up output, wait 1us for change
            output.set_high();
            Timer::after_micros(1).await;

            for (in_index, input) in self.inputs.iter_mut().enumerate() {
                let key_state: &mut KeyState = &mut self.keys[*layer as usize][in_index][out_index];
                let key_code = key_state.get_key_code();

                let cur_tick = Instant::now().as_ticks() as u32;
                let delta_ms = (cur_tick - self.last_tick) as u16;

                let high = input.is_high();
                let changed = key_state.detect_change(high, delta_ms);
                if changed {
                    self.last_tick = cur_tick;
                    key_state.toggle_pressed();
                }
                key_state.changed = changed;

                if changed {
                    send_report |= Self::process_key_change(high, key_code, layer, &mut self.report);
                }
            }

            output.set_low();
        }

        send_report
    }
}
