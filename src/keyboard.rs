use usbd_hid::descriptor::KeyboardReport;
use embassy::gpio::{Input, Output};
use embassy_time::{Duration, Instant, Timer};

mod keycodes;
use crate::keycodes::*;

macro_rules! define_keymap {
    ($cols:expr, $rows:expr, $layers:expr, $($key:ident),*) => {{
        let mut matrix = [[[KeyCode::No; $cols]; $rows]; $layers];
        let mut i = 0;
        $(
            let mut ri = 0;
            $(
                let mut ci = 0;
                $(
                    matrix[i][r][c] = KeyCode::$key;
                    ci += 1;
                )*
                ri += 1;
            )*
            i += 1;
        )*

        matrix
    }}
}

pub struct Keyboard<const COLS: usize, const ROWS: usize, const LAYERS: usize> {
    inputs: [Input; COLS],
    outputs: [Output; ROWS],
    keymap: [[[KeyCode; COLS]; ROWS]; LAYERS],
    report: KeyboardReport,
    current_layer: usize,
}

impl <const COLS: usize, const ROWS: usize, const LAYERS: usize> Keyboard<ROWS, COLS, LAYERS> {
    pub fn new(
        inputs: [Input; COLS], 
        outputs: [Output; ROWS], 
        keymap: [[[KeyCode; COLS]; ROWS]; LAYERS]
    ) -> Self {
        Self {
            inputs,
            outputs,
            keymap: [[[KeyCode; COLS]; ROWS]; LAYERS],
            report: KeyboardReport {
                modifier: 0,
                reserved: 0,
                leds: 0,
                keycodes: [0; 6],
            },
            current_layer: usize,
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

    pub async fn scan(&mut self) {
        // Update matrix state
        for (out_index, output) in self.outputs.iter_mut().enumerate() {

            // Pull up output, wait 1us for change
            output.set_high().ok();
            Timer::after_micros(1).await;

            for(in_index, input) in self.inputs.iter_mut().enummerate() {
                let high = input.is_high().ok().unwrap_or_default();
                let key = self.keymap[self.current_layer][out_index][in_index];

                if high {
                    if key.is_modifier() {
                        self.report.modifier |= key.as_modifier_bit();
                    } else if let Some(i) = self.report.keycodes.iter().position(|&k| k == 0) {
                        self.report.keycodes[i] = key as u8
                    }
                } else {
                    if key.is_modifier() {
                        self.report.modifier &= !key.as_modifier_bit();
                    } else if let Some(i) = self.report.keycodes.iter().position(|&k| k == key as u8) {
                        self.report.keycodes[i] = 0;
                    }
                }
            }

            output.set_low().ok();
        }
    }
}
