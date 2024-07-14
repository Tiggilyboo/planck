use super::KeyCode;

#[derive(Copy, Clone)]
pub struct KeyState {
    pub changed: bool,
    pub pressed: bool,
    key_code: KeyCode,
    debounce_counter: u16,
}


impl Default for KeyState {
    fn default() -> Self {
        Self::new(KeyCode::No)
    }
}

impl KeyState {
    pub fn new(kc: KeyCode) -> Self {
        Self {
            key_code: kc,
            changed: false,
            pressed: false,
            debounce_counter: 0,
        }
    }

    pub fn toggle_pressed(&mut self) {
        self.pressed = !self.pressed;
    }

    pub fn get_key_code(&self) -> KeyCode {
        self.key_code
    }

    pub fn set_key_code(&mut self, kc: KeyCode) {
        self.key_code = kc;
    }

    fn debounce_counter_add(&mut self, delta_ms: u16) {
        if u16::MAX - self.debounce_counter <= delta_ms {
            self.debounce_counter = u16::MAX;
        } else {
            self.debounce_counter += 1;
        }
    }

    fn debounce_counter_sub(&mut self, delta_ms: u16) {
        if delta_ms > self.debounce_counter {
            self.debounce_counter = 0;
        } else {
            self.debounce_counter -= 1;
        }
    }

    pub fn detect_change(&mut self, pin_state: bool, delta_ms: u16) -> bool {
        if delta_ms > 0 {
            if self.pressed == pin_state {
                self.debounce_counter_sub(delta_ms);
            } else {
                if self.debounce_counter < 100 {
                    self.debounce_counter_add(delta_ms);
                } else {
                    self.debounce_counter = 0;
                    return true;
                }
            }
        }

        return false;
    }
}
