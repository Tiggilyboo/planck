use core::sync::atomic::{AtomicBool, Ordering};

use embassy_usb::Handler;
use {defmt_rtt as _, panic_probe as _};

pub struct PlanckUsbDeviceHandler {
    configured: AtomicBool,
}

impl PlanckUsbDeviceHandler {
    pub fn new() -> Self {
        Self {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for PlanckUsbDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            log::info!("Device enabled");
        } else {
            log::info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        log::info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        log::info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            log::info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            log::info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
