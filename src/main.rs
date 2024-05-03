#![no_std]
#![no_main]

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod request_handler;
mod device_handler;
mod keyboard;

use keyboard::*;
use request_handler::*;
use device_handler::*;

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

/*
Planck 12 column x 4 rows
Pinout:
    Columns (Left to right): 
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, UsbIrqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Simon Willshire");
    config.product = Some("Planck RP2040");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = PlanckRequestHandler {};
    let mut device_handler = PlanckDeviceHandler::new();
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);
    let mut usb = builder.build();
    let usb_fut = usb.run();

    let mut col_1 = Input::new(p.PIN_2, Pull::None);
    col_1.set_schmitt(true);

    let (reader, mut writer) = hid.split();

    let inputs = [
        Input::new(p.PIN_2, Pull::None),
        Input::new(p.PIN_3, Pull::None),
        Input::new(p.PIN_4, Pull::None),
        Input::new(p.PIN_5, Pull::None),
        Input::new(p.PIN_6, Pull::None),
        Input::new(p.PIN_7, Pull::None),
        Input::new(p.PIN_8, Pull::None),
        Input::new(p.PIN_9, Pull::None),
        Input::new(p.PIN_10, Pull::None),
        Input::new(p.PIN_11, Pull::None),
        Input::new(p.PIN_12, Pull::None),
        Input::new(p.PIN_13, Pull::None),        
    ];
    let outputs = [
        Output::new(p.PIN_18, Level::Low),
        Output::new(p.PIN_19, Level::Low),
        Output::new(p.PIN_20, Level::Low),
        Output::new(p.PIN_21, Level::Low),
    ];
    let keymap = define_keymap!(inputs.len(), outputs.len(), 3, (
        
    ));
    let keyboard = Keyboard::<12, 4, 3>::new(inputs, outputs, keymap);    

    let in_fut = async {
        loop {
            info!("Waiting for HIGH on pin 4");
            col_1.wait_for_high().await;
            info!("HIGH DETECTED");

            // Create a report with the A key pressed. (no shift modifier)
            let report = KeyboardReport {
                keycodes: [4, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            // Send the report.
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
            col_1.wait_for_low().await;
            info!("LOW DETECTED");
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
        }
    };

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, join(in_fut, out_fut)).await;
}
