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
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::hid::{HidReaderWriter, ReportId, State as UsbState};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver as SerialReceiver, Sender as SerialSender, State as SerialState};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder as UsbBuilder, Config as UsbConfig, Handler as UsbHandler};
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // usb
    let usb_driver = UsbDriver::new(p.USB, UsbIrqs);
    let mut usb_config = UsbConfig::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("Simon Willshire");
    usb_config.product = Some("Planck RP2040");
    usb_config.serial_number = Some("12345678");
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;
    
    // Required for windows compatiblity.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = PlanckRequestHandler {};
    let mut device_handler = PlanckDeviceHandler::new();
    let mut usb_state = UsbState::new();
    let mut logger_state = SerialState::new();

    let mut usb_builder = UsbBuilder::new(
        usb_driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    usb_builder.handler(&mut device_handler);

    // Create classes on the builder.
    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut usb_builder, &mut usb_state, hid_config);

    // serial usb
    let logger_class = CdcAcmClass::new(&mut usb_builder, &mut logger_state, 64);
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);
    
    let mut usb = usb_builder.build();
    let usb_fut = usb.run();
    
    let (reader, mut writer) = hid.split();

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

    let keymap = define_keymap!(12, 4, 3, [ 
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
    let mut keyboard = Keyboard::<12, 4, 3>::new(inputs, outputs, keymap);    

    let in_fut = async {
        loop {                     
            let send_report = keyboard.scan().await;

            if send_report {          
                let report = keyboard.get_report();
            
                // Send the report.
                match writer.write_serialize(report).await {
                    Ok(()) => {}
                    Err(e) => log::info!("Failed to send report: {:?}", e),
                };

                keyboard.reset_report();
            }

            //Timer::after_secs(1).await;
        }
    };

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, join(log_fut, join(in_fut, out_fut))).await;
}
