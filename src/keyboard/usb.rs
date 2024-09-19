use core::sync::atomic::{AtomicBool, Ordering};
use embassy_futures::join::{self, join};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver as UsbDriver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as SerialState};
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State as UsbState};
use embassy_usb::control::OutResponse;
use embassy_usb::Handler;
use embassy_usb::{Builder as UsbBuilder, Config as UsbConfig};
use futures::future;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use {defmt_rtt as _, panic_probe as _};

use crate::{Irqs, KeyboardInput, Mode};

pub struct PlanckRequestHandler {}

impl RequestHandler for PlanckRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        log::info!("Got report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        log::info!("Set report for {:?}: {:?}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        log::info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        log::info!("Get idle rate for {:?}", id);
        None
    }
}

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

pub struct UsbSystem {}
impl UsbSystem {
    pub async fn init<'a>(
        usb_peripheral: USB,
        serial_logging: bool,
        input: &mut impl KeyboardInput,
        irqs: Irqs,
    ) {
        // usb
        let usb_driver = UsbDriver::new(usb_peripheral, irqs);
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
        let mut device_handler = PlanckUsbDeviceHandler::new();
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
        let hid_future = if input.mode() == Mode::UsbHid {
            let hid_config = embassy_usb::class::hid::Config {
                report_descriptor: KeyboardReport::desc(),
                request_handler: None,
                poll_ms: 60,
                max_packet_size: 64,
            };
            let hid = HidReaderWriter::<_, 1, 8>::new(&mut usb_builder, &mut usb_state, hid_config);

            let mut request_handler = PlanckRequestHandler {};
            let (reader, mut writer) = hid.split();
            let in_fut = async {
                loop {
                    if let Some(report) = input.scan().await {
                        // Send the report.
                        match writer.write_serialize(report).await {
                            Ok(()) => {}
                            Err(e) => log::info!("Failed to send report: {:?}", e),
                        };
                    }
                }
            };
            let out_fut = async {
                reader.run(false, &mut request_handler).await;
            };

            join(in_fut, out_fut).await;

            future::ready(())
        } else {
            future::ready(())
        };

        // serial usb
        let log_future = if serial_logging {
            let logger_class = CdcAcmClass::new(&mut usb_builder, &mut logger_state, 64);
            embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class).await;
            future::ready(())
        } else {
            future::ready(())
        };

        let mut usb_device = usb_builder.build();
        let usb_task = {
            usb_device.run().await;
            future::ready(())
        };

        join(usb_task, join(log_future, hid_future)).await;
    }
}
