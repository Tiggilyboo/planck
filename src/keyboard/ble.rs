use bt_hci::controller::{Controller, ExternalController};
use cyw43_pio::PioSpi;
use embassy_rp::{gpio::Output, peripherals::DMA_CH0, peripherals::PIO0};

use crate::{Irqs, KeyboardInput, KeyboardOutput};

#[embassy_executor::task]
pub async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

pub struct BleSystem<C> {
    controller: C,
}

impl<C> BleSystem<C>
where
    C: Controller,
{
    pub async fn init(controller: C) -> Self {
        Self { controller }
    }
}

impl<C> KeyboardOutput for BleSystem<C>
where
    C: Controller,
{
    async fn run(&mut self) {}
}
