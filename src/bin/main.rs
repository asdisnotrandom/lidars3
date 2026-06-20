#![no_std]
#![no_main]

use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts, peripherals,
    uart::{self, BufferedUartRx, Config as UartConfig},
    usb::{Driver, InterruptHandler as UsbInterruptHandler},
};
use static_cell::StaticCell;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    Builder, Config as UsbConfig, UsbDevice,
};
use embedded_io_async::Write;

use lidar_driver::{LidarMap, LidarS3};

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<peripherals::UART0>;
    USBCTRL_IRQ => UsbInterruptHandler<peripherals::USB>;
});

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, peripherals::USB>>) -> ! {
    usb.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);
    let mut usb_config = UsbConfig::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("Hydronom");
    usb_config.product = Some("Lidar Otonom Modulu");
    usb_config.serial_number = Some("12345678");
    usb_config.max_power = 100;

    static STATE: StaticCell<State> = StaticCell::new();
    static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        DEVICE_DESCRIPTOR.init([0; 256]),
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let usb = builder.build();
    
    spawner.spawn(usb_task(usb).unwrap());

    let (mut usb_tx, mut _usb_rx) = class.split();

    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;

    let uart_rx = BufferedUartRx::new(p.UART0, Irqs, p.PIN_1, RX_BUF.init([0; 1024]), uart_config);
    let mut lidar = LidarS3::new(uart_rx);
    let mut lidar_map = LidarMap::new();

    loop {
        usb_tx.wait_connection().await;

        loop {
            match lidar.read_new_data().await {
                Ok(data) => {
                    lidar_map.update_points(data);

                    let angle_u16 = (data.angle_dg * 10.0) as u16; 
                    let dist_u16 = data.mm_dist as u16;

                    let payload: [u8; 6] = [
                        if data.start_flag { 0x01 } else { 0x00 },
                        (angle_u16 >> 8) as u8, (angle_u16 & 0xFF) as u8,
                        (dist_u16 >> 8) as u8,  (dist_u16 & 0xFF) as u8,
                        data.quality
                    ];

                    if let Err(_) = usb_tx.write_all(&payload).await {
                        break; 
                    }
                }
                Err(_) => {}
            }
        }
    }
}