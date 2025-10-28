use std::borrow::Borrow;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::spi::*;
use esp_idf_hal::sys::EspError;
use esp_idf_hal::units::*;

fn read_register<'d, T>(
    spi_device: &mut SpiDeviceDriver<'d, T>,
    register_address: u8,
    channel: u8,
) -> Result<(), EspError>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    let write = false;
    let write_byte = if write { 0x00u8 } else { 0x80u8 };
    let spi_data: [u8; 2] = [
        write_byte | ((register_address << 3) | (channel << 1)),
        0xFFu8,
    ];

    let mut rx: [u8; 2] = [0u8; 2];
    let ret_val = spi_device.transfer(&mut rx, &spi_data);
    log::info!(
        "Device 1: Wrote [{:02x}, {:02x}], read [{:02x}, {:02x}]",
        spi_data[0],
        spi_data[1],
        rx[0],
        rx[1]
    );
    ret_val
}

fn write_register<'d, T>(
    spi_device: &mut SpiDeviceDriver<'d, T>,
    register_address: u8,
    channel: u8,
    value: u8,
) -> Result<(), EspError>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    let write = true;
    let write_byte = if write { 0x00u8 } else { 0x80u8 };
    let spi_data: [u8; 2] = [
        write_byte | ((register_address << 3) | (channel << 1)),
        value,
    ];

    let mut rx: [u8; 0] = [];
    let ret_val = spi_device.transfer(&mut rx, &spi_data);
    log::info!(
        "Device 1: Wrote [{:02x}, {:02x}]]",
        spi_data[0],
        spi_data[1]
    );
    ret_val
}

// build: cargo build
// flash: espflash flash target/riscv32imc-esp-espidf/debug/c3-ru-mamu --monitor
fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    use esp_idf_hal::peripherals::Peripherals;

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio5;
    let miso = peripherals.pins.gpio7; // SDI
    let mosi = peripherals.pins.gpio8; // SDO
    let cs = peripherals.pins.gpio6;

    println!("Starting SPI loopback test");

    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();

    let config_1 = config::Config::new().baudrate(1.MHz().into());
    let mut device_1 = SpiDeviceDriver::new(&driver, Some(cs), &config_1).unwrap();

    let register_address = 0x0A;
    let channel = 0u8;
    let mut value = 0;
    loop {
        match write_register(&mut device_1, register_address, channel, value) {
            Ok(_) => {
                log::info!(
                    "Device 1: Wrote register address {:02x} successfully",
                    register_address
                );
            }
            Err(_) => {
                log::info!("Oh no Error!");
            }
        }
        if value == 0 {
            value = 1;
        } else {
            value = 0;
        }
        FreeRtos::delay_ms(500);
        // for register_address in 0u8..0xFu8 {
        //     match read_register(&mut device_1, register_address, 0u8) {
        //         Ok(_) => {
        //             log::info!(
        //                 "Device 1: Read register address {:02x} successfully",
        //                 register_address
        //             );
        //         }
        //         Err(_) => {
        //             log::info!("Oh no Error!");
        //         }
        //     }

        //     FreeRtos::delay_ms(500);
        // }
        log::info!("\n");
    }
}
