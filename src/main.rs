mod SC16IS752;
use SC16IS752::SC16IS752Device;

use std::borrow::Borrow;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::spi::*;
use esp_idf_hal::sys::EspError;
use esp_idf_hal::units::*;


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
    let device_1 = SpiDeviceDriver::new(&driver, Some(cs), &config_1).unwrap();

    let mut sc16is752 = SC16IS752Device::new(device_1);
    // let uart1_config = SC16IS752::UARTConfig {
    //     baud_rate: 9600,
    //     data_bits: 8,
    //     stop_bits: 1,
    //     parity: esp_idf_hal::uart::config::Parity::ParityNone,

    // };
    // let uart1_device = SC16IS752::SC16IS752UART::new(sc16is752, uart1_config).unwrap();

    let register_address = 0x0A;
    let channel = 0u8;
    let mut value = 0;
    loop {
        match sc16is752.write_register(register_address, channel, value) {
            Ok(_) => {
                log::info!(
                    "Device 1: Wrote register address 0x{:02X} successfully\n",
                    register_address
                );
            }
            Err(_) => {
                log::info!("Oh no Error\n");
            }
        }
        if value == 0 {
            value = 1;
        } else {
            value = 0;
        }
        FreeRtos::delay_ms(500);
    }
}
