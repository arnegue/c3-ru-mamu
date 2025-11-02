mod sc16is752;
use esp_idf_hal::uart::config::StopBits;
use sc16is752::SC16IS752Device;

use std::cell::RefCell;
use std::rc::Rc;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::spi::*;
use esp_idf_hal::sys::EspError;
use esp_idf_hal::units::*;

use crate::sc16is752::UARTParity;

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

    let reset_pin = peripherals.pins.gpio4; // RST

    println!("Starting SPI loopback test");

    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();

    let config_1 = config::Config::new().baudrate(1.MHz().into());
    let device_1 = SpiDeviceDriver::new(&driver, Some(cs), &config_1).unwrap();

    let sc16is752_device = Rc::new(RefCell::new(SC16IS752Device::new(
        device_1,
        reset_pin,
        1843200.Hz().into(),
    )));
    let uart1_config = sc16is752::UARTConfig {
        baud_rate: 9600,
        data_bits: 8,
        stop_bits: StopBits::STOP1,
        parity: UARTParity::None,
    };
    sc16is752_device.borrow_mut().hard_reset();

    let mut uart1_device =
        sc16is752::SC16IS752UART::new(sc16is752_device.clone(), uart1_config, false).unwrap();
    let mut _uart2_device =
        sc16is752::SC16IS752UART::new(sc16is752_device.clone(), uart1_config, true).unwrap();

    let mut temp_write_byte: u8 = 33;
    let mut current_led = true;
    loop {
        // Toggle LED to show activity
        sc16is752_device
            .borrow_mut()
            .set_gpio_direction(current_led as u8)
            .unwrap();
        current_led = !current_led;

        // Read a byte
        match uart1_device.read_byte() {
            Ok(read_byte) => {
                log::info!("Device 1: Read byte: {read_byte}");
            }
            Err(_) => {}
        }

        // Write a byte
        match uart1_device.write_byte(temp_write_byte) {
            Ok(_) => {
                let temp_write_char: char = temp_write_byte as char;
                log::info!("Device 1: Wrote byte: {temp_write_char}");
            }
            Err(e) => {
                log::info!("Device 1: Failed to write byte: {e}");
            }
        }
        temp_write_byte = temp_write_byte + 1;
        if temp_write_byte >= 127 {
            temp_write_byte = 33;
        }

        FreeRtos::delay_ms(500);
    }
}
