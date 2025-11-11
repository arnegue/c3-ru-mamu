mod sc16is752;

use ::sc16is752::Channel;
use ::sc16is752::Parity;
use ::sc16is752::PinMode;
use ::sc16is752::PinState;
use ::sc16is752::SC16IS752spi;
use ::sc16is752::UartConfig;
use ::sc16is752::GPIO;
use ::sc16is752::SC16IS752;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::spi::*;
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

    let sclk = peripherals.pins.gpio5; // CLK
    let miso = peripherals.pins.gpio7; // SDI
    let mosi = peripherals.pins.gpio8; // SDO
    let cs = peripherals.pins.gpio3; // CS (old device is pin 6)

    let isr = peripherals.pins.gpio2; // IRQ
    let reset_pin = peripherals.pins.gpio4; // RST

    println!("Starting SPI loopback test");

    // SPI only
    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();

    let config_1 = config::Config::new().baudrate(1.MHz().into());
    let device_1 = SpiDeviceDriver::new(&driver, Some(cs), &config_1).unwrap();

    // SC16IS752 setup
    let spi_bus = SC16IS752spi::new(device_1);
    let mut sc16is752_a = SC16IS752::new(spi_bus, 1843200.Hz().into());

    let device_a_config = UartConfig::new(9600, 8, Parity::NoParity, 1);

    sc16is752_a
        .initialise_uart(Channel::A, device_a_config)
        .unwrap();
    sc16is752_a.gpio_set_pin_mode(GPIO::GPIO0, PinMode::Output);

    let mut temp_write_byte: u8 = 33;
    let mut current_led = PinState::High;
    loop {
        let bool_val = match current_led {
            PinState::High => true,
            PinState::Low => false,
        };
        // Toggle LED to show activity
        match sc16is752_a.gpio_set_pin_state(GPIO::GPIO0, current_led) {
            Ok(_) => {
                //log::info!("Device 1: Set LED to: {bool_val}");
            }
            Err(e) => {
                log::info!("Device 1: Failed to set LED: {e}");
            }
        }
        current_led = match bool_val {
            true => PinState::Low,
            false => PinState::High,
        };

        // Read a byte
        let mut my_read_buffer = [0u8; 23];
        if sc16is752_a.fifo_available_data(Channel::A).unwrap() > 0 {
            match sc16is752_a.read(Channel::A, &mut my_read_buffer) {
                Ok(read_bytes) => {
                    let buf_str = buffer_to_string(my_read_buffer.as_ref(), read_bytes);
                    log::info!("Device 1: Read {read_bytes} bytes: {buf_str}");
                }
                Err(_) => {}
            }
        }

        // Write a byte
        let my_buffer = [temp_write_byte; 1];
        match sc16is752_a.write(Channel::A, &my_buffer) {
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

fn buffer_to_string(buffer: &[u8], size: usize) -> String {
    let mut result = String::new();
    for i in 0..size {
        result.push(buffer[i] as char);
    }
    result
}
