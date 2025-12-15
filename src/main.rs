use crate::led_task::start_led_task;
use crate::spi_task::start_spi_task;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::*;

mod led_task;
mod message_parser;
mod parser;
mod spi_task;
mod telemetry_data;

fn main() {
    // ESP-IDF initialization stuff
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    use esp_idf_hal::peripherals::Peripherals;

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio5; // CLK
    let miso = peripherals.pins.gpio7; // SDI
    let mosi = peripherals.pins.gpio8; // SDO
    let cs = peripherals.pins.gpio3; // CS (old device is pin 6)
    let isr_pin = peripherals.pins.gpio2; // IRQ

    // SPI only
    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();
    let config = config::Config::new().baudrate(1.MHz().into());
    let device_driver = SpiDeviceDriver::new(&driver, Some(cs), &config).unwrap();

    // SPI isr-pin
    let isr_pin_driver = PinDriver::input(isr_pin).unwrap();
    start_spi_task(device_driver, isr_pin_driver);

    // LED-Task
    let led_pin = peripherals.pins.gpio0;
    let led: PinDriver<'_, esp_idf_hal::gpio::Gpio0, esp_idf_hal::gpio::Output> =
        PinDriver::output(led_pin).unwrap();
    start_led_task(led);

    // Main loop
    loop {
        log::debug!("Main-Delay");
        FreeRtos::delay_ms(1000);
    }
}
