use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::InterruptType;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::*;
use heapless::Vec;
use sc16is752::Bus;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use crate::led_task::start_led_task;
use crate::spi_task::start_spi_task;

mod led_task;
mod spi_task;

static INTERUPT_OCCURRED: AtomicBool = AtomicBool::new(false); // Notifier that message transmission is complete

fn main() {
    // ESP-IDF initialization stuff
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    use esp_idf_hal::peripherals::Peripherals;

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio5; // CLK
    let miso = peripherals.pins.gpio7; // SDI
    let mosi = peripherals.pins.gpio8; // SDO
    let cs = peripherals.pins.gpio3; // CS (old device is pin 6)

    let isr_pin = peripherals.pins.gpio2; // IRQ
    let reset_pin = peripherals.pins.gpio4; // RST


    // SPI only
    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();
    let config = config::Config::new().baudrate(1.MHz().into());
    let device_driver = SpiDeviceDriver::new(&driver, Some(cs), &config).unwrap();

    //start_spi_task::<SPI>(driver);
    start_spi_task::<&esp_idf_hal::spi::SpiDriver<'_>, dyn Bus::<Error = Type>>(&mut device_driver);
    
    // LED-Task
    let led_pin = peripherals.pins.gpio0;
    let led: PinDriver<'_, esp_idf_hal::gpio::Gpio0, esp_idf_hal::gpio::Output> =
        PinDriver::output(led_pin).unwrap();
    let result_led = start_led_task(led);
    if result_led != 1 {
        panic!("Spawning LED task failed: {}", result_led);
    }

    // Main loop
    loop {
        log::info!("Main-Delay");

        FreeRtos::delay_ms(1000);
    }
}
