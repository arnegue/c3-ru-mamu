mod bit_banged_uart;
mod led_task;
mod spi_task;
mod uart_task;
mod utils;

use crate::bit_banged_uart::{BitBangedUART, ParityType, };
use crate::led_task::start_led_task;
use crate::spi_task::start_spi_task;
use crate::uart_task::start_uart_task;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::spi::{SpiDeviceDriver, SpiDriver, SpiDriverConfig, SPI2};
use esp_idf_hal::uart::config::{DataBits, EventConfig, EventFlags, Parity, StopBits};
use esp_idf_hal::uart::UartDriver;
use esp_idf_hal::units::{Hertz, MegaHertz};
use esp_idf_svc::hal::task::queue::Queue;

fn main() {
    // ESP-IDF initialization stuff
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    use esp_idf_hal::peripherals::Peripherals;

    let peripherals = Peripherals::take().unwrap();

    // SPI-Task
    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio5; // CLK
    let miso = peripherals.pins.gpio7; // SDI
    let mosi = peripherals.pins.gpio8; // SDO
    let cs = peripherals.pins.gpio3; // CS (old device is pin 6)
    let isr_pin = peripherals.pins.gpio2; // IRQ


    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();
    let config = esp_idf_hal::spi::config::Config::new().baudrate(MegaHertz(1).into());
    let device_driver = SpiDeviceDriver::new(&driver, Some(cs), &config).unwrap();
    // SPI isr-pin
    let isr_pin_driver = PinDriver::input(isr_pin).unwrap();
    start_spi_task(device_driver, isr_pin_driver);

    // UART-Task

    let uart_tx_queue: Queue<u8>;
    let uart_rx_queue: Queue<u8>;
    let tx = peripherals.pins.gpio18;
    let rx = peripherals.pins.gpio19;

    let tx_pin: PinDriver<'static, esp_idf_hal::gpio::Gpio18, esp_idf_hal::gpio::Output> =
        PinDriver::output(tx).unwrap();
    let mut bbu = BitBangedUART::new(
        PinDriver::input(rx).unwrap(),
        tx_pin,
        Hertz(4800),
        ParityType::PARITY_EVEN,
        8,
    );

    // LED-Task
    let led_pin = peripherals.pins.gpio0;
    let led: PinDriver<'_, esp_idf_hal::gpio::Gpio0, esp_idf_hal::gpio::Output> =
        PinDriver::output(led_pin).unwrap();
    start_led_task(led);

    // Main loop
    let mut counter: u8 = 0;
    loop {
        log::info!("Main-Delay");
        for byte in b"Hello, World! " {
            bbu.write(*byte);
        }
        bbu.write(counter);
        counter += 1;

        for byte in b"\r\n" {
            bbu.write(*byte);
        }
        FreeRtos::delay_ms(1000);
    }
}
