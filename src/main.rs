mod led_task;
mod spi_task;
mod uart_task;
mod utils;
mod bit_banged_uart;

use crate::led_task::start_led_task;
use crate::spi_task::start_spi_task;
use crate::uart_task::start_uart_task;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{self, PinDriver};
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
    let mut config = esp_idf_hal::uart::config::Config::default();
    config.baudrate = Hertz(4800);
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityEven;
    config.stop_bits = StopBits::STOP1;
    config.event_config = EventConfig::new();
    config.event_config.flags.insert(EventFlags::ParityError);

    let uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )
    .unwrap();

    start_uart_task(uart);

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
