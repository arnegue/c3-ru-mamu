use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::InterruptType;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::*;
use heapless::Vec;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use sc16is752::Channel;
use sc16is752::InterruptEvents;
use sc16is752::Parity;
use sc16is752::PinMode;
use sc16is752::PinState;
use sc16is752::SC16IS752spi;
use sc16is752::UartConfig;
use sc16is752::FIFO_MAX_TRANSMITION_LENGTH;
use sc16is752::FIFO_SIZE;
use sc16is752::GPIO;
use sc16is752::SC16IS752;

use crate::led_task::start_led_tasks;

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

    let led_pin = peripherals.pins.gpio0;
    let led: PinDriver<'_, esp_idf_hal::gpio::Gpio0, esp_idf_hal::gpio::Output> =
        PinDriver::output(led_pin).unwrap();

    let result_led = start_led_tasks(led);
    if result_led != 1 {
        panic!("Spaning LED task failed: {}", result_led);
    }

  
    // Main loop
    loop {
        log::info!("Main-Delay");
        
        FreeRtos::delay_ms(1000);
    }
        
}

