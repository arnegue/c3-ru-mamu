use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

use ::sc16is752::Channel;
use ::sc16is752::InterruptEvents;
use ::sc16is752::Parity;
use ::sc16is752::PinMode;
use ::sc16is752::PinState;
use ::sc16is752::SC16IS752spi;
use ::sc16is752::UartConfig;
use ::sc16is752::GPIO;
use ::sc16is752::SC16IS752;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::InterruptType;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::*;
use heapless::Vec;
use sc16is752::FIFO_MAX_TRANSMITION_LENGTH;
use sc16is752::FIFO_SIZE;

static INTERUPT_OCCURRED: AtomicBool = AtomicBool::new(false); // Notifier that message transmission is complete

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
    let reset_pin = peripherals.pins.gpio4; // RST

    println!("Starting SPI loopback test");

    // SPI only
    let driver =
        SpiDriver::new::<SPI2>(spi, sclk, mosi, Some(miso), &SpiDriverConfig::new()).unwrap();

    let config_1 = config::Config::new().baudrate(1.MHz().into());
    let device_1 = SpiDeviceDriver::new(&driver, Some(cs), &config_1).unwrap();

    // Interrupt pin for SC16IS752
    let mut isr_pin_driver = PinDriver::input(isr_pin).unwrap();
    isr_pin_driver
        .set_interrupt_type(InterruptType::NegEdge)
        .unwrap();
    unsafe { isr_pin_driver.subscribe(gpio_isr).unwrap() }
    isr_pin_driver.enable_interrupt().unwrap();

    // SC16IS752 setup
    let spi_bus = SC16IS752spi::new(device_1);
    let mut sc16is752_a = SC16IS752::new(spi_bus, 1843200.Hz().into(), Channel::A);
    let device_a_config = UartConfig::new(9600, 8, Parity::NoParity, 1);
    sc16is752_a.reset_device();
    sc16is752_a.initialise_uart(device_a_config).unwrap();
    sc16is752_a.ping(); // TODO error handling
    sc16is752_a.gpio_set_pin_mode(GPIO::GPIO0, PinMode::Output);
    let interrupt_bitmask = 0b111;
    sc16is752_a.interrupt_control(interrupt_bitmask);

    // Debugging stuff
    let mut current_led = PinState::High;

    let mut my_buffer: Vec<u8, FIFO_MAX_TRANSMITION_LENGTH> = Vec::new();
    for m in 0u8..(FIFO_SIZE as u8) {
        my_buffer.push(m);
    }

    // Main loop
    loop {
        let mut try_read = false;
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

        if INTERUPT_OCCURRED.load(Ordering::Relaxed) {
            log::info!("Interrupt occured!");
            match sc16is752_a.isr() {
                Ok(interrupt_kind) => match interrupt_kind {
                    InterruptEvents::RHR_INTERRUPT => {
                        log::info!("RHR_INTERRUPT");
                        try_read = true;
                    }
                    InterruptEvents::RECEIVE_LINE_STATUS_ERROR => {
                        try_read = true;
                        log::info!("RECEIVE_LINE_STATUS_ERROR")
                    }
                    InterruptEvents::RECEIVE_TIMEOUT_INTERRUPT => {
                        try_read = true;
                        log::info!("RECEIVE_TIMEOUT_INTERRUPT")
                    }
                    InterruptEvents::THR_INTERRUPT => log::info!("THR_INTERRUPT"),
                    InterruptEvents::MODEM_INTERRUPT => log::info!("MODEM_INTERRUPT"),
                    InterruptEvents::INPUT_PIN_CHANGE_STATE => {
                        log::info!("INPUT_PIN_CHANGE_STATE")
                    }
                    InterruptEvents::RECEIVE_XOFF => log::info!("RECEIVE_XOFF"),
                    InterruptEvents::CTS_RTS_CHANGE => log::info!("CTS_RTS_CHANGE"),
                    InterruptEvents::NO_INTERRUPT => log::info!("NO_INTERRUPT"),
                    InterruptEvents::UNKNOWN => log::info!("UNKNOWN"),
                },
                Err(err) => {
                    log::error!("Error in isr");
                }
            }
            //spi_interrupt_event(sc16is752_a);
            isr_pin_driver.enable_interrupt().unwrap(); // Reenable interrupt again
            INTERUPT_OCCURRED.store(false, Ordering::Relaxed);
        }

        // Interrupt happend which tells, that there might be data in buffer
        if try_read {
            let available_bytes = sc16is752_a.fifo_available_data().unwrap();
            if available_bytes > 0 {
                match sc16is752_a.read_cycle(available_bytes as usize) {
                    Ok(read_bytes) => {
                        // There could be a race-condition, that between the call of available bytes and the actual reading the size increases,
                        // but that shouldn't be that bad and could be handled later when parsing the buffer
                        let buf_str =
                            buffer_to_string(read_bytes.as_ref(), available_bytes as usize);
                        log::info!("Device 1: Read {available_bytes} bytes: {buf_str}");
                    }
                    Err(_) => {}
                }
            }
        }

        // Write bytes
        match sc16is752_a.write_cycle(my_buffer.clone(), FIFO_SIZE) {
            Ok(_) => {
                log::info!("Device 1: Wrote bytes:");
            }
            Err(e) => {
                log::info!("Device 1: Failed to write bytes: {e}");
            }
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

fn gpio_isr() {
    INTERUPT_OCCURRED.store(true, Ordering::Relaxed);
}
