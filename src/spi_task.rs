use esp_idf_hal::spi::SpiError;
use esp_idf_hal::units::*;
use esp_idf_hal::{
    gpio::{Input, InterruptType, Pin, PinDriver},
    spi::{SpiDeviceDriver, SpiDriver},
    sys::{xTaskCreatePinnedToCore, xTaskGenericNotifyFromISR, xTaskGenericNotifyWait},
    task::do_yield,
};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_sys::{eNotifyAction_eSetBits, BaseType_t, TaskHandle_t};
use sc16is752::{
    Channel, InterruptEvents, Parity, PinMode, PinState, SC16IS752spi, UartConfig, GPIO, SC16IS752,
};
use std::{borrow::Borrow, ffi::CString, ptr};

// Task handle for SPI task (needed by ISR)
static mut TASK_HANDLE: Option<TaskHandle_t> = None;

// ISR: GPIO interrupt fires -> notify SPI task
#[no_mangle]
fn gpio_isr_handler() {
    unsafe {
        let mut hp_task_woken: BaseType_t = 0;
        if let Some(task) = TASK_HANDLE {
            xTaskGenericNotifyFromISR(
                task,                   // task handle
                0,                      // notification index (always 0 unless using arrays)
                1,                      // value or bitmask
                eNotifyAction_eSetBits, // notify action
                std::ptr::null_mut(),
                &mut hp_task_woken, // did we wake a higher priority task?
            );

            if hp_task_woken != 0 {
                do_yield();
            }
        }
    }
}

type Sc16<'d, T> = SC16IS752<SC16IS752spi<SpiDeviceDriver<'d, T>>>;

struct TaskParameter<'d, T, U: Pin>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    sc16: Sc16<'d, T>,                            // Instance of SPI-SC16IS752
    isr_pin_driver: PinDriver<'static, U, Input>, // Interrupt pin for ISR for SC16IS752
}

// SPI-Task which reads and writes from/to SC16IS752
extern "C" fn spi_task<'d, T, U: Pin>(param: *mut core::ffi::c_void)
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    // Task-Parameters
    let task_parameter: Box<TaskParameter<'d, T, U>>;
    let mut sc16: SC16IS752<SC16IS752spi<SpiDeviceDriver<'_, T>>>;
    let mut isr_pin_driver: PinDriver<'_, U, Input>;
    unsafe {
        // Unbox them from raw pointer
        task_parameter = Box::from_raw(param as *mut _);
        sc16 = task_parameter.sc16;
        isr_pin_driver = task_parameter.isr_pin_driver;
    }

    // Variables for in-loop-stuff
    let mut notif_value: u32 = 0; // TODO later important for multiple SC16s
    let mut first_run = true; // In case an interrupt was raised before this task was spawned and xTaskGenericNotifyWait was reached
    let mut led_state = false; // LED-State to toggle

    loop {
        // Check if interrupt happened
        let interrupt_occurred: bool;
        unsafe {
            interrupt_occurred = xTaskGenericNotifyWait(
                0,        // index
                0,        // bits to clear on entry
                u32::MAX, // bits to clear on exit
                &mut notif_value,
                TickType::new_millis(1000).ticks(),
            ) == 1;
        }

        // Handle interrupt
        if interrupt_occurred || first_run {
            // TODO how to determine which channel has an
            for channel in [Channel::A, Channel::B] {
                let mut try_read = false; // Interrupt concerning Receiving occurred. Try to read from register
                let mut try_write = false; // Interrupt concerning Transmitting occurred. Try to write to register

                match sc16.isr(channel) {
                    Ok(interrupt_kind) => match interrupt_kind {
                        InterruptEvents::RHR_INTERRUPT => {
                            log::info!("Device {channel}: RHR_INTERRUPT");
                            try_read = true;
                        }
                        InterruptEvents::RECEIVE_LINE_STATUS_ERROR => {
                            // TODO error-handling?
                            try_read = true;
                            log::info!("Device {channel}: RECEIVE_LINE_STATUS_ERROR")
                        }
                        InterruptEvents::RECEIVE_TIMEOUT_INTERRUPT => {
                            try_read = true;
                            log::info!("Device {channel}: RECEIVE_TIMEOUT_INTERRUPT")
                        }
                        InterruptEvents::THR_INTERRUPT => {
                            log::info!("Device {channel}: THR_INTERRUPT");
                            try_write = true;
                        }
                        InterruptEvents::NO_INTERRUPT => {
                            // Interrupt happened on other channel?
                            log::info!("Device {channel}: NO_INTERRUPT");
                        }
                        InterruptEvents::UNKNOWN => {
                            // TODO error-handling?
                            log::error!("Device {channel}: UNKNOWN interrupt occurred");
                        }
                        other => {
                            let u8_interrupt = other as u8;
                            log::info!("Device {channel}: Uninteresting Interrupt occurred ({u8_interrupt})");
                        }
                    },
                    Err(err) => {
                        let s = format!("{:?}", err);
                        log::error!("Error in isr: {}", s);
                    }
                }
                if try_read {
                    let available_bytes = sc16.fifo_available_data(channel).unwrap();
                    match sc16.read_cycle(channel, available_bytes as usize) {
                        Ok(read_bytes) => {
                            // There could be a race-condition, that between the call of available bytes and the actual reading the size increases,
                            // but that shouldn't be that bad and could be handled later when parsing the buffer
                            let buf_str =
                                buffer_to_string(read_bytes.as_ref(), available_bytes as usize);
                            log::info!("Device {channel}: Read {available_bytes} bytes: {buf_str}");
                        }
                        Err(_) => {
                            log::error!(
                                "Error when reading from device {channel}: {available_bytes} bytes"
                            );
                        }
                    }
                } else if try_write {
                    // TODO sc16.write_cycle(payload, length)
                } else {
                    log::debug!("Nothing to do after interrupt of device {channel}");
                }
            }
            first_run = false;
            isr_pin_driver.enable_interrupt().unwrap(); // Reenable interrupt again
        } else {
            log::debug!("Timeout");

            match sc16.gpio_set_pin_state(
                GPIO::GPIO0,
                if led_state {
                    PinState::High
                } else {
                    PinState::Low
                },
            ) {
                Ok(_) => log::debug!("Toggled LED"),
                Err(_) => log::warn!("Error toggling LED"),
            }
            led_state = !led_state;
        }
    }
}

// Creates a SC16IS752 instance
fn setup_sc16is752<'d, T, U: Pin>(
    spi_device_driver: SpiDeviceDriver<'d, T>,
    mut isr_pin_driver: PinDriver<'static, U, Input>,
) -> TaskParameter<'d, T, U>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    // Interrupt pin for SC16IS752
    isr_pin_driver
        .set_interrupt_type(InterruptType::NegEdge)
        .unwrap();
    unsafe { isr_pin_driver.subscribe(gpio_isr_handler).unwrap() }
    isr_pin_driver.enable_interrupt().unwrap();

    // SC16IS752 setup
    let spi_bus = SC16IS752spi::new(spi_device_driver);
    let mut sc16is752 = SC16IS752::new(spi_bus, 1843200.Hz().into());

    // Initialize SC16IS752
    let result: Result<(), SpiError> = (|| {
        sc16is752.reset_device()?;
        sc16is752.gpio_set_pin_mode(GPIO::GPIO0, PinMode::Output)?;
        sc16is752.ping()?; // TODO error handling

        // Initialize Channel A
        let device_a_config = UartConfig::new(9600, 8, Parity::NoParity, 1);
        sc16is752.initialise_uart(Channel::A, device_a_config)?;
        let interrupt_bitmask = 0b111;
        sc16is752.interrupt_control(Channel::A, interrupt_bitmask)?;

        // Initialize Channel B
        let device_b_config = UartConfig::new(4800, 8, Parity::Even, 1);
        sc16is752.initialise_uart(Channel::B, device_b_config)?;
        let interrupt_bitmask = 0b111;
        sc16is752.interrupt_control(Channel::B, interrupt_bitmask)?;
        Ok(())
    })();

    if result.is_err() {
        let s = format!("{:?}", result.err());
        panic!("Creating SC16IS752 failed: {}", s)
    }

    // Put them into parameters to pass to task
    TaskParameter {
        sc16: sc16is752,
        isr_pin_driver: isr_pin_driver,
    }
}

// Setups SC16IS752, its ISR and starts task
pub fn start_spi_task<'d, T, U: Pin>(
    spi_device_driver: SpiDeviceDriver<'d, T>,
    isr_pin_driver: PinDriver<'static, U, Input>,
) where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    let task_param = setup_sc16is752(spi_device_driver, isr_pin_driver);

    // Box it so ownership can be transferred
    let task_param_box = Box::new(task_param);
    // Convert into raw pointer for FreeRTOS
    let task_param_ptr = Box::into_raw(task_param_box);

    unsafe {
        // Start task
        let mut task_handle: TaskHandle_t = ptr::null_mut();
        let res = xTaskCreatePinnedToCore(
            Some(spi_task::<T, U>),
            CString::new("SPI Task").unwrap().as_ptr(),
            5000,
            task_param_ptr as *mut core::ffi::c_void,
            10,
            &mut task_handle,
            0,
        );

        if res != 1 {
            panic!("Task creation failed");
        }

        // Assign handle for ISR
        TASK_HANDLE = Some(task_handle);
    }
}

// Simple u8-buffer to string converter
fn buffer_to_string(buffer: &[u8], size: usize) -> String {
    let mut result = String::new();
    for i in 0..size {
        result.push(buffer[i] as char);
    }
    result
}
