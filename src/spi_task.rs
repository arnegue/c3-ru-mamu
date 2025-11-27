use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Input, InterruptType, Pin, PinDriver},
    spi::{SpiDeviceDriver, SpiDriver},
    sys::{xTaskCreatePinnedToCore, xTaskGenericNotifyFromISR, xTaskGenericNotifyWait},
    task::do_yield,
};
use esp_idf_sys::{eNotifyAction_eSetBits, BaseType_t, TaskHandle_t};
use std::{
    borrow::Borrow,
    ffi::CString,
    ptr,
    sync::atomic::{AtomicBool, Ordering},
};

use esp_idf_hal::units::*;
use sc16is752::Channel;
use sc16is752::InterruptEvents;
use sc16is752::Parity;
use sc16is752::PinMode;
use sc16is752::SC16IS752spi;
use sc16is752::UartConfig;
use sc16is752::GPIO;
use sc16is752::SC16IS752;

// Task handle for SPI task (needed by ISR)
static mut TASK_HANDLE: Option<TaskHandle_t> = None;
static ISR_HAPPENED: AtomicBool = AtomicBool::new(false); // Notifier that message transmission is complete

// ISR: GPIO interrupt fires -> notify SPI task
#[no_mangle]
fn gpio_isr_handler() {
    ISR_HAPPENED.store(true, Ordering::SeqCst);
    /*unsafe {
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

            /*if hp_task_woken != 0 {
                do_yield();
            }*/
        }
    }*/
}

type Sc16<'d, T> = SC16IS752<SC16IS752spi<SpiDeviceDriver<'d, T>>>;

struct TaskParameter<'d, T, U: Pin>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    sc16: Sc16<'d, T>,
    isr_pin_driver: PinDriver<'static, U, Input>,
}

// SPI-Task which reads and writes from/to SC16IS752
extern "C" fn spi_task<'d, T, U: Pin>(param: *mut core::ffi::c_void)
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    let mut notif_value: u32 = 0;
    let mut try_read = false;
    let mut try_write = false;
    let mut first_run = true; // In case an interrupt was raised before this task was spawned and xTaskGenericNotifyWait was reached
    unsafe {
        // Recover Box from raw pointer
        let task_parameter: Box<TaskParameter<'d, T, U>> = Box::from_raw(param as *mut _);
        let mut sc16 = task_parameter.sc16;
        let mut isr_pin_driver = task_parameter.isr_pin_driver;

        loop {
            if ISR_HAPPENED.load(Ordering::SeqCst) {
                log::info!("ISR_HAPPENED!");
                isr_pin_driver.enable_interrupt().unwrap(); // Reenable interrupt again
                ISR_HAPPENED.store(false, Ordering::SeqCst);
            }
            if first_run
                || (xTaskGenericNotifyWait(
                    0,        // index
                    0,        // bits to clear on entry
                    u32::MAX, // bits to clear on exit
                    &mut notif_value,
                    0,
                ) == 1)
            {
                log::info!("Interrupt occured!");
                match sc16.isr() {
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
                        InterruptEvents::THR_INTERRUPT => {
                            log::info!("THR_INTERRUPT");
                            try_write = true;
                        }
                        InterruptEvents::MODEM_INTERRUPT => {
                            log::info!("MODEM_INTERRUPT");
                        }
                        InterruptEvents::INPUT_PIN_CHANGE_STATE => {
                            log::info!("INPUT_PIN_CHANGE_STATE")
                        }
                        InterruptEvents::RECEIVE_XOFF => {
                            log::info!("RECEIVE_XOFF");
                        }
                        InterruptEvents::CTS_RTS_CHANGE => {
                            log::info!("CTS_RTS_CHANGE");
                        }
                        InterruptEvents::NO_INTERRUPT => {
                            log::info!("NO_INTERRUPT");
                        }
                        InterruptEvents::UNKNOWN => {
                            log::info!("UNKNOWN");
                        }
                    },
                    Err(err) => {
                        log::error!("Error in isr");
                    }
                }
                if try_read {
                    let available_bytes = sc16.fifo_available_data().unwrap();
                    let _read_bytes = sc16.read_cycle(available_bytes as usize);
                    // TODO
                } else if try_write {
                    // TODO sc16.write_cycle(payload, length)
                } else {
                    log::warn!("Noting to do after interrupt");
                }
                first_run = false;
                isr_pin_driver.enable_interrupt().unwrap(); // Reenable interrupt again
            } else {
                log::warn!("Timeout");
            }
            FreeRtos::delay_ms(500);
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
    let mut sc16is752 = SC16IS752::new(spi_bus, 1843200.Hz().into(), Channel::A);
    let device_a_config = UartConfig::new(9600, 8, Parity::NoParity, 1);
    sc16is752.reset_device();
    sc16is752.initialise_uart(device_a_config).unwrap();
    sc16is752.ping(); // TODO error handling
    sc16is752.gpio_set_pin_mode(GPIO::GPIO0, PinMode::Output);
    let interrupt_bitmask = 0b111;
    sc16is752.interrupt_control(interrupt_bitmask);
    TaskParameter {
        sc16: sc16is752,
        isr_pin_driver: isr_pin_driver,
    }
}

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
        let mut task_handle: TaskHandle_t = ptr::null_mut();
        let res = xTaskCreatePinnedToCore(
            Some(spi_task::<T, U>),
            CString::new("SPI Task").unwrap().as_ptr(),
            5000,
            task_param_ptr as *mut core::ffi::c_void,
            10,
            &mut task_handle, // Out: task handle
            0,
        );

        if res != 1 {
            panic!("Task creation failed");
        }
        TASK_HANDLE = Some(task_handle);
    }
}
