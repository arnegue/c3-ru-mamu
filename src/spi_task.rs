use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Input, InterruptType, Pin, PinDriver},
    spi::{SpiDeviceDriver, SpiDriver},
    sys::{self, tskTaskControlBlock, xTaskCreatePinnedToCore, xTaskGenericNotifyFromISR},
};
use esp_idf_sys::TaskHandle_t;
use std::{
    borrow::Borrow,
    ffi::CString,
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
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
static SPI_TASK_HANDLE: AtomicPtr<tskTaskControlBlock> = AtomicPtr::new(core::ptr::null_mut());

// ISR: GPIO interrupt fires -> notify SPI task
fn gpio_isr_handler() {
    let handle = SPI_TASK_HANDLE.load(Ordering::SeqCst);
    if handle.is_null() {
        return;
    }

    unsafe {
        let mut hp_task_woken: sys::BaseType_t = 0;

        xTaskGenericNotifyFromISR(
            handle, // task handle
            0,      // notification index (always 0 unless using arrays)
            1,      // value or bitmask
            0,      // notify action
            std::ptr::null_mut(),
            &mut hp_task_woken, // did we wake a higher priority task?
        );

        if hp_task_woken != 0 {
            //sys::portYIELD_FROM_ISR();
        }
    }
}
extern "C" fn spi_task<'d, T>(param: *mut core::ffi::c_void)
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    unsafe {
        // Recover Box from raw pointer
        let mut sc16: Box<SC16IS752<SC16IS752spi<SpiDeviceDriver<'_, T>>>> =
            Box::from_raw(param as *mut _);
        let mut notif_value: u32 = 0;
        let mut try_read = false;
        let mut try_write = false;

        loop {
            sys::xTaskGenericNotifyWait(
                0,        // index
                0,        // bits to clear on entry
                u32::MAX, // bits to clear on exit
                &mut notif_value,
                0,
            );
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
            FreeRtos::delay_ms(500);
        }
    }
}

type Sc16<'d, T> = SC16IS752<SC16IS752spi<SpiDeviceDriver<'d, T>>>;

fn setup_sc16is752<'d, T, U: Pin>(
    spi_device_driver: SpiDeviceDriver<'d, T>,
    mut isr_pin_driver: PinDriver<'static, U, Input>,
) -> Sc16<'d, T>
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
    sc16is752
}

pub fn start_spi_task<'d, T, U: Pin>(
    spi_device_driver: SpiDeviceDriver<'d, T>,
    isr_pin_driver: PinDriver<'static, U, Input>,
) where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    let sc16is752: SC16IS752<SC16IS752spi<SpiDeviceDriver<'_, T>>> =
        setup_sc16is752(spi_device_driver, isr_pin_driver);
    // Box it so ownership can be transferred
    let sc16is752_boxed: Box<Sc16<'d, T>> = Box::new(sc16is752);
    // Convert into raw pointer for FreeRTOS
    let sc16is752_ptr: *mut Sc16<'d, T> = Box::into_raw(sc16is752_boxed);

    unsafe {
        let mut task_handle: TaskHandle_t = ptr::null_mut();
        let res = xTaskCreatePinnedToCore(
            Some(spi_task::<T>),
            CString::new("SPI Task").unwrap().as_ptr(),
            5000,
            sc16is752_ptr as *mut core::ffi::c_void,
            10,
            &mut task_handle as *mut _, // Out: task handle
            0,
        );

        if res != 1 {
            panic!("Task creation failed");
        }
    }
}
