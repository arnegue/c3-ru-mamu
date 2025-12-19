use esp_idf_hal::sys::xTaskCreatePinnedToCore;
use esp_idf_hal::uart::UartDriver;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_sys::TaskHandle_t;
use std::{ffi::CString, ptr};

use crate::utils::buffer_to_string;

// UART-Task which reads and writes from/to UART
extern "C" fn uart_task(param: *mut core::ffi::c_void) {
    // Task-Parameters
    let uart_driver: Box<UartDriver>;
    unsafe {
        // Unbox UART from raw pointer
        uart_driver = Box::from_raw(param as *mut _);
    }

    let mut read_buf: [u8; 255] = [0; 255];
    loop {
        match uart_driver.read(&mut read_buf, TickType::new_millis(1000).ticks()) {
            Ok(amount_bytes) => {
                if amount_bytes > 0 {
                    log::info!(
                        "Read {} bytes_ {}",
                        amount_bytes,
                        buffer_to_string(&read_buf, amount_bytes)
                    );
                } else {
                    log::debug!("Timeout when reading from UART-Task");
                }
            }
            Err(esp_err) => {
                let s = format!("{:?}", esp_err);
                log::error!("Error in UART-Read-Task: {}", s);
            }
        }
    }
}

pub fn start_uart_task(uart_driver: UartDriver) {
    // Box it so ownership can be transferred
    let task_param_box = Box::new(uart_driver);
    // Convert into raw pointer for FreeRTOS
    let task_param_ptr = Box::into_raw(task_param_box);

    unsafe {
        // Start task
        let mut task_handle: TaskHandle_t = ptr::null_mut();
        let res = xTaskCreatePinnedToCore(
            Some(uart_task),
            CString::new("UART Task").unwrap().as_ptr(),
            5000,
            task_param_ptr as *mut core::ffi::c_void,
            10,
            &mut task_handle,
            0,
        );

        if res != 1 {
            panic!("Task creation failed");
        }
    }
}
