use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Output, Pin, PinDriver},
    sys::xTaskCreatePinnedToCore,
};
use std::{ffi::CString, ptr};

const TASK_NAME: &str = "LED-Task";

extern "C" fn led_task<T: Pin>(param: *mut core::ffi::c_void) {
    unsafe {
        // Cast back to the full PinDriver type
        let led: &mut PinDriver<'static, T, Output> =
            &mut *(param as *mut PinDriver<'static, T, Output>);
        loop {
            led.toggle().unwrap();
            log::debug!("{TASK_NAME}: Toggled LED");
            FreeRtos::delay_ms(500);
        }
    }
}

pub fn start_led_task<T: Pin>(pin: PinDriver<'static, T, Output>) {
    let pin_box = Box::new(pin);
    let pin_raw = Box::into_raw(pin_box); // leak into raw pointer

    unsafe {
        let res = xTaskCreatePinnedToCore(
            Some(led_task::<T>),
            CString::new(TASK_NAME).unwrap().as_ptr(),
            1000,
            pin_raw as *mut core::ffi::c_void,
            10,
            ptr::null_mut(),
            0,
        );

        if res != 1 {
            panic!("{TASK_NAME}: Task creation failed");
        }
    }
}
