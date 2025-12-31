use esp_idf_hal::{
    gpio::{Input, Level, Output, PinDriver},
    peripheral::Peripheral,
    prelude::Peripherals,
    timer::{self, Timer, TimerConfig, TimerDriver},
};
use esp_idf_sys::gpio_set_level;
//extern gptimer_handle_t clk_timer{}

#[derive(PartialEq)]
enum ParityType {
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2,
}

#[derive(Clone, Copy)]
enum BAUD_RATE {
    BAUD_RATE_4800 = 208,
    BAUD_RATE_9600 = 104,
    BAUD_RATE_19200 = 52,
    BAUD_RATE_38400 = 26,
    BAUD_RATE_57600 = 17,
    BAUD_RATE_115200 = 9,
    BAUD_RATE_125000 = 8,
}

struct UART<T: esp_idf_hal::gpio::Pin> {
    rx_pin: PinDriver<'static, T, Input>,  // RX Pin for UART
    tx_pin: PinDriver<'static, T, Output>, // TX Pin for UART
    baud_rate: BAUD_RATE,                  // Baud rate for communication
    parity: ParityType,                    // Parity Type
    data_frame_size: u32,                  // Data frame size in bits (5-8)

    // Static variables to hold the state of the protocol
    elapsed_cycles: u8,
    byte_to_send: u8,
    parity_value: u8,
    timer_active: bool,
    received_byte: u8,

    timer_driver: TimerDriver<'static>,
}
fn timer_callback() {
    match self.elapsed_cycles {
        0 => self.send_start_bit(),
        1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 => {
            self.send_bit(self.byte_to_send & 0x01);
            self.parity_value ^= self.byte_to_send & 0x01;
            self.byte_to_send >>= 1;
        }
        9 => {
            if self.parity != ParityType::PARITY_NONE {
                self.send_parity_bit();
            }
            self.send_stop_bit();
            self.parity_value = 0;
            self.elapsed_cycles = 0;
            self.timer_active = false;
        }
        10 => {
            self.send_stop_bit();
            self.parity_value = 0;
            self.elapsed_cycles = 0;
            self.timer_active = false;
        }
        _ => {}
    }

    self.elapsed_cycles += 1;
}

/*UART PROTOTYPES*/
impl<T: esp_idf_hal::gpio::Pin> UART<T> {
    // Public functions

    // Creates new instance
    pub fn new<TIMER: Timer>(
        rx_pin: PinDriver<'static, T, Input>,
        tx_pin: PinDriver<'static, T, Output>,
        baud_rate: BAUD_RATE,
        parity: ParityType,
        data_frame_size: u32,
        _timer: impl Peripheral<P = TIMER> + 'static,
    ) -> Self {
        let timer_config: TimerConfig = TimerConfig::new().auto_reload(true).divider(80);
        let timer_driver: TimerDriver = TimerDriver::new(_timer, &timer_config).unwrap();

        Self {
            rx_pin,
            tx_pin,
            baud_rate,
            parity,
            data_frame_size,
            elapsed_cycles: 0,
            byte_to_send: 0,
            parity_value: 0,
            timer_active: false,
            received_byte: 0,
            timer_driver,
        }
    }

    // Initializes gpio, timers, etc
    pub fn init<TIMER: Timer>(&mut self) {
        // Dereference cfg to access the configuration settings

        // TX IDLE STATE
        self.send_bit(1);

        let _ = self.timer_driver.set_alarm(self.baud_rate as u64);

        // Register callback
        unsafe {
            let _ = self.timer_driver.subscribe_nonstatic(move || {
                let _s = self.elapsed_cycles;
            });
        };

        // Enable timer
        let _ = self.timer_driver.enable(true);

        //Call Transmit once during initialization to stabilize the timing
        let stable: [u8; 1] = [1];
        self.uart_transmit(&stable, 1);
    }

    // Sends given buffer
    pub fn uart_transmit(&mut self, data: &[u8], length: usize) {
        self.start_timer(self.baud_rate as u64);
        for i in 0..length {
            self.timer_active = true;
            self.elapsed_cycles = 0;
            self.set_data_frame(data[i]);
            while self.timer_active {}
        }
        self.end_timer();
    }

    // Receives and writes in given buffer
    pub fn uart_receive(&mut self, data: &mut [u8], length: usize) {
        for i in 0..length {
            /* 1 Detect Start Condition*/
            while self.rx_pin.is_high() {}

            /* 2 Start Timer such that we clock in data in the middle of a cycle*/
            self.start_timer((self.baud_rate as u64) / 2);

            /* 3 Take the bit and shift it into a uint8_t data variable*/
            for bit in 0..self.data_frame_size {
                while self.elapsed_cycles == 0 {}

                let current_bit = if self.rx_pin.get_level() == Level::Low {
                    0
                } else {
                    1
                };
                self.received_byte |= current_bit << bit;
                self.parity_value ^= current_bit;
                self.elapsed_cycles = 0;
            }

            /* 4 Check for parity bit if enabled*/
            if self.parity != ParityType::PARITY_NONE {
                while self.elapsed_cycles == 0 {}

                let parity_bit = if self.rx_pin.get_level() == Level::Low {
                    0
                } else {
                    1
                };

                if self.parity == ParityType::PARITY_EVEN && parity_bit != self.parity_value {
                    log::error!("Parity error (Even)");
                } else if self.parity == ParityType::PARITY_ODD && parity_bit == self.parity_value {
                    log::error!("Parity error (Odd)");
                }
                self.elapsed_cycles = 0;
            }

            /* 5 Check for Stop Condition*/
            while self.elapsed_cycles == 0 {}

            /* 6 Take the byte and place it in the data array*/
            data[i] = self.received_byte;

            self.elapsed_cycles = 0;
        }

        self.end_timer();
    }

    // private helper functions

    fn send_start_bit(&mut self) {
        /*UART pulls transmission line LOW for one clock cycle*/
        self.tx_pin.set_level(Level::Low);
    }

    fn set_data_frame(&mut self, byte: u8) {
        self.byte_to_send = byte;
    }

    fn send_bit(&mut self, bit: u8) {
        self.tx_pin
            .set_level(if bit > 0 { Level::High } else { Level::Low });
    }

    fn send_parity_bit(&mut self) {
        /*parity_value is 0 when there are an even amount of 1s*/
        if self.parity == ParityType::PARITY_EVEN {
            self.send_bit(self.parity_value);
        } else if self.parity == ParityType::PARITY_ODD {
            self.send_bit(!self.parity_value);
        }
    }

    fn send_stop_bit(&mut self) {
        /*UART pulls transmission line HIGH*/
        self.tx_pin.set_level(Level::High);
    }

    /*TIMER FUNCTIONS*/
    fn start_timer(&mut self, period: u64) {}
    fn end_timer(&mut self) {}
}
