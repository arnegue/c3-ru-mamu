//! Serial communication (USART)
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for transmission (TX)
//! - Input GPIO pin for reception (RX)
//!
//! The timer must be configured to twice the desired communication frequency.
//!

use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Input, Level, Output, PinDriver},
    peripheral::Peripheral,
    prelude::Peripherals,
    timer::{self, Timer, TimerConfig, TimerDriver},
};
use esp_idf_sys::{esp_rom_delay_us, gpio_set_level};
//extern gptimer_handle_t clk_timer{}

#[derive(PartialEq)]
pub enum ParityType {
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2,
    PARITY_SPACE = 3,
    PARTY_MARK = 4,
}

#[derive(Clone, Copy)]
pub enum BAUD_RATE {
    BAUD_RATE_4800 = 4800,
    BAUD_RATE_9600 = 9600,
    BAUD_RATE_19200 = 19200,
    BAUD_RATE_38400 = 38400,
    BAUD_RATE_57600 = 57600,
    BAUD_RATE_115200 = 115200,
    BAUD_RATE_125000 = 125000,
}

pub struct BitBangedUART<RX: esp_idf_hal::gpio::Pin, TX: esp_idf_hal::gpio::Pin> {
    rx_pin: PinDriver<'static, RX, Input>,  // RX Pin for UART
    tx_pin: PinDriver<'static, TX, Output>, // TX Pin for UART
    baud_rate: BAUD_RATE,                   // Baud rate for communication
    parity: ParityType,                     // Parity Type
    data_frame_size: u32,                   // Data frame size in bits (5-8)

    sleep_time_per_bit_ms: u32, // Waiting time before sending another bit
}

/// Serial communication error type
#[derive(Debug)]
pub enum Error {
    /// Bus error
    ParityError,
    SendError,
}

/// Bit banging serial communication (USART) device

impl<RX: esp_idf_hal::gpio::Pin, TX: esp_idf_hal::gpio::Pin> BitBangedUART<RX, TX> {
    /// Create instance

    // Creates new instance
    pub fn new(
        rx_pin: PinDriver<'static, RX, Input>,
        tx_pin: PinDriver<'static, TX, Output>,
        baud_rate: BAUD_RATE,
        parity: ParityType,
        data_frame_size: u32,
    ) -> Self {
        let sleep_time_per_bit_ms = (1000u32 * 1000u32) / baud_rate as u32;
        log::info!("SleepTime: {sleep_time_per_bit_ms} ms");

        Self {
            rx_pin,
            tx_pin,
            baud_rate,
            parity,
            data_frame_size,
            sleep_time_per_bit_ms: sleep_time_per_bit_ms,
        }
    }

    #[inline]
    fn wait_for_timer(&mut self) {
        unsafe { esp_rom_delay_us(self.sleep_time_per_bit_ms) };
    }

    pub fn write(&mut self, byte: u8) -> Result<(), Error> {
        let mut data_out = byte;

        // start bit
        self.tx_pin.set_low();
        self.wait_for_timer();

        // data
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.tx_pin.set_high();
            } else {
                self.tx_pin.set_low();
            }
            data_out >>= 1;
            self.wait_for_timer();
        }

        // parity
        match self.parity {
            ParityType::PARITY_NONE => {
                // Do nothing
            }
            ParityType::PARITY_ODD => {
                let parity = byte.count_ones() % 2;
                self.tx_pin.set_level((parity == 0).into());
                self.wait_for_timer();
            }
            ParityType::PARITY_EVEN => {
                let parity = byte.count_ones() % 2;
                self.tx_pin.set_level((parity == 1).into());
                self.wait_for_timer();
            }
            ParityType::PARITY_SPACE => {
                self.tx_pin.set_low();
                self.wait_for_timer();
            }
            ParityType::PARTY_MARK => {
                self.tx_pin.set_high();
                self.wait_for_timer();
            }
        }

        // stop bit
        self.tx_pin.set_high();
        self.wait_for_timer();
        Ok(())
    }

    pub fn read(&mut self) -> Result<u8, Error> {
        let mut data_in = 0;

        // wait for start bit
        while self.rx_pin.is_high() {}
        self.wait_for_timer();

        // wait for data
        for _bit in 0..8 {
            data_in <<= 1;
            if self.rx_pin.is_high() {
                data_in |= 1
            }
            self.wait_for_timer();
        }

        // wait for parity
        if self.parity != ParityType::PARITY_NONE{
            // TODO ignore parity value for now
            self.wait_for_timer();

        }

        // wait for stop bit
        self.wait_for_timer();
        Ok(data_in)
    }
}
