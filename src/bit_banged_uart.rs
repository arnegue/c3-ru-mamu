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
    gpio::{Input, Output, PinDriver},
    units::Hertz,
};
use esp_idf_sys::esp_rom_delay_us;

#[derive(PartialEq)]
pub enum ParityType {
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2,
    PARITY_SPACE = 3,
    PARTY_MARK = 4,
}

pub struct BitBangedUART<RX: esp_idf_hal::gpio::Pin, TX: esp_idf_hal::gpio::Pin> {
    rx_pin: PinDriver<'static, RX, Input>,  // RX Pin for UART
    tx_pin: PinDriver<'static, TX, Output>, // TX Pin for UART
    parity: ParityType,                     // Parity Type
    data_frame_size: u8,                    // Data frame size in bits (0-16, anything that fits into u16)

    sleep_time_per_bit_ms: u32, // Waiting time before sending another bit
}

/// Bit banging serial communication (USART) device
impl<RX: esp_idf_hal::gpio::Pin, TX: esp_idf_hal::gpio::Pin> BitBangedUART<RX, TX> {
    // Creates new instance
    pub fn new(
        rx_pin: PinDriver<'static, RX, Input>,
        tx_pin: PinDriver<'static, TX, Output>,
        baud_rate: Hertz,
        parity: ParityType,
        data_frame_size: u8,
    ) -> Self {
        let sleep_time_per_bit_ms = (1000u32 * 1000u32) / u32::from(baud_rate) as u32;
        log::info!("SleepTime: {sleep_time_per_bit_ms} ms");

        Self {
            rx_pin,
            tx_pin,
            parity,
            data_frame_size,
            sleep_time_per_bit_ms: sleep_time_per_bit_ms,
        }
    }

    // Sends given bit and waits corresponding time
    #[inline]
    fn send_bit(&mut self, value: bool) {
        let _ = self.tx_pin.set_level(value.into());
        self.wait_for_timer();
    }

    // Sleep function used between transmission/reception of bits
    #[inline]
    fn wait_for_timer(&mut self) {
        unsafe { esp_rom_delay_us(self.sleep_time_per_bit_ms) };
    }

    // Writes given word
    pub fn write(&mut self, byte: u16) {
        let mut data_out = byte;

        // start bit
        self.send_bit(false);

        // data
        for _bit in 0..self.data_frame_size {
            self.send_bit(data_out & 1 == 1);
            data_out >>= 1;
        }

        // parity
        match self.parity {
            ParityType::PARITY_NONE => {
                // Do nothing
            }
            ParityType::PARITY_ODD => {
                let parity = byte.count_ones() % 2;
                self.send_bit(parity == 0);
            }
            ParityType::PARITY_EVEN => {
                let parity = byte.count_ones() % 2;
                self.send_bit(parity == 1);
            }
            ParityType::PARITY_SPACE => {
                self.send_bit(false);
            }
            ParityType::PARTY_MARK => {
                self.send_bit(true);
            }
        }

        // stop bit
        self.send_bit(true);
    }

    // Reads a word
    pub fn read(&mut self) -> u16 {
        let mut data_in = 0;

        // wait for start bit
        while self.rx_pin.is_high() {}
        self.wait_for_timer();

        // wait for data
        for _bit in 0..self.data_frame_size {
            data_in <<= 1;
            if self.rx_pin.is_high() {
                data_in |= 1
            }
            self.wait_for_timer();
        }

        // wait for parity
        if self.parity != ParityType::PARITY_NONE {
            // TODO ignore parity value for now
            self.wait_for_timer();
        }

        // wait for stop bit
        self.wait_for_timer();
        data_in
    }
}
