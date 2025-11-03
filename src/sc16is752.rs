use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{OutputPin, PinDriver},
    peripheral::Peripheral,
    spi::{SpiDeviceDriver, SpiDriver},
    sys::EspError,
    uart::config::StopBits,
    units::Hertz,
};
use std::{borrow::Borrow, cell::RefCell, num::NonZero, rc::Rc};

#[derive(Debug, Copy, Clone)]
pub enum SC16IS752Registers {
    RHR_THR_DLL = 0x0, // Receive Holding Register / Transmit Holding Register / Divisor Latch LSB
    IER_DLH = 0x1,     // Interrupt Enable Register / Divisor Latch MSB
    FCR_IIR = 0x2,     // FIFO Control Register / Interrupt Identification Register
    LCR = 0x3,         // Line Control Register
    MCR = 0x4,         // Modem Control Register
    LSR = 0x5,         // Line Status Register
    MSR_TCR = 0x6,     // Modem Status Register / Transmission Control Register
    SPR_TLR = 0x7,     // Scratchpad Register / Trigger Level Register
    TXLVL = 0x8,       // Transmit Level Register
    RXLVL = 0x9,       // Receive Level Register
    IODir = 0xA,       // I/O Direction Register
    IOState = 0xB,     // I/O State Register
    IOIntEna = 0xC,    // I/O Interrupt Enable Register
    IOControl = 0xE,   // I/O Control Register
    EFCR = 0xF,        // Extended Feature Control Register
}

pub struct SC16IS752Device<'d, T, OutPin>
where
    T: Borrow<SpiDriver<'d>> + 'd,
    OutPin: OutputPin,
{
    spi_device: SpiDeviceDriver<'d, T>,
    reset_pin: PinDriver<'d, OutPin, esp_idf_hal::gpio::Output>,
    frequency: Hertz,
}

impl<'d, T, OutPin> SC16IS752Device<'d, T, OutPin>
where
    T: Borrow<SpiDriver<'d>> + 'd,
    OutPin: OutputPin,
{
    pub fn new(
        spi_device: SpiDeviceDriver<'d, T>,
        reset_pin: impl Peripheral<P = OutPin> + 'd,
        frequency: Hertz,
    ) -> Self
    where
        T: Borrow<SpiDriver<'d>> + 'd,
        Self: Sized,
    {
        Self {
            spi_device,
            reset_pin: PinDriver::output(reset_pin).unwrap(),
            frequency,
        }
    }

    // Writes a byte to a register
    pub fn write_register(
        &mut self,
        register_address: SC16IS752Registers,
        channel: bool,
        value: u8,
    ) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let write = true;
        let write_byte = if write { 0x00u8 } else { 0x80u8 };
        let spi_data: [u8; 2] = [
            write_byte | (((register_address as u8) << 3) | ((channel as u8) << 1)),
            value,
        ];

        let mut rx: [u8; 0] = [];
        let ret_val = self.spi_device.transfer(&mut rx, &spi_data);
        ret_val
    }

    // Writes a byte from a register
    pub fn read_register(
        &mut self,
        register_address: SC16IS752Registers,
        channel: bool,
    ) -> Result<u8, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let write = false;
        let write_byte = if write { 0x00u8 } else { 0x80u8 };
        let spi_data: [u8; 2] = [
            write_byte | (((register_address as u8) << 3) | ((channel as u8) << 1)),
            0xFFu8,
        ];

        let mut rx: [u8; 2] = [0u8; 2];
        match self.spi_device.transfer(&mut rx, &spi_data) {
            Ok(_) => Ok(rx[1]),
            Err(e) => Err(e),
        }
    }
    // TODO read and write function merge-helper?

    pub fn set_gpio_direction(&mut self, direction: u8) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        self.write_register(SC16IS752Registers::IODir, false, direction)
    }

    // Makes a hard reset
    pub fn hard_reset(&mut self) {
        self.reset_pin.set_low().unwrap();
        FreeRtos::delay_ms(500);
        self.reset_pin.set_high().unwrap();
    }
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum UARTParity {
    None,  // No parity
    Odd,   // Odd parity bit
    Even,  // Even pairty bit
    Space, // 0 parity bit
    Mark,  // 1 parity bit
}

#[derive(Debug, Copy, Clone)]
pub struct UARTConfig {
    pub baud_rate: u32,
    pub data_bits: u8,
    pub stop_bits: StopBits,
    pub parity: UARTParity,
}

pub struct SC16IS752UART<'d, T, OutPin>
where
    T: Borrow<SpiDriver<'d>> + 'd,
    OutPin: OutputPin,
{
    sc16is752: Rc<RefCell<SC16IS752Device<'d, T, OutPin>>>,
    uart_config: UARTConfig,
    channel: bool,
}

impl<'d, T, OutPin> SC16IS752UART<'d, T, OutPin>
where
    T: Borrow<SpiDriver<'d>> + 'd,
    OutPin: OutputPin,
{
    // Cretes a new instance and configures it
    pub fn new(
        sc16is752: Rc<RefCell<SC16IS752Device<'d, T, OutPin>>>,
        uart_config: UARTConfig,
        channel: bool,
    ) -> Result<Self, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        // Create instance
        let new_instance = Self {
            sc16is752,
            uart_config,
            channel,
        };

        // Configure Uart
        match new_instance.configure_uart() {
            Ok(_) => Ok(new_instance),
            Err(e) => Err(e),
        }
    }

    // Configures every part chosen in uart-config
    fn configure_uart(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        self.soft_reset().unwrap();

        self.set_word_length().unwrap();

        self.set_stop_bit().unwrap();

        self.set_parity().unwrap();

        self.set_baudrate().unwrap();

        self.enable_fifo().unwrap();

        self.enable_transmition()
    }

    // Sets word-length (data-bits). Only 5, 6, 7, 8 is supported
    fn set_word_length(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let mut temporary_lcr = borrowed_sc16is752
            .read_register(SC16IS752Registers::LCR, self.channel)
            .unwrap();

        let (lcr_0, lcr_1) = match self.uart_config.data_bits {
            5 => (0, 0),
            6 => (1, 0),
            7 => (0, 1),
            8 => (1, 1),
            _ => return Err(EspError::from_non_zero(NonZero::new(1).unwrap())),
        };

        temporary_lcr |= lcr_0 << 0;
        temporary_lcr |= lcr_1 << 1;

        borrowed_sc16is752.write_register(
            SC16IS752Registers::LCR,
            self.channel,
            temporary_lcr,
        )
    }

    // Sets baudrate to device
    fn set_baudrate(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        // Calulate Baudrate
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let mcr = borrowed_sc16is752
            .read_register(SC16IS752Registers::MCR, self.channel)
            .unwrap();
        let prescaler_value: u32 = match mcr >> 7 {
            0 => 1,
            1 => 4,
            _ => {
                return Err(EspError::from_non_zero(NonZero::new(1).unwrap())); // Unexpected prescaler value
            }
        };
        let upper_divisor: u32 = (borrowed_sc16is752.frequency / (prescaler_value)).into();
        let divisor: u16 = ((upper_divisor) / (16u32 * (self.uart_config.baud_rate as u32))) as u16;

        // Enable special Register set to set baudrate
        let temporary_lcr = borrowed_sc16is752
            .read_register(SC16IS752Registers::LCR, self.channel)
            .unwrap();

        borrowed_sc16is752
            .write_register(SC16IS752Registers::LCR, self.channel, temporary_lcr | 0x80)
            .unwrap();

        // Set Baudrate
        // LSB
        borrowed_sc16is752
            .write_register(SC16IS752Registers::RHR_THR_DLL, self.channel, divisor as u8)
            .unwrap();
        // MSB
        borrowed_sc16is752
            .write_register(
                SC16IS752Registers::IER_DLH,
                self.channel,
                (divisor >> 8) as u8,
            )
            .unwrap();

        // Disable special register set again
        borrowed_sc16is752.write_register(
            SC16IS752Registers::LCR,
            self.channel,
            temporary_lcr & 0x7F,
        )
    }

    // Sets parity type to device
    fn set_parity(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let mut temporary_lcr = borrowed_sc16is752
            .read_register(SC16IS752Registers::LCR, self.channel)
            .unwrap();

        let (parity_enable, parity_type, forced_parity_bit) = match self.uart_config.parity {
            UARTParity::None => (0, 0, 0),
            UARTParity::Odd => (1, 0, 0),
            UARTParity::Even => (1, 1, 0),
            UARTParity::Space => (1, 1, 1),
            UARTParity::Mark => (1, 0, 1),
            _ => return Err(EspError::from_non_zero(NonZero::new(1).unwrap())),
        };

        temporary_lcr |= parity_enable << 3;
        temporary_lcr |= parity_type << 4;
        temporary_lcr |= forced_parity_bit << 5;

        borrowed_sc16is752.write_register(
            SC16IS752Registers::LCR,
            self.channel,
            temporary_lcr,
        )
    }

    // Sets stop bit to device
    fn set_stop_bit(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let mut temporary_lcr = borrowed_sc16is752
            .read_register(SC16IS752Registers::LCR, self.channel)
            .unwrap();

        let stop_bit: u8 = match (self.uart_config.stop_bits, self.uart_config.data_bits) {
            (StopBits::STOP1, 5..=8) => 0,
            (StopBits::STOP1P5, 5) => 1,
            (StopBits::STOP2, 6..=8) => 1,
            _ => return Err(EspError::from_non_zero(NonZero::new(1).unwrap())),
        };

        temporary_lcr |= stop_bit << 2;

        borrowed_sc16is752.write_register(
            SC16IS752Registers::LCR,
            self.channel,
            temporary_lcr,
        )
    }

    // Makes a software reset
    pub fn soft_reset(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let reset_val: u8 = 1 << 3;
        // TODO dont override other bytes
        borrowed_sc16is752.write_register(SC16IS752Registers::IODir, self.channel, reset_val)
    }

    // Enabels FIFO-buffer for transmitting and receiving
    fn enable_fifo(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let frc_val = 0b00000111; // Enable FIFO, reset RX and TX FIFO

        borrowed_sc16is752.write_register(
            SC16IS752Registers::LCR,
            self.channel,
            frc_val,
        )
    }

    fn enable_transmition(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let mut efcr = borrowed_sc16is752
            .read_register(SC16IS752Registers::EFCR, self.channel)
            .unwrap();

        efcr |= 0b11111001; // Enable TX and RX

        borrowed_sc16is752.write_register(
            SC16IS752Registers::EFCR,
            self.channel,
            efcr,
        )
    }

    // This register reports the fill level of the receive FIFO, that is, the number of characters in the RX FIFO
    pub fn get_rx_fifo_fill_level(&self) -> Result<u8, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        borrowed_sc16is752.read_register(SC16IS752Registers::RXLVL, self.channel)
    }

    // This register reports the number of spaces available in the transmit FIFO
    pub fn get_tx_fifo_fill_level(&self) -> Result<u8, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        borrowed_sc16is752.read_register(SC16IS752Registers::TXLVL, self.channel)
    }

    // Reads a byte from RX-FIFO-Buffer
    pub fn read_byte(&self) -> Result<u8, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let available_bytes = self.get_rx_fifo_fill_level().unwrap();
        let channel = self.channel;
        if available_bytes == 0 {
            log::warn!("Device {channel}: Cannot read. No byte available in RX-buffer");
            return Err(EspError::from_non_zero(NonZero::new(1).unwrap()));
        } else {
            log::info!("Device {channel}: Available bytes in RX-buffer: {available_bytes}");
        }

        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        borrowed_sc16is752.read_register(SC16IS752Registers::RHR_THR_DLL, self.channel)
    }

    // Writes a byte to TX-FIFO-Buffer
    pub fn write_byte(&self, byte: u8) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        if self.get_tx_fifo_fill_level().unwrap() == 0 {
            let channel = self.channel;
            log::warn!("Device {channel}: Cannot write. No space left in TX-buffer");
            return Err(EspError::from_non_zero(NonZero::new(1).unwrap())); // Buffer full
        }

        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        borrowed_sc16is752.write_register(SC16IS752Registers::RHR_THR_DLL, self.channel, byte)
    }

    pub fn check_line_status(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        let lsr_val = borrowed_sc16is752
            .read_register(SC16IS752Registers::LSR, self.channel)
            .unwrap();

        if lsr_val & 1 != 0 {
            log::warn!("Data Ready");
        }
        else if lsr_val & 2 != 0 {
            return Err(EspError::from_non_zero(NonZero::new(1).unwrap())); // Overrun Error
        }
        else if lsr_val & 4 != 0 {
            return Err(EspError::from_non_zero(NonZero::new(1).unwrap())); // Parity Error
        }
        else if lsr_val & 8 != 0 {
            return Err(EspError::from_non_zero(NonZero::new(1).unwrap())); // Framing Error
        }
        else if lsr_val & 16 != 0 {
            log::warn!("Break Interrupt");
        }
        else if lsr_val & 32 != 0 {
            log::warn!("Transmitter Holding Register Empty");
        }
        else if lsr_val & 64 != 0 {
            log::warn!("Transmitter Empty");
        }
        else if lsr_val & 128 != 0 {
            return Err(EspError::from_non_zero(NonZero::new(1).unwrap())); // FIFO Error
        } else {
            // TODO debug
            log::info!("No Line Status Error");
        }

        Ok(())
    }
}
