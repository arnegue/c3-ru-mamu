use esp_idf_hal::{
    spi::{SpiDeviceDriver, SpiDriver},
    sys::EspError,
    uart::config::Parity,
    units::Hertz,
};
use std::{borrow::Borrow, cell::RefCell, rc::Rc};

#[derive(Debug, Copy, Clone)]
pub enum SC16IS752Registers {
    RHR_THR = 0x0,   // Receive Holding Register / Transmit Holding Register
    IER = 0x1,       // Interrupt Enable Register
    FCR_IIR = 0x2,   // FIFO Control Register / Interrupt Identification Register
    LCR = 0x3,       // Line Control Register
    MCR = 0x4,       // Modem Control Register
    LSR = 0x5,       // Line Status Register
    MSR_TCR = 0x6,   // Modem Status Register / Transmission Control Register
    SPR_TLR = 0x7,   // Scratchpad Register / Trigger Level Register
    TXLVL = 0x8,     // Transmit Level Register
    RXLVL = 0x9,     // Receive Level Register
    IODir = 0xA,     // I/O Direction Register
    IOState = 0xB,   // I/O State Register
    IOIntEna = 0xC,  // I/O Interrupt Enable Register
    IOControl = 0xE, // I/O Control Register
    EFCR = 0xF,      // Extended Feature Control Register
}

pub struct SC16IS752Device<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    spi_device: SpiDeviceDriver<'d, T>,
    frequency: Hertz,
}

impl<'d, T> SC16IS752Device<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(spi_device: SpiDeviceDriver<'d, T>, frequency: Hertz) -> Self
    where
        T: Borrow<SpiDriver<'d>> + 'd,
        Self: Sized,
    {
        Self {
            spi_device,
            frequency,
        }
    }

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
        let register_address = SC16IS752Registers::IODir;
        self.write_register(register_address, false, direction)
    }
}

#[derive(Debug, Copy, Clone)]
pub struct UARTConfig {
    pub baud_rate: u32,
    pub data_bits: u8,
    pub stop_bits: u8,
    pub parity: Parity,
}

pub struct SC16IS752UART<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    sc16is752: Rc<RefCell<SC16IS752Device<'d, T>>>,
    uart_config: UARTConfig,
    channel: bool,
}

impl<'d, T> SC16IS752UART<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(
        sc16is752: Rc<RefCell<SC16IS752Device<'d, T>>>,
        uart_config: UARTConfig,
        channel: bool,
    ) -> Result<Self, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        Ok(Self {
            sc16is752,
            uart_config,
            channel,
        })
        /*
        let register_address = SC16IS752Registers::IODir;
        let value = 0;
        match sc16is752.borrow_mut().write_register(register_address, channel, value) {
            Ok(_) => Ok(Self {
                sc16is752,
                uart_config,
                channel,
            }),
            Err(e) => Err(e),
        }*/
    }

    pub fn configure_uart(&mut self) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        // Set Baudrate
        let register_address = SC16IS752Registers::MCR;
        let prescaler: u8;
        let mut borrowed_sc16is752 = self.sc16is752.borrow_mut();
        match borrowed_sc16is752.read_register(SC16IS752Registers::MCR, false) {
            Ok(prescaler_val) => {
                prescaler = prescaler_val;
            }
            Err(e) => return Err(e),
        }

        let upper_divisor: u32 = (borrowed_sc16is752.frequency / (prescaler as u32)).into();
        let divisor: u16 = ((upper_divisor) / (16u32 * (self.uart_config.baud_rate as u32))) as u16;

        borrowed_sc16is752.write_register(register_address, self.channel, (divisor & 0xFF) as u8)
    }

    pub fn toggle_led(&mut self, state: u8) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        self.sc16is752.borrow_mut().set_gpio_direction(state)
    }
}
