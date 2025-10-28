use esp_idf_hal::{
    spi::{SpiDeviceDriver, SpiDriver},
    sys::EspError,
    uart::config::Parity,
};
use std::borrow::Borrow;

pub struct SC16IS752Device<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    spi_device: SpiDeviceDriver<'d, T>,
}

impl<'d, T> SC16IS752Device<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(spi_device: SpiDeviceDriver<'d, T>) -> Self
    where
        T: Borrow<SpiDriver<'d>> + 'd,
        Self: Sized,
    {
        Self {
            spi_device: spi_device,
        }
    }

    pub fn write_register( // todo private
        &mut self,
        register_address: u8,
        channel: u8,
        value: u8,
    ) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let write = true;
        let write_byte = if write { 0x00u8 } else { 0x80u8 };
        let spi_data: [u8; 2] = [
            write_byte | ((register_address << 3) | (channel << 1)),
            value,
        ];

        let mut rx: [u8; 0] = [];
        let ret_val = self.spi_device.transfer(&mut rx, &spi_data);
        ret_val
    }

    fn read_register( // todo private
        &mut self,
        register_address: u8,
        channel: u8,
    ) -> Result<(), EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        let write = false;
        let write_byte = if write { 0x00u8 } else { 0x80u8 };
        let spi_data: [u8; 2] = [
            write_byte | ((register_address << 3) | (channel << 1)),
            0xFFu8,
        ];

        let mut rx: [u8; 2] = [0u8; 2];
        let ret_val = self.spi_device.transfer(&mut rx, &spi_data);
        ret_val
    }
}

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
    sc16is752: SC16IS752Device<'d, T>,
    uart_config: UARTConfig,
}

impl<'d, T> SC16IS752UART<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(sc16is752: SC16IS752Device<'d, T>, uart_config: UARTConfig) -> Result<Self, EspError>
    where
        T: Borrow<SpiDriver<'d>> + 'd,
    {
        Ok(Self {
            sc16is752,
            uart_config,
        })
    }
}
