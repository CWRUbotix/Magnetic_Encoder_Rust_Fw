use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use num_traits::{cast::FromPrimitive, cast::ToPrimitive, Num, PrimInt};

use heapless::consts::*;
use heapless::Vec;

pub enum Address {}

pub struct Memory<I2C> {
    i2c: I2C,
}

impl<I2C, E> Memory<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    const EEPROM_ADDRESS: u8 = 0b10101110; // last bit signals read or write
    const MIN_MEMORY_ADDRESS: u8 = 0;
    const MAX_MEMORY_ADDRESS: u8 = 127;

    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub fn clear_all_data(&mut self) {
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.resize(9, 0).unwrap();

        for i in Self::MIN_MEMORY_ADDRESS..Self::MAX_MEMORY_ADDRESS + 1 {
            buf[0] = i;
            self.i2c.write(Self::EEPROM_ADDRESS, buf.as_ref()).ok();
        }
    }

    pub fn write_data<T: Num + PrimInt + ToPrimitive>(&mut self, address: Address) {
        let mut buf = Vec::<u8, U16>::new();
        buf.push(address as u8).unwrap();
    }
}
