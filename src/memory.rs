use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use num_traits::{cast::FromPrimitive, cast::ToPrimitive, Num, PrimInt};

use heapless::consts::*;
use heapless::Vec;

use core::convert::{From, Into, TryInto};
use core::ops::Deref;

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

    pub fn clear_all_data(&mut self) -> Result<(), ()> {
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.resize(9, 0).unwrap();

        for i in Self::MIN_MEMORY_ADDRESS..Self::MAX_MEMORY_ADDRESS + 1 {
            buf[0] = i;
            match self.i2c.write(Self::EEPROM_ADDRESS, buf.as_ref()) {
                Ok(_) => {}
                Err(_) => return Err(()),
            }
        }
        Ok(())
    }

    pub fn write_data<T: ToPrimitive>(&mut self, address: Address, data: T) -> Result<(), ()> {
        let mut buf = Vec::<u8, U16>::new();
        buf.push(address as u8).unwrap();

        let number: i64 = ToPrimitive::to_i64(&data).unwrap();
        buf.extend_from_slice(&number.to_ne_bytes()).unwrap();
        match self.i2c.write(Self::EEPROM_ADDRESS | 0x1, buf.as_ref()) {
            Ok(_) => Ok(()),
            Err(_) => Err(()),
        }
    }

    pub fn read_data<T: PrimInt + Num + FromPrimitive>(
        &mut self,
        address: Address,
    ) -> Result<T, ()> {
        let size: usize = (T::zero().count_zeros() / 8) as usize;
        let buf: [u8; 1] = [address as u8];
        let mut out_buf = Vec::<u8, U8>::new();
        out_buf.resize(size, 0).ok();

        match self
            .i2c
            .write_read(Self::EEPROM_ADDRESS, &buf, out_buf.as_mut())
        {
            Ok(_) => {}
            Err(_) => return Err(()),
        }

        let temp = i64::from_ne_bytes(out_buf[..].try_into().unwrap());
        let out: T = FromPrimitive::from_i64(temp).unwrap();
        Ok(out)
    }

    pub fn write_bool(&mut self, address: Address, data: bool) -> Result<(), ()> {
        self.write_data(address, data as u8)
    }

    pub fn read_bool(&mut self, address: Address) -> Result<bool, ()> {
        let temp: u8 = self.read_data(address)?;
        return Ok(temp != 0);
    }
}
