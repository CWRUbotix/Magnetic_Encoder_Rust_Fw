use embedded_hal::blocking::i2c::{Read, Write, WriteIter, WriteIterRead, WriteRead};

use num_traits::{cast::FromPrimitive, cast::ToPrimitive, Num, PrimInt};

use heapless::consts::*;
use heapless::Vec;

use core::convert::{From, Into, TryInto};
use core::iter::IntoIterator;
use core::ops::Deref;

use defmt::unwrap;
use defmt::Format;

use rtic::cyccnt::Instant;
use rtic::cyccnt::U32Ext as _;

pub struct Memory<I2C> {
    i2c: I2C,
    last_op: Instant,
}

impl<I2C> Memory<I2C> {
    const EEPROM_ADDRESS: u8 = 0b1010111; // last bit signals read or write
    const MIN_MEMORY_ADDRESS: u8 = 0;
    const MAX_MEMORY_ADDRESS: u8 = 127;

    const R_FLAG: u8 = 0b1;
    const W_FLAG: u8 = 0b0;

    // these are word addresses
    pub const HAS_DATA_ADDR: u8 = 0x00;
    pub const TICK_ADDR: u8 = 0x01;
    pub const POL_ADDR: u8 = 5;
    pub const ABS_OFF_ADDR: u8 = 6;
}

use core::fmt::Debug;

impl<I2C, E> Memory<I2C>
where
    I2C: Write<Error = nb::Error<E>> + WriteRead<Error = nb::Error<E>>,
    E: Debug,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            last_op: Instant::now(),
        }
    }

    pub fn clear_all_data(&mut self) -> Result<(), ()> {
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.resize(9, 0).unwrap();

        for i in Self::MIN_MEMORY_ADDRESS..Self::MAX_MEMORY_ADDRESS + 1 {
            buf[0] = i;
            self.wait_from(self.last_op);
            nb::block!(self.i2c.write(Self::EEPROM_ADDRESS, buf.as_mut())).unwrap();
            self.last_op = Instant::now();
        }
        Ok(())
        // let (_ticks, _polarity, _abs_offset): (i32, bool, u16) =
    }

    pub fn write_data<T: ToPrimitive>(&mut self, address: u8, data: T) -> Result<(), ()> {
        let mut buf = Vec::<u8, U16>::new();
        buf.push(address).unwrap();

        let number: i64 = ToPrimitive::to_i64(&data).unwrap();

        buf.extend_from_slice(&number.to_ne_bytes()).unwrap();

        self.wait_from(self.last_op);

        defmt::debug!(
            "Writing data to eeprom address ({:u8}): {:[u8]}",
            buf[0],
            buf[1..]
        );

        self.last_op = Instant::now();

        nb::block!(self.i2c.write(Self::EEPROM_ADDRESS, buf.as_mut())).unwrap();

        defmt::debug!("Write sucessfull");

        Ok(())
    }

    pub fn read_data<T: PrimInt + Num + FromPrimitive + Format>(
        &mut self,
        address: u8,
    ) -> Result<T, ()> {
        let mut in_buf = Vec::<u8, U4>::new();
        let mut out_buf = Vec::<u8, U8>::new();

        in_buf.push(address).unwrap();
        out_buf.resize(8, 0).unwrap();

        defmt::debug!("Reading from eeprom address = {:[u8]}", in_buf.as_ref());

        // make sure we've waited long enough
        self.wait_from(self.last_op);

        nb::block!(self
            .i2c
            .write_read(Self::EEPROM_ADDRESS, in_buf.as_ref(), out_buf.as_mut()))
        .unwrap();

        self.last_op = Instant::now();

        defmt::debug!("Got back result = {:[u8]}", out_buf.as_ref());

        let temp = i64::from_ne_bytes(out_buf[..].try_into().unwrap());
        let out: T = unwrap!(FromPrimitive::from_i64(temp));

        defmt::debug!("{:?}", out);

        Ok(out)
    }

    pub fn write_bool(&mut self, address: u8, data: bool) -> Result<(), ()> {
        self.write_data(address, data as u8)
    }

    pub fn read_bool(&mut self, address: u8) -> Result<bool, ()> {
        let temp: u8 = { self.read_data(address)? };
        Ok(temp != 0)
    }

    #[inline(always)]
    fn wait(&mut self) {
        self.wait_from(Instant::now());
    }

    /// method to wait until the next spi operation will actually work
    /// this allows us to call as many mem operations as we want
    /// without running into bus timing issues in the main file (so no spin loops)
    /// outside of this class
    /// the data sheet says that delay is around 1200 ns minimum.
    /// in actuality, this should wait around 4000 ns I think?
    #[inline(never)]
    fn wait_from(&mut self, from: rtic::cyccnt::Instant) {
        use super::SYS_CLOCK_MHZ;
        // I have no idea why this particular number works, but it does.
        // basically just trial and errored it to the lowest value it could go
        // if you get "Ack" panics when you deploy the firmware, increase the duration
        while from.elapsed() < (SYS_CLOCK_MHZ * 4_000).cycles() {
            core::sync::atomic::spin_loop_hint();
        }
    }
}
