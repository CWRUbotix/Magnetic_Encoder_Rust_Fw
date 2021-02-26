use stm32f1xx_hal::spi::Error;
use stm32f1xx_hal::spi::FullDuplex;
use stm32f1xx_hal::spi::Spi;

use bitfield::bitfield;

// bitfield! {
// pub struct Settings1(u32);
// impl Debug;
// iwidth, _ :
// }

#[derive(Copy, Clone, defmt::Format)]
#[repr(u16)]
pub enum Address {
    // Volatile
    NOP = 0x0000,
    ERRFL = 0x0001,
    PROG = 0x0003,
    DIAAGC = 0x3FFC,
    MAG = 0x3FFD,
    ANGLEUNC = 0x3FFE,
    ANGLECOM = 0x3FFF,

    // Non-Volatile
    ZPOSM = 0x0016,
    ZPOSL = 0x0017,
    SETTINGS1 = 0x0018,
    SETTINGS2 = 0x0019,
    RED = 0x001A,
}

impl From<Address> for u16 {
    fn from(a: Address) -> Self {
        a as u16
    }
}

fn generate_control_frame(read: bool, address: u16) -> u16 {
    let read = read as u16;
    let frame = address | read << 14;
    let parity = frame.count_ones() as u16 % 2;
    frame | (parity << 15)
}

pub trait SpiExt<T>: FullDuplex<T> {
    fn read(&mut self, address: Address) -> Result<T, Error>;
    fn write(&mut self, address: Address, data: T) -> Result<(), Error>;
}

impl<SPI, REMAP, PINS> SpiExt<u16> for Spi<SPI, REMAP, PINS, u16>
where
    SPI: core::ops::Deref<Target = stm32f1xx_hal::pac::spi1::RegisterBlock>,
{
    fn read(&mut self, address: Address) -> Result<u16, Error> {
        use FullDuplex;
        let control_frame = generate_control_frame(true, address.into());
        nb::block!(self.send(control_frame))?;
        match nb::block!(FullDuplex::read(self)) {
            Ok(r) => {
                let err = (r >> 14) & 1 == 1;
                defmt::debug_assert!(!err);
                Ok(r & !(0b11 << 14))
            }
            Err(e) => Err(e),
        }
    }

    fn write(&mut self, address: Address, data: u16) -> Result<(), Error> {
        use FullDuplex;
        let control_frame = generate_control_frame(false, address.into());
        nb::block!(self.send(control_frame))?;
        nb::block!(self.send(data))?;

        let ctrl = nb::block!(FullDuplex::read(self))?;

        defmt::debug_assert!((ctrl >> 14) & 1 == 0);

        let _ = nb::block!(FullDuplex::read(self))?;

        Ok(())
    }
}
