use stm32f1xx_hal::gpio::ExtiPin;
use stm32f1xx_hal::spi::{FullDuplex, SpiReadWrite};

use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::*;

use volatile::Volatile;

#[allow(dead_code)]
#[derive(Copy, Clone)]
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

pub struct Encoder<A, B, I, SPI> {
    enc_a: A,
    enc_b: B,
    enc_i: I,
    spi: SPI,

    ticks: Volatile<i32>,
    inverted: Volatile<bool>,
    prev_gpio_value: i32,
}

impl<A, B, I, SPI> Encoder<A, B, I, SPI>
where
    A: ExtiPin + InputPin,
    B: ExtiPin + InputPin,
    I: ExtiPin + InputPin,
    SPI: FullDuplex<u16> + SpiReadWrite<u16>,
{
    const LUT: [i32; 16] = [0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0];

    pub fn new(enc_a: A, enc_b: B, enc_i: I, spi: SPI) -> Self {
        Self {
            enc_a,
            enc_b,
            enc_i,
            spi,
            ticks: Volatile::new(0),
            inverted: Volatile::new(false),
            prev_gpio_value: 0,
        }
    }
}
