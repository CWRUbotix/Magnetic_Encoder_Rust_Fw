use stm32f1xx_hal::gpio::ExtiPin;
use stm32f1xx_hal::spi::{FullDuplex, SpiReadWrite};

use stm32f1xx_hal::afio;
use stm32f1xx_hal::pac::{AFIO, EXTI};

use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::*;

use defmt;

use core::fmt::Debug;

use volatile::Volatile;

use bitfield::bitfield;

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

bitfield! {
    pub struct Settings1(u16);
    impl Debug;

    u8;
    pub iwidth, set_iwidth: 0;
    pub noiseset, set_noiseset: 1;
    pub dir, set_dir: 2;
    pub uvw_abi, set_ubw_abi: 3;
    pub daecdis, set_daecdis: 4,5;
    pub dataselect, set_dataselect: 6;
    pub pwmon, set_pwmon: 7;
}

bitfield! {
    pub struct Settings2(u16);
    impl Debug;

    pub data, set_data: 15, 0;
}

bitfield! {
    pub struct Zpos(u16);
    impl Debug;

    pub error_low, set_error_low: 6;
    pub error_high, set_error_high: 7;
    pub offset, set_offset: 2, 15;
}

pub struct Encoder<A, B, I, SPI> {
    enc_a: A,
    enc_b: B,
    enc_i: I,
    spi: SPI,

    ticks: Volatile<i32>,
    inverted: Volatile<bool>,
    prev_gpio_value: u32,
}

impl<A, B, I, SPI> Encoder<A, B, I, SPI>
where
    A: ExtiPin + InputPin,
    B: ExtiPin + InputPin,
    I: ExtiPin + InputPin,
    <A as InputPin>::Error: Debug,
    <B as InputPin>::Error: Debug,
    <I as InputPin>::Error: Debug,
    SPI: FullDuplex<u16> + SpiReadWrite<u16>,
{
    const LUT: [i32; 16] = [0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0];

    pub fn new(
        mut enc_a: A,
        mut enc_b: B,
        #[allow(unused_mut)] mut enc_i: I,
        spi: SPI,
        exti: &EXTI,
        afio: &mut afio::Parts,
    ) -> Self {
        enc_a.make_interrupt_source(afio);
        enc_b.make_interrupt_source(afio);
        // enc_i.make_interrupt_source(afio);

        enc_a.trigger_on_edge(&exti, stm32f1xx_hal::gpio::Edge::RISING_FALLING);
        enc_b.trigger_on_edge(&exti, stm32f1xx_hal::gpio::Edge::RISING_FALLING);
        // enc_i.trigger_on_edge(&exti, stm32f1xx_hal::gpio::Edge::RISING);

        enc_a.enable_interrupt(&exti);
        enc_b.enable_interrupt(&exti);
        // enc_i.enable_interrupt(&exti);

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

    pub fn has_update(&mut self) -> bool {
        self.enc_a.check_interrupt() || self.enc_b.check_interrupt()
    }

    pub fn reset_interrupts(&mut self) {
        self.enc_a.clear_interrupt_pending_bit();
        self.enc_b.clear_interrupt_pending_bit();
        self.enc_i.clear_interrupt_pending_bit();
    }

    pub fn update(&mut self) {
        defmt::debug!(
            "Enc interrupt states: {:bool}, {:bool}, {:bool}",
            self.enc_a.check_interrupt(),
            self.enc_b.check_interrupt(),
            self.enc_i.check_interrupt(),
        );

        let a: u32 = self.enc_a.is_high().unwrap().into();
        let b: u32 = self.enc_b.is_high().unwrap().into();
        let i: u32 = self.enc_i.is_high().unwrap().into();

        defmt::assert!(a == 0 || a == 1);
        defmt::assert!(b == 0 || b == 1);
        defmt::assert!(i == 0 || i == 1);

        defmt::info!(
            "Updating encoder with values {:u32}, {:u32}, and {:u32}",
            a,
            b,
            i
        );

        // we dont care about this channel for now
        if self.enc_i.check_interrupt() {
            defmt::debug!("Skipping enc I interrupt");
            self.enc_i.clear_interrupt_pending_bit();
            return;
        }

        // check if this interrupt is actually for the encoder
        if self.enc_a.check_interrupt() || self.enc_b.check_interrupt() {
        } else {
            defmt::debug!("No interrupts");
            return;
        };

        let mut gpio_state = 0;
        gpio_state |= a;
        gpio_state |= b << 1;
        gpio_state &= 0b11;

        let idx = self.prev_gpio_value * 4 + gpio_state;
        if idx as usize >= Self::LUT.len() {
            defmt::error!("Overflow of encoder lookup table");
            return;
        }

        let increment = Self::LUT[idx as usize];

        if increment == 2 {
            defmt::warn!(
                "Bad encoder increment. Encoder is either spinning too fast or broken (or theres a bug)"
            );
            self.prev_gpio_value = 0;
        } else {
            let ticks = self.ticks.read();
            let invert = if self.inverted.read() { -1 } else { 1 };
            self.ticks.write(ticks + increment * invert);
            self.prev_gpio_value = gpio_state;
        }

        self.enc_a.clear_interrupt_pending_bit();
        self.enc_b.clear_interrupt_pending_bit();
    }
}
