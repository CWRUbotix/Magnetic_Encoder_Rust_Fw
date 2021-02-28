/// main file for encoder class
///
///
///
// use embedded_hal::spi::{self, FullDuplex};
use bitfield::bitfield;

use core::convert::From;
use core::convert::Into;

use getset::{Getters, Setters};

bitfield! {
    #[derive(Copy, Clone, defmt::Format, Default)]
    pub struct ControlFrame(u16);

    parity, set_parity : 15;
    pub read, set_read: 14;
    pub addr, set_addr: 13, 0;
}

impl Into<u16> for ControlFrame {
    fn into(self) -> u16 {
        self.0
    }
}

impl ControlFrame {
    pub fn calculate_parity(&mut self) {
        let ones = self.0.count_ones();
        self.set_parity(ones % 2 == 1);
    }
}

pub mod registers {

    use bitfield::bitfield;

    pub trait Register: Copy + Clone {
        const ADDRESS: u16;
    }

    // marker traits
    pub trait ReadRegister: Register + From<u16> {}
    pub trait WriteRegister: Register + Into<u16> {}

    macro_rules! impl_read_reg {
        ($ty: ty) => {
            impl From<u16> for $ty {
                fn from(val: u16) -> $ty {
                    Self(val)
                }
            }
            impl ReadRegister for $ty {}
        };
    }

    macro_rules! impl_write_reg {
        ($i: ty) => {
            impl Into<u16> for $i {
                fn into(self) -> u16 {
                    self.0
                }
            }
            impl WriteRegister for $i {}
        };
    }

    macro_rules! impl_read_write {
        ($i: ty) => {
            impl_write_reg!($i);
            impl_read_reg!($i);
        };
    }

    #[derive(Copy, Clone)]
    pub struct NOP(u16);

    impl Register for NOP {
        const ADDRESS: u16 = 0;
    }
    impl_read_reg!(NOP);
    impl_write_reg!(NOP);

    bitfield! {
        #[derive(Clone, Copy, defmt::Format)]
        pub struct ERRFL(u16);

        pub parerr, _ : 2;
        pub invcomm, _: 1;
        pub frerr, _: 0;
    }

    impl Register for ERRFL {
        const ADDRESS: u16 = 0x0001;
    }
    impl_read_reg!(ERRFL);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct PROG(u16);

        pub verify, set_verify: 6;
        pub start_otp, set_start_otp: 3;
        pub otp_refresh, set_opt_refresh: 2;
        pub enable, set_enable: 0;
    }

    impl Register for PROG {
        const ADDRESS: u16 = 0x0003;
    }
    impl_read_reg!(PROG);
    impl_write_reg!(PROG);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct DIAAGC(u16);

        pub magl, _ : 11;
        pub magh, _ : 10;
        pub cordic_overflow, _  : 9;
        pub loops_finished, _ : 8;
        pub agc_val, _ : 7, 0;
    }

    impl Register for DIAAGC {
        const ADDRESS: u16 = 0x3FFC;
    }

    impl_read_reg!(DIAAGC);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct MAG(u16);

        pub cordic_magnitude_info, _ : 13, 0;
    }

    impl Register for MAG {
        const ADDRESS: u16 = 0x3FFD;
    }

    impl_read_reg!(MAG);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct ANGLEUNC(u16);

        pub cordicang, _ : 13, 0;
    }

    impl Register for ANGLEUNC {
        const ADDRESS: u16 = 0x3FFE;
    }

    impl_read_reg!(ANGLEUNC);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct ANGLECOM(u16);

        pub daecang, _ : 13, 0;
    }

    impl Register for ANGLECOM {
        const ADDRESS: u16 = 0x3FFF;
    }

    impl_read_reg!(ANGLECOM);

    pub fn gen_zpos(offset: u16, error_high_mag: bool, error_low_mag: bool) -> (ZposL, ZposM) {
        let mut l = ZposL(0);
        let mut m = ZposM(0);

        l.set_comp_i_en(error_low_mag);
        l.set_comp_h_en(error_high_mag);

        let l_offset_mask = 0b111111;

        l.set_zposl(offset & l_offset_mask);
        m.set_zposm((offset & !l_offset_mask) >> 6);

        (l, m)
    }

    bitfield! {
        #[derive(Clone, Copy)]
        pub struct ZposM(u16);

        pub zposm, set_zposm: 7, 0;
    }

    impl Register for ZposM {
        const ADDRESS: u16 = 0x16;
    }

    impl_read_write!(ZposM);

    bitfield! {
        #[derive(Clone, Copy)]
        pub struct ZposL(u16);

        pub zposl, set_zposl: 5, 0;
        pub comp_i_error_en, set_comp_i_en: 6;
        pub comp_h_error_en, set_comp_h_en: 7;

    }

    impl Register for ZposL {
        const ADDRESS: u16 = 0x17;
    }

    impl_read_write!(ZposL);

    bitfield! {
        #[derive(Clone, Copy)]
        pub struct Settings1(u16);
    }

    impl Register for Settings1 {
        const ADDRESS: u16 = 0x18;
    }

    impl_read_write!(Settings1);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct Settings2(u16);
    }

    impl Register for Settings2 {
        const ADDRESS: u16 = 0x19;
    }

    impl_read_write!(Settings2);

    bitfield! {
        #[derive(Copy, Clone)]
        pub struct Red(u16);
    }

    impl Register for Red {
        const ADDRESS: u16 = 0x18;
    }

    impl_read_write!(Red);
}

use stm32f1xx_hal::gpio;

pub type MOSIPin = gpio::gpioa::PA5<gpio::Alternate<gpio::PushPull>>;
pub type MISOPin = gpio::gpioa::PA6<gpio::Input<gpio::Floating>>;
pub type SCKPin = gpio::gpioa::PA7<gpio::Alternate<gpio::PushPull>>;
pub type SPIPins = (MOSIPin, MISOPin, SCKPin);

use embedded_hal::digital::v2::InputPin;

use stm32f1xx_hal::afio;
use stm32f1xx_hal::gpio::{Edge, ExtiPin};
use stm32f1xx_hal::pac::EXTI;
use stm32f1xx_hal::spi::Error as SPIError;
use stm32f1xx_hal::spi::{self, Spi};
use stm32f1xx_hal::time::U32Ext as _;

pub const SPI_MODE: embedded_hal::spi::Mode = embedded_hal::spi::MODE_1;
pub const MAX_SPEED_MHZ: u32 = 10;

#[derive(Getters, Setters)]
pub struct AS5147P<A, B> {
    enc_a: A,
    enc_b: B,

    last_pin_value: u8,

    #[getset(get = "pub", set = "pub")]
    count: i32,

    #[getset(get = "pub", set = "pub")]
    offset: u16,

    #[getset(get = "pub", set = "pub")]
    inverted: bool,

    spi: stm32f1xx_hal::spi::Spi<stm32f1xx_hal::pac::SPI1, spi::Spi1NoRemap, SPIPins, u16>,
}

impl<A, B> AS5147P<A, B> {
    // TODO: Fill this in
    const LUT: [i32; 16] = [0; 16];

    pub fn new(
        spi: Spi<stm32f1xx_hal::pac::SPI1, spi::Spi1NoRemap, SPIPins, u16>,
        mut a: A,
        mut b: B,
        afio: &mut afio::Parts,
        exti: &mut EXTI,
    ) -> Result<Self, ()>
    where
        A: InputPin + ExtiPin,
        B: InputPin + ExtiPin,
    {
        a.make_interrupt_source(afio);
        b.make_interrupt_source(afio);

        a.trigger_on_edge(exti, Edge::RISING_FALLING);
        b.trigger_on_edge(exti, Edge::RISING_FALLING);

        a.enable_interrupt(exti);
        b.enable_interrupt(exti);

        Ok(Self {
            spi,
            enc_a: a,
            enc_b: b,
            count: 0,
            offset: 0,
            inverted: false,
            last_pin_value: 0,
        })
    }

    pub fn handle_encoder_update(&mut self) -> Result<(), ()>
    where
        A: InputPin + ExtiPin,
        B: InputPin + ExtiPin,
        <A as InputPin>::Error: core::fmt::Debug,
        <B as InputPin>::Error: core::fmt::Debug,
    {
        if !self.enc_a.check_interrupt() && !self.enc_b.check_interrupt() {
            return Ok(());
        }

        let a = self.enc_a.is_high().unwrap() as u8;
        let b = self.enc_b.is_high().unwrap() as u8;
        let pin_val = (a << 1) | b;

        let idx = pin_val + self.last_pin_value * 4;

        let increment = Self::LUT[idx as usize];

        if increment != 2 {
            self.count += if self.inverted { -increment } else { increment };
        } else {
            return Err(());
        }

        self.last_pin_value = pin_val;
        self.enc_a.clear_interrupt_pending_bit();
        self.enc_b.clear_interrupt_pending_bit();
        Ok(())
    }

    fn generate_control_frame(read: bool, addr: u16) -> ControlFrame {
        let mut control_frame = ControlFrame::default();
        control_frame.set_read(read);
        control_frame.set_addr(addr);
        control_frame.calculate_parity();
        control_frame
    }

    pub fn read_register<REG: registers::Register + registers::ReadRegister>(
        &mut self,
    ) -> Result<REG, SPIError> {
        use stm32f1xx_hal::prelude::*;
        let control_frame = Self::generate_control_frame(true, REG::ADDRESS);

        nb::block!(self.spi.send(control_frame.into()))?;
        let result = nb::block!(self.spi.read())?;

        let _err = (result >> 14) & 1 == 1;

        Ok(REG::from(result))
    }

    pub fn write_register<REG: registers::Register + registers::WriteRegister>(
        &mut self,
        reg: REG,
    ) -> Result<(), SPIError> {
        use stm32f1xx_hal::prelude::*;
        let control_frame = Self::generate_control_frame(false, REG::ADDRESS);

        nb::block!(self.spi.send(control_frame.into()))?;
        nb::block!(self.spi.send(reg.into()))?;

        // TODO: do something with these evenutally.
        let _old_data = nb::block!(self.spi.read())?;
        let _new_data = nb::block!(self.spi.read())?;

        Ok(())
    }
}
