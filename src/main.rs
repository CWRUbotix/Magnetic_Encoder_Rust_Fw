#![feature(lang_items, panic_info_message, num_as_ne_bytes)]
#![no_std]
#![no_main]
#![allow(unused_imports, dead_code)]

mod can_types;
mod error_codes;
mod memory;
mod spi_ext;
mod status;

use core::prelude::v1::*;
use embedded_hal::prelude::*;
use stm32f1xx_hal::prelude::*;

use rtic::app;

use defmt;
use defmt_rtt as _;

use rtic::cyccnt::{Duration, Instant, U32Ext as _};

use stm32f1xx_hal;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::{can, gpio, i2c, spi};

use heapless::pool::{
    singleton::{Box, Pool},
    Init,
};
use heapless::BinaryHeap;

use cortex_m::peripheral::DWT;

// hardware types
type PowerSensePin = gpio::gpiob::PB15<gpio::Input<gpio::Floating>>;

type Status1Pin = gpio::gpiob::PB5<gpio::Output<gpio::PushPull>>;
type Status2Pin = gpio::gpiob::PB3<gpio::Output<gpio::PushPull>>;
type Status3Pin = gpio::gpiob::PB4<gpio::Output<gpio::PushPull>>;

type EncoderAPin = gpio::gpioa::PA8<gpio::Input<gpio::Floating>>;
type EncoderBPin = gpio::gpioa::PA9<gpio::Input<gpio::Floating>>;
type EncoderIPin = gpio::gpioa::PA10<gpio::Input<gpio::PullDown>>;

// SPI definitions
pub type MOSIPin = gpio::gpioa::PA5<gpio::Alternate<gpio::PushPull>>;
pub type MISOPin = gpio::gpioa::PA6<gpio::Input<gpio::Floating>>;
pub type SCKPin = gpio::gpioa::PA7<gpio::Alternate<gpio::PushPull>>;
pub type SPIPins = (MOSIPin, MISOPin, SCKPin);
pub type SPI = spi::Spi<pac::SPI1, spi::Spi1NoRemap, SPIPins, u16>;

pub type SclPin = gpio::gpiob::PB10<gpio::Alternate<gpio::OpenDrain>>;
pub type SdaPin = gpio::gpiob::PB11<gpio::Alternate<gpio::OpenDrain>>;
pub type I2CPins = (SclPin, SdaPin);
pub type I2C = i2c::BlockingI2c<pac::I2C2, I2CPins>;

type Memory = memory::Memory<I2C>;

/// External oscilator clock in MHz
const HSE_CLOCK_MHZ: u32 = 8;
/// System clock in MHz
const SYS_CLOCK_MHZ: u32 = 72;

/// External oscilator clock in Hz
const HSE_CLOCK_HZ: u32 = HSE_CLOCK_MHZ * 1_000_000;
/// System clock in Hz
const SYS_CLOCK_HZ: u32 = SYS_CLOCK_MHZ * 1_000_000;

const CAN_CONFIG: u32 = 0x001e0001;

const ENCODER_LUT: [i8; 16] = [0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0];

const fn times_per_second(times: u32) -> u32 {
    SYS_CLOCK_HZ / times
}

const LED_UPDATE_PD: u32 = times_per_second(8);
const ABS_POS_PD: u32 = times_per_second(100);
const CAN_TX_PD: u32 = times_per_second(20);

heapless::pool!(
    #[allow(non_upper_case_globals)]
    CanFramePool: can_types::PriorityFrame
);

fn allocate_tx_frame(frame: bxcan::Frame) -> Box<CanFramePool, Init> {
    let b = CanFramePool::alloc().unwrap();
    b.init(can_types::PriorityFrame(frame))
}

#[defmt::timestamp]
fn timestamp() -> u64 {
    DWT::get_cycle_count() as u64
}

#[app(device=stm32f1xx_hal::stm32, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        can_id: bxcan::Id,
        can_tx_queue: BinaryHeap<
            Box<CanFramePool, heapless::pool::Init>,
            heapless::consts::U16,
            heapless::binary_heap::Max,
        >,
        #[init(None)]
        last_can_rx: Option<Instant>,

        can_tx: bxcan::Tx<can::Can<pac::CAN1>>,
        can_rx: bxcan::Rx<can::Can<pac::CAN1>>,

        memory: Memory,

        power_sense: PowerSensePin,

        enc_a: EncoderAPin,
        enc_b: EncoderBPin,

        #[init(0)]
        last_encoder_val: i8,

        count: i32,
        inverted: bool,
        abs_offset: u16,

        spi: SPI,

        status1: status::StatusLed<Status1Pin>,
        status2: status::StatusLed<Status2Pin>,
        status3: status::StatusLed<Status3Pin>,
    }

    #[init(schedule = [update_leds, encoder_data, can_tx])]
    fn init(cx: init::Context) -> init::LateResources {
        CanFramePool::grow(cortex_m::singleton!(: [u8;1024] = [0; 1024]).unwrap());
        let can_tx_queue = BinaryHeap::new();
        let device = cx.device;
        let mut peripherals = cx.core;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        let status1 = status::StatusLed::new(gpiob.pb5.into_push_pull_output(&mut gpiob.crl));
        let (status2, status3) = {
            let (_, s2, s3) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
            (
                status::StatusLed::new_with_mode(
                    s2.into_push_pull_output(&mut gpiob.crl),
                    status::LedMode::FlashSlow,
                ),
                status::StatusLed::new_with_mode(
                    s3.into_push_pull_output(&mut gpiob.crl),
                    status::LedMode::FlashFast,
                ),
            )
        };

        let exti = device.EXTI;

        let clocks = rcc
            .cfgr
            .use_hse(HSE_CLOCK_MHZ.mhz())
            .sysclk(SYS_CLOCK_MHZ.mhz())
            .hclk(SYS_CLOCK_MHZ.mhz())
            .pclk1((SYS_CLOCK_MHZ / 2).mhz())
            .pclk2(SYS_CLOCK_MHZ.mhz())
            .freeze(&mut flash.acr);

        // power sense
        let mut power_sense = gpiob.pb15.into_floating_input(&mut gpiob.crh);

        use stm32f1xx_hal::gpio::{Edge, ExtiPin};
        #[cfg(feature = "has-5V")]
        {
            // use embedded_hal::exti
            power_sense.make_interrupt_source(&mut afio);
            power_sense.trigger_on_edge(&exti, Edge::FALLING);
            power_sense.enable_interrupt(&exti);
        }
        defmt::debug!("Created power sense pin.");

        let mut enc_a = gpioa.pa8.into_floating_input(&mut gpioa.crh);
        enc_a.make_interrupt_source(&mut afio);
        enc_a.trigger_on_edge(&exti, Edge::RISING_FALLING);
        enc_a.enable_interrupt(&exti);

        let mut enc_b = gpioa.pa9.into_floating_input(&mut gpioa.crh);
        enc_b.make_interrupt_source(&mut afio);
        enc_b.trigger_on_edge(&exti, Edge::RISING_FALLING);
        enc_b.enable_interrupt(&exti);

        // TODO: replace with the correct pins
        #[allow(unused_must_use)]
        let can_id = {
            use embedded_hal::digital::v2::InputPin;
            use heapless::consts::*;

            let mut id = 0_u16;
            let mut pins = heapless::Vec::<gpio::Pxx<gpio::Input<gpio::PullDown>>, U8>::new();

            // put all can_id pins in a vec
            pins.push(gpiob.pb14.into_pull_down_input(&mut gpiob.crh).downgrade());
            pins.push(gpiob.pb13.into_pull_down_input(&mut gpiob.crh).downgrade());
            pins.push(gpiob.pb12.into_pull_down_input(&mut gpiob.crh).downgrade());
            pins.push(gpiob.pb2.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb1.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb0.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpioa.pa4.into_pull_down_input(&mut gpioa.crl).downgrade());
            pins.push(gpioa.pa3.into_pull_down_input(&mut gpioa.crl).downgrade());

            // interate over the pins and shift values as we go
            for (shift, pin) in pins.iter().enumerate() {
                id |= (pin.is_high().unwrap() as u16) << (shift as u16);
                defmt::debug!("Id so far: {:u16}", id);
            }
            pins.clear(); // just to make sure this happens.
            id
        };

        let (can_tx, can_rx) = {
            use bxcan;
            use bxcan::{ExtendedId, Id, Interrupts, StandardId};
            let can = can::Can::new(device.CAN1, &mut rcc.apb1, device.USB);

            let can_tx_pin = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
            let can_rx_pin = gpiob.pb8.into_floating_input(&mut gpiob.crh);

            can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

            let mut can = bxcan::Can::new(can);

            can.configure(|config| {
                config.set_bit_timing(CAN_CONFIG);
                config.set_loopback(false);
                config.set_silent(false);
            });

            let mask = 0xFF_u16;
            let filter1 = bxcan::filter::Mask32::frames_with_ext_id(
                ExtendedId::new(can_id as u32).unwrap(),
                ExtendedId::new(mask as u32).unwrap(),
            );

            let filter2 = bxcan::filter::Mask32::frames_with_std_id(
                StandardId::new(can_id).unwrap(),
                StandardId::new(mask).unwrap(),
            );

            assert!(can.modify_filters().num_banks() >= 2);

            can.modify_filters()
                .clear()
                .enable_bank(0, filter1)
                .enable_bank(1, filter2);

            can.enable_interrupts(
                Interrupts::FIFO0_MESSAGE_PENDING | Interrupts::FIFO1_MESSAGE_PENDING,
            );

            nb::block!(can.enable()).unwrap();

            can.split()
        };

        let can_id = bxcan::Id::Standard(unsafe { bxcan::StandardId::new_unchecked(can_id) });
        let mut memory = {
            let i2c_pins: I2CPins = (
                gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh),
                gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh),
            );

            let i2c = i2c::BlockingI2c::i2c2(
                device.I2C2,
                i2c_pins,
                i2c::Mode::Standard {
                    frequency: 400_000.hz(),
                },
                clocks,
                &mut rcc.apb1,
                // NOTE: dont really know what these 4 parameters should actually be
                // I copied these from someones example and they seem to work
                1000, // start_timeout_us
                10,   // start_retries
                1000, // addr_timeout_us
                1000, // data_timeout_us
            );

            Memory::new(i2c)
        };

        #[cfg(feature = "wipe-mem")]
        {
            // we may want to wipe the eeprom sometimes.
            // it can be enabled with the 'wipe-mem' feature
            defmt::debug!("Clearing eeprom");
            memory.clear_all_data().unwrap();
            defmt::debug!("Done clearing eeprom");
        }

        // get values from mem
        #[cfg(not(debug_assertions))]
        let read_mem = memory.read_bool(Memory::HAS_DATA_ADDR).unwrap_or(false);

        // if we are in debug mode, its ok to panic,
        // so just unwrap so we can debug
        #[cfg(debug_assertions)]
        let read_mem = memory.read_bool(Memory::HAS_DATA_ADDR).unwrap();

        #[allow(unused_variables)]
        let (ticks, polarity, offset) = if read_mem {
            (
                memory.read_data::<i32>(Memory::TICK_ADDR).unwrap_or(0),
                memory.read_bool(Memory::POL_ADDR).unwrap_or(false),
                memory.read_data::<u16>(Memory::ABS_OFF_ADDR).unwrap_or(0),
            )
        } else {
            (0, false, 0)
        };

        // configure spi

        let mut spi = {
            let spi_pins = (
                gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
                gpioa.pa6.into_floating_input(&mut gpioa.crl),
                gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
            );
            let mut spi = spi::Spi::spi1(
                device.SPI1,
                spi_pins,
                &mut afio.mapr,
                embedded_hal::spi::MODE_1,
                10.mhz(),
                clocks,
                &mut rcc.apb2,
            )
            .frame_size_16bit();

            spi.bit_format(spi::SpiBitFormat::MsbFirst);

            spi
        };

        // clear shift register
        while nb::block!(spi.read()).is_ok() {}

        // configure spi
        {
            use spi_ext::Address;
            use spi_ext::SpiExt;

            //NOTE:
            // Datasheet:
            // https://ams.com/documents/20143/36005/AS5147P_DS000328_2-00.pdf
            // for now, I am too lazy to make bitfields/abstractions over the
            // different registers, so we will hard code them. They dont really
            // need to be changed anyways
            let settings1: u16 = 0b0000_0000_0000_0001;
            let settings2: u16 = 0b0;

            let zpos_mask = 0b111111;

            let zposm: u16 = offset & !zpos_mask;
            let zposl: u16 = offset & zpos_mask; // | (1 << 6) | (1 << 7);

            SpiExt::write(&mut spi, Address::SETTINGS1, settings1).unwrap();
            SpiExt::write(&mut spi, Address::SETTINGS2, settings2).unwrap();
            SpiExt::write(&mut spi, Address::ZPOSM, zposm).unwrap();
            SpiExt::write(&mut spi, Address::ZPOSL, zposl).unwrap();
        }

        // enable cycle counter for scheduling
        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        let now = cx.start;

        cx.schedule.can_tx(now + CAN_TX_PD.cycles()).unwrap();

        cx.schedule.encoder_data(now + ABS_POS_PD.cycles()).unwrap();

        cx.schedule
            .update_leds(now + LED_UPDATE_PD.cycles())
            .unwrap();

        init::LateResources {
            can_id,
            can_tx_queue,
            can_tx,
            can_rx,
            power_sense,
            enc_a,
            enc_b,
            spi,
            memory,
            status1,
            status2,
            status3,
            count: ticks,
            inverted: polarity,
            abs_offset: offset,
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = EXTI9_5, priority = 16, resources = [power_sense, memory, count, inverted, abs_offset])]
    fn exti95(cx: exti95::Context) {
        use embedded_hal::digital::v2::InputPin;
        use stm32f1xx_hal::gpio::ExtiPin;
        let memory = cx.resources.memory;
        let power_sense: &mut PowerSensePin = cx.resources.power_sense;

        if power_sense.check_interrupt() && power_sense.is_low().unwrap() {
            //
            memory.write_data(Memory::TICK_ADDR, 0).unwrap();
            memory.write_bool(Memory::POL_ADDR, false).unwrap();
            memory.write_data(Memory::ABS_OFF_ADDR, 0).unwrap();

            // write validity bit
            memory.write_bool(Memory::HAS_DATA_ADDR, true).unwrap();

            // low power
            while power_sense.is_low().unwrap() {
                // wait for death
                core::hint::spin_loop();
            }
            power_sense.clear_interrupt_pending_bit();
        }
    }

    #[task(priority = 8, binds = EXTI15_10, resources = [can_tx_queue, &can_id,  enc_a, enc_b, last_encoder_val, count, inverted])]
    fn exti1510(mut cx: exti1510::Context) {
        use embedded_hal::digital::v2::InputPin;
        use stm32f1xx_hal::gpio::ExtiPin;
        let enc_a: &mut EncoderAPin = cx.resources.enc_a;
        let enc_b: &mut EncoderBPin = cx.resources.enc_b;
        let last_encoder_val: &mut i8 = cx.resources.last_encoder_val;

        if enc_a.check_interrupt() || enc_b.check_interrupt() {
            defmt::debug!("Encoder interrupt");
            let encoder_value =
                ((enc_a.is_high().unwrap() as i8) << 1) | ((enc_b.is_high().unwrap() as i8) << 0);

            let idx = encoder_value + *last_encoder_val * 4;
            defmt::debug_assert!(idx < 16 && idx >= 0);
            let increment = ENCODER_LUT[idx as usize];

            match increment {
                2 => {
                    use can_types::IntoWithId;
                    // this is an error,
                    // TODO: Handle this correctly
                    cx.resources
                        .can_tx_queue
                        .push(allocate_tx_frame(
                            can_types::OutgoingFrame::Error(2).into_with_id(*cx.resources.can_id),
                        ))
                        .unwrap();
                }
                _ => {
                    let inverted = cx.resources.inverted.lock(|i| *i);
                    cx.resources.count.lock(|c| {
                        *c += if inverted { -increment } else { increment } as i32;
                    });
                }
            }

            *last_encoder_val = encoder_value;

            enc_a.clear_interrupt_pending_bit();
            enc_b.clear_interrupt_pending_bit();
        }
    }

    #[task(priority = 8, schedule=[encoder_data], resources=[spi, status1, status2, can_tx_queue, &can_id, count], spawn=[])]
    fn encoder_data(mut cx: encoder_data::Context) {
        use can_types::IntoWithId;
        use spi_ext::Address;
        use spi_ext::SpiExt;

        let spi = cx.resources.spi;
        let tx_queue = cx.resources.can_tx_queue;

        let angle = SpiExt::read(spi, Address::ANGLECOM).unwrap();
        let angle = angle & !(0b11_u16 << 14);

        tx_queue
            .push(allocate_tx_frame(
                can_types::OutgoingFrame::Update {
                    abs_pos: angle,
                    ticks: cx.resources.count.lock(|c| *c),
                }
                .into_with_id(*cx.resources.can_id),
            ))
            .unwrap();

        let err = SpiExt::read(spi, Address::ERRFL).unwrap();
        let parity_err = (err >> 2) & 1 == 1;
        let invalid_cmd = (err >> 1) & 1 == 1;
        let framing_err = (err >> 0) & 1 == 1;

        if parity_err || invalid_cmd || framing_err {
            tx_queue
                .push(allocate_tx_frame(
                    can_types::OutgoingFrame::Error(1).into_with_id(*cx.resources.can_id),
                ))
                .unwrap();
        };

        let diag = SpiExt::read(spi, Address::DIAAGC).unwrap();
        let magl = (diag >> 11) & 1 == 1;
        let magh = (diag >> 10) & 1 == 1;
        let _cof = (diag >> 9) & 1 == 1;
        let _lf = (diag >> 8) & 1 == 1;

        if magl {
            cx.resources.status1.off();
        } else {
            cx.resources.status1.on();
        }

        if magh {
            cx.resources.status2.off();
        } else {
            cx.resources.status2.on();
        }

        cx.schedule
            .encoder_data(Instant::now() + ABS_POS_PD.cycles())
            .unwrap();
    }

    #[task(schedule=[update_leds], resources=[status1, status2, status3])]
    fn update_leds(mut cx: update_leds::Context) {
        cx.resources.status1.lock(|s| s.update());
        cx.resources.status2.lock(|s| s.update());
        cx.resources.status3.update();
        cx.schedule
            .update_leds(Instant::now() + LED_UPDATE_PD.cycles())
            .unwrap();
    }

    #[task(priority=7, resources = [last_can_rx, count, inverted, abs_offset])]
    fn handle_rx_frame(mut cx: handle_rx_frame::Context, frame: bxcan::Frame) {
        use can_types::IncomingFrame;
        use can_types::IncomingFrame::*;
        use core::convert::TryFrom;

        let last_rx = cx.resources.last_can_rx;
        match IncomingFrame::try_from(frame) {
            Ok(SetTicks(ticks)) => {
                defmt::info!("Setting encoder count to {:i32}", ticks);
                cx.resources.count.lock(|t| *t = ticks);
            }
            Ok(Invert(inv)) => {
                defmt::info!("Setting inversion to: {:bool}", inv);
                cx.resources.inverted.lock(|i| *i = inv);
            }
            Ok(Request) => {
                defmt::info!("Recieved request frame");
            }
            Ok(SetAbsOffset(offset)) => {
                defmt::info!("Setting offset to {:u16}", offset);
                cx.resources.abs_offset.lock(|ao| *ao = offset);
            }
            Err(_) => return,
        }

        *last_rx = Some(Instant::now());
    }

    #[task(priority = 5, schedule = [can_tx], resources = [can_tx, can_tx_queue, last_can_rx])]
    fn can_tx(cx: can_tx::Context) {
        let tx = cx.resources.can_tx;
        let mut tx_queue = cx.resources.can_tx_queue;
        let mut last_rx = cx.resources.last_can_rx;

        let can_ok = last_rx.lock(|last_rx| {
            let can_ok = if let Some(t) = last_rx {
                // one second timeout is acceptable
                !(t.elapsed() > SYS_CLOCK_HZ.cycles())
            } else {
                false
            };
            can_ok
        });

        tx.clear_interrupt_flags();

        if !can_ok {
            defmt::debug!("Canbus timeout, waiting for recieved frame before tx");
            return;
        }
        tx_queue.lock(|tx_queue| {
            while let Some(frame) = tx_queue.peek() {
                match tx.transmit(&frame.0) {
                    Ok(None) => {
                        use core::ops::Deref;
                        let sent_frame = tx_queue.pop();
                        defmt::info!("Sent Frame: {:?}", sent_frame.unwrap().deref().0);
                    }
                    Ok(Some(pending_frame)) => {
                        tx_queue.pop();
                        tx_queue.push(allocate_tx_frame(pending_frame)).unwrap();
                    }
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
        cx.schedule
            .can_tx(Instant::now() + CAN_TX_PD.cycles())
            .unwrap();
    }

    #[task(priority=5, binds = USB_LP_CAN_RX0, resources=[can_rx, last_can_rx], spawn=[handle_rx_frame])]
    fn can_rx0(cx: can_rx0::Context) {
        let rx = cx.resources.can_rx;
        loop {
            match rx.receive() {
                Ok(frame) => {
                    cx.spawn.handle_rx_frame(frame).unwrap();
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {}
            }
        }
    }

    #[task(priority=4, binds = CAN_RX1)]
    fn can_rx1(_: can_rx1::Context) {
        use stm32f1xx_hal::pac::Interrupt;
        rtic::pend(Interrupt::USB_LP_CAN_RX0);
    }

    #[allow(non_snake_case)]
    extern "C" {
        fn USART1();
        fn USART2();

        fn SPI2();
        fn SPI3();
    }
};

#[panic_handler]
#[inline(never)]
fn core_panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Debug;

    // eventually, we want to get rid of these debug2blank traits, they use a lot of
    // flash space. Defmt isnt done fixing this though
    use defmt::{consts, Debug2Format};

    defmt::error!(
        "Panic: \"{:?}\" \nin file {:str} at line {:u32}",
        Debug2Format::<defmt::consts::U1024>(info.message().unwrap()),
        info.location().unwrap().file(),
        info.location().unwrap().line()
    );
    loop {}
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}
