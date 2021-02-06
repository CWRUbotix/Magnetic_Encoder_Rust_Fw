#![feature(lang_items, panic_info_message)]
#![no_std]
#![no_main]
#![allow(unused_imports, dead_code)]

mod can_types;
mod memory;
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
const ABS_POS_PD: u32 = times_per_second(10);

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
        #[init(0)]
        count: i32,
        #[init(false)]
        inverted: bool,
        #[init(0)]
        abs_offset: u16,

        spi: SPI,

        status1: status::StatusLed<Status1Pin>,
        status2: status::StatusLed<Status2Pin>,
        status3: status::StatusLed<Status3Pin>,
    }

    #[init(schedule = [update_leds])]
    fn init(cx: init::Context) -> init::LateResources {
        CanFramePool::grow(cortex_m::singleton!(: [u8;1024] = [0; 1024]).unwrap());
        let can_tx_queue = BinaryHeap::new();
        let mut device = cx.device;
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
        let memory = {
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

        let mut spi = {
            let spi_pins = (
                gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
                gpioa.pa6.into_floating_input(&mut gpioa.crl),
                gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
            );
            let spi = spi::Spi::spi1(
                device.SPI1,
                spi_pins,
                &mut afio.mapr,
                embedded_hal::spi::MODE_1,
                100.khz(),
                clocks,
                &mut rcc.apb2,
            )
            .frame_size_16bit();

            spi
        };

        // clear shift register
        while let Ok(_) = nb::block!(spi.read()) {}

        // configure spi

        // enable cycle counter for scheduling
        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        let now = cx.start;

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
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = EXTI9_5, priority = 16, resources = [power_sense])]
    fn exti95(cx: exti95::Context) {
        use embedded_hal::digital::v2::InputPin;
        use stm32f1xx_hal::gpio::ExtiPin;
        let power_sense: &mut PowerSensePin = cx.resources.power_sense;

        if power_sense.check_interrupt() && power_sense.is_low().unwrap() {
            //

            // low power
            while power_sense.is_low().unwrap() {
                // wait for death
                core::hint::spin_loop();
            }
        }
    }

    #[task(binds = EXTI15_10, resources = [enc_a, enc_b, last_encoder_val, count, inverted])]
    fn exti1510(cx: exti1510::Context) {
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
                    // this is an error,
                    // TODO: Handle this correctly
                }
                _ => {
                    *cx.resources.count += if *cx.resources.inverted {
                        -increment
                    } else {
                        increment
                    } as i32;
                }
            }

            *last_encoder_val = encoder_value;

            enc_a.clear_interrupt_pending_bit();
            enc_b.clear_interrupt_pending_bit();
        }
    }

    // #[task(resources=[spi])]
    // fn spi_write(cx: spi_write::Context, address: u16, data: u16) {
    //     use spi::FullDuplex;
    //     let spi = cx.resources.spi;
    //     let ones: u16 = address.count_ones() as u16 + 1_u16;
    //     let address = (1 << 14) | address | ((ones % 2) << 15);
    //
    //     spi.send(address).unwrap();
    //     spi.send(data).unwrap();
    //
    //     // do we need to wait???
    //
    //     for _ in 0..2 {
    //         let _ = spi.read();
    //     }
    // }
    //
    // #[task(resources=[spi])]
    // fn spi_read(cx: spi_read::Context, address: u16, out: &mut u16) {
    //     use spi::FullDuplex;
    //     let spi = cx.resources.spi;
    //     let ones = address.count_ones() as u16;
    //     let address = (0 << 14) | address | ((ones % 2) << 15);
    //     nb::block!(spi.send(address)).unwrap();
    //     let res = spi.read().unwrap();
    //     let err = (res >> 14) & 0b1 == 1;
    //     defmt::debug_assert!(!err);
    //     *out = res & !(0b11 << 14);
    // }

    #[task(schedule=[encoder_data], resources=[spi], spawn=[])]
    fn encoder_data(cx: encoder_data::Context) {
        // TODO: Get encoder_position
        // TODO: Get diagnostics
        // TODO: Get errors

        cx.schedule
            .encoder_data(Instant::now() + ABS_POS_PD.cycles())
            .unwrap();
    }

    #[task(schedule=[update_leds], resources=[status1, status2, status3])]
    fn update_leds(cx: update_leds::Context) {
        cx.resources.status1.update();
        cx.resources.status2.update();
        cx.resources.status3.update();
        cx.schedule
            .update_leds(Instant::now() + LED_UPDATE_PD.cycles())
            .unwrap();
    }

    #[task(capacity = 16, priority=5, resources = [last_can_rx])]
    fn handle_rx_frame(cx: handle_rx_frame::Context, frame: bxcan::Frame) {
        let last_rx = cx.resources.last_can_rx;

        *last_rx = Some(Instant::now());
    }

    #[task(priority = 3, binds = USB_HP_CAN_TX, resources = [can_tx, can_tx_queue, last_can_rx])]
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
    loop {
        // cortex_m::asm::bkpt();
    }
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}
