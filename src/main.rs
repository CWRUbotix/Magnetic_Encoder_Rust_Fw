#![feature(lang_items)]
#![no_std]
#![no_main]
#![allow(dead_code, unused_imports)]

// dev profile: easier to debug panics; can put a breakpoint on `rust_begin_unwind`
#[cfg(debug_assertions)]
use panic_halt as _;

// release profile: minimize the binary size of the application
#[cfg(not(debug_assertions))]
use panic_abort as _;

use core::prelude::*;
use embedded_hal::prelude::*;
use stm32f1xx_hal::prelude::*;

use embedded_hal::digital::v2::{InputPin, OutputPin};

use cortex_m::peripheral::DWT;

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

use bxcan::{filter::Mask32, Data, ExtendedId, Frame, Instance, Interrupts, Rx, StandardId, Tx};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::pac::Interrupt;
use stm32f1xx_hal::pac::CAN1;
use stm32f1xx_hal::{can::Can, gpio, gpio::ExtiPin, i2c, spi};

use heapless::binary_heap::{BinaryHeap, Max};
use heapless::consts::*;
use heapless::pool::singleton::Pool;
use heapless::{pool, pool::singleton::Box, pool::Init};

use defmt::{debug, info, Format};

/// enable the defmt logger if desired
#[cfg(debug_assertions)]
use defmt_rtt as _;

use core::cmp::Ordering;

use core::convert::{From, Into};

mod encoder;
mod memory;
mod status;

/// Wrapper around a bxcan::Frame that allows
/// a binary heap to sort them by priority
#[derive(Debug, Format)]
pub struct PriorityFrame(Frame);

impl Ord for PriorityFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl PartialOrd for PriorityFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for PriorityFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for PriorityFrame {}

// hardware type declarations
type PowerSensePin = gpio::gpiob::PB15<gpio::Input<gpio::Floating>>;

type Status1Pin = gpio::gpiob::PB5<gpio::Output<gpio::PushPull>>;
type Status2Pin = gpio::gpiob::PB3<gpio::Output<gpio::PushPull>>;
type Status3Pin = gpio::gpiob::PB4<gpio::Output<gpio::PushPull>>;

type EncoderAPin = gpio::gpioa::PA8<gpio::Input<gpio::PullDown>>;
type EncoderBPin = gpio::gpioa::PA9<gpio::Input<gpio::PullDown>>;
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

/// External oscilator clock in MHz
const HSE_CLOCK_MHZ: u32 = 8;
/// System clock in MHz
const SYS_CLOCK_MHZ: u32 = 72;

/// External oscilator clock in Hz
const HSE_CLOCK_HZ: u32 = HSE_CLOCK_MHZ * 1_000_000;
/// System clock in Hz
const SYS_CLOCK_HZ: u32 = SYS_CLOCK_MHZ * 1_000_000;

#[cfg(debug_assertions)]
#[defmt::timestamp]
fn timestamp() -> u64 {
    DWT::get_cycle_count() as u64
}

fn times_per_second(times: u32) -> rtic::cyccnt::Duration {
    (SYS_CLOCK_HZ / times).cycles()
}

pool!(
    #[allow(non_upper_case_globals)]
    CanFramePool: PriorityFrame
);

fn allocate_tx_frame(frame: Frame) -> Box<CanFramePool, Init> {
    let b = CanFramePool::alloc().unwrap();
    b.init(PriorityFrame(frame))
}

#[app(device=stm32f1xx_hal::pac, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(true)]
        power_ok: bool,

        #[init(false)]
        can_ok: bool,

        /// can id
        can_id: bxcan::StandardId,

        /// can transmit handle
        can_tx: Tx<Can<CAN1>>,

        /// queue for outgoing can frames
        can_tx_queue: BinaryHeap<Box<CanFramePool, Init>, U16, Max>,

        /// can reciever hanndle
        can_rx: Rx<Can<CAN1>>,

        /// gpio pin for sensing when power drops
        /// We will have an exti interrupt on this pin
        power_sense: PowerSensePin,

        /// leds that display status
        status1: status::StatusLed<Status1Pin>,
        status2: status::StatusLed<Status2Pin>,
        status3: status::StatusLed<Status3Pin>,

        /// encoder
        encoder: encoder::Encoder<EncoderAPin, EncoderBPin, EncoderIPin, SPI>,

        //
        memory: memory::Memory<I2C>,
    }

    #[init(schedule=[update_leds])]
    fn init(cx: init::Context) -> init::LateResources {
        /// allocate a chunk of memory for storing outgoing
        /// can frames. 1KB should be fine
        static mut CAN_TX_MEM: [u8; 1024] = [0; 1024];

        defmt::info!("Initializing...");

        // grow the allocation pool
        CanFramePool::grow(CAN_TX_MEM);

        // create the binary heap (on the stack)
        let can_tx_queue = BinaryHeap::new();

        let mut peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let exti = device.EXTI;

        let clocks = rcc
            .cfgr
            .use_hse(HSE_CLOCK_MHZ.mhz())
            .sysclk(SYS_CLOCK_MHZ.mhz())
            .hclk(SYS_CLOCK_MHZ.mhz())
            .pclk1((SYS_CLOCK_MHZ / 2).mhz())
            .pclk2(SYS_CLOCK_MHZ.mhz())
            .freeze(&mut flash.acr);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // status leds
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

        // power sense
        let mut power_sense = gpiob.pb15.into_floating_input(&mut gpiob.crh);
        power_sense.trigger_on_edge(&exti, gpio::Edge::FALLING);

        #[cfg(feature = "has_5V")]
        power_sense.enable_interrupt(&exti);

        // encoder setup
        let encoder_a = gpioa.pa8.into_pull_down_input(&mut gpioa.crh);
        let encoder_b = gpioa.pa9.into_pull_down_input(&mut gpioa.crh);
        let encoder_i = gpioa.pa10.into_pull_down_input(&mut gpioa.crh);

        let spi1 = {
            let spi_pins = (
                gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
                gpioa.pa6.into_floating_input(&mut gpioa.crl),
                gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
            );

            let spi_mode = spi::Mode {
                polarity: spi::Polarity::IdleLow,
                phase: spi::Phase::CaptureOnFirstTransition,
            };

            spi::Spi::spi1(
                device.SPI1,
                spi_pins,
                &mut afio.mapr,
                spi_mode,
                100.khz(),
                clocks,
                &mut rcc.apb2,
            )
            .frame_size_16bit()
        };

        let encoder =
            encoder::Encoder::new(encoder_a, encoder_b, encoder_i, spi1, &exti, &mut afio);

        let i2c_pins: I2CPins = (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh),
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh),
        );

        let i2c = i2c::BlockingI2c::i2c2(
            device.I2C2,
            i2c_pins,
            i2c::Mode::Standard {
                frequency: 100_000.hz(),
            },
            clocks,
            &mut rcc.apb1,
            // TODO: dont really know what these 4 parameters should actually be
            50,  // start_timeout_us
            5,   // start_retries
            100, // addr_timeout_us
            100, // data_timeout_us
        );

        let memory = memory::Memory::new(i2c);

        // get can id from dip switches
        let can_id: u16 = 0;

        let can_id = StandardId::new(can_id).unwrap();

        // setup can interface
        let can = Can::new(device.CAN1, &mut rcc.apb1, device.USB);
        {
            let can_tx_pin = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
            let can_rx_pin = gpiob.pb8.into_floating_input(&mut gpiob.crh);

            can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);
        }

        let mut can = bxcan::Can::new(can);

        can.configure(|config| {
            config.set_bit_timing(0x001c_0000);
        });

        let can_id_mask = StandardId::new(0xFF).unwrap();
        let mut can_filters = can.modify_filters();
        can_filters.enable_bank(0, Mask32::frames_with_std_id(can_id, can_id_mask));
        drop(can_filters);

        nb::block!(can.enable()).unwrap();

        let (can_tx, can_rx) = can.split();

        // enable cycle counter for scheduling
        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        let now = cx.start;

        // schedule led update for 1/8 of a second from now
        cx.schedule.update_leds(now + times_per_second(8)).unwrap();

        init::LateResources {
            can_id,
            can_tx_queue,
            can_tx,
            can_rx,
            power_sense,
            status1,
            status2,
            status3,
            encoder,
            memory,
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    // due to a stroke of luck/genius, the a and b channels of the encoders are
    // on exti lines 8 and 9, meaning we can treat this as the only encoder callback
    // since we dont care about the i channel at the moment
    #[task(binds = EXTI9_5, priority=2, resources = [encoder])]
    fn exti9_5(cx: exti9_5::Context) {
        cx.resources.encoder.update();
    }

    // in the future, we may want to handle encoder channel i on
    // line 10 of the exti, but not for now
    #[task(priority = 1,binds = EXTI15_10, resources=[ power_sense, status1, status2, status3 ])]
    fn exti15_10(cx: exti15_10::Context) {
        #[allow(unused_variables)]
        let power_sense = cx.resources.power_sense;
        #[allow(unused_variables)]
        let status1 = cx.resources.status1;
        #[allow(unused_variables)]
        let status2 = cx.resources.status2;
        #[allow(unused_variables)]
        let status3 = cx.resources.status3;

        power_sense.clear_interrupt_pending_bit();

        // this will only work when we tell the compiler the board
        // will have 5 volts
        #[cfg(feature = "has_5V")]
        {
            if power_sense.check_interrupt() && power_sense.is_low().unwrap() {
                // there is a loss in power
                // store data and wait for power to
                // return (or not return)

                // turn of leds to save some power
                status1.flash_slow();
                status2.flash_slow();
                status3.flash_slow();

                // store stuff in memory

                while power_sense.is_low().unwrap() {
                    // wait until power is back
                    core::sync::atomic::spin_loop_hint();
                }
            }
        }
    }

    #[task(resources=[status1, status2, status3], schedule=[update_leds])]
    fn update_leds(cx: update_leds::Context) {
        cx.resources.status1.update();
        cx.resources.status2.update();
        cx.resources.status3.update();

        defmt::trace!("Updating leds");

        cx.schedule
            .update_leds(Instant::now() + times_per_second(8))
            .unwrap();
    }

    #[task(binds = USB_LP_CAN_RX0, resources=[can_rx], spawn=[handle_rx_frame])]
    fn can_rx0(cx: can_rx0::Context) {
        let rx = cx.resources.can_rx;

        loop {
            match rx.receive() {
                Ok(frame) => {
                    // handle frames here
                    cx.spawn.handle_rx_frame(frame).unwrap();
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => { /* ignore other errors */ }
            }
        }
    }

    #[task(resources = [can_id, can_tx_queue, can_ok])]
    fn handle_rx_frame(cx: handle_rx_frame::Context, frame: Frame) {
        *cx.resources.can_ok = true;
        let can_id = cx.resources.can_id;
        let tx_queue = cx.resources.can_tx_queue;

        let rx_id = match frame.id() {
            bxcan::Id::Standard(id) => id.as_raw(),
            bxcan::Id::Extended(_) => return,
        };

        defmt::info!("Recieved can frame: {:?}", frame);

        let _id = rx_id & 0xFF;
        let _cmd = rx_id >> 8;

        let ret_frame: Frame;
        // we can assume that any frame that makes it here is supposed to be here
        // since we filter out any frames without our id

        // if we get a remote frame, send the tick count back
        if frame.is_remote_frame() {
            // put encoder data here
            ret_frame = Frame::new_data(bxcan::Id::Standard(*can_id), (0_u16).to_ne_bytes());
            // push to tx queue
            tx_queue.push(allocate_tx_frame(ret_frame)).unwrap();
        } else {
            defmt::todo!();
        }

        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    #[task(binds = USB_HP_CAN_TX, resources = [can_tx, can_tx_queue])]
    fn can_tx(cx: can_tx::Context) {
        let tx = cx.resources.can_tx;
        let tx_queue = cx.resources.can_tx_queue;

        tx.clear_interrupt_flags();

        // make sure we send the frame with the highest priority first
        while let Some(frame) = tx_queue.peek() {
            match tx.transmit(&frame.0) {
                Ok(None) => {
                    use core::ops::Deref;
                    let sent_frame = tx_queue.pop();
                    defmt::info!("Sent frame: {:?}", sent_frame.unwrap().deref());
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

    #[allow(non_snake_case)]
    extern "C" {
        fn USART1();
        fn USART2();

        fn SPI2();
        fn SPI3();
    }
};

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}
