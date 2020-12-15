use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio;

use core::fmt::Debug;

pub enum LedMode {
    Off = 0,
    On,
    FlashSlow,
    FlashFast,
    None,
}

pub struct StatusLed<Pin> {
    pin: Pin,
    current_mode: LedMode,
    count: u8,
}

impl<Pin> StatusLed<Pin>
where
    Pin: OutputPin,
    <Pin as OutputPin>::Error: Debug,
{
    pub fn new_with_mode(pin: Pin, mode: LedMode) -> Self {
        Self {
            pin,
            current_mode: mode,
            count: 0,
        }
    }

    pub fn new(pin: Pin) -> Self {
        Self::new_with_mode(pin, LedMode::Off)
    }

    pub fn update(&mut self) {
        match self.current_mode {
            LedMode::On => self.pin.set_high().unwrap(),
            LedMode::Off => self.pin.set_low().unwrap(),
            LedMode::FlashSlow => {
                if self.count < 2 {
                    self.pin.set_high().unwrap();
                } else {
                    self.pin.set_low().unwrap();
                }
            }
            LedMode::FlashFast => {
                if self.count % 2 == 0 {
                    self.pin.set_high().unwrap();
                } else {
                    self.pin.set_low().unwrap();
                }
            }
            _ => unimplemented!(),
        };
        self.count += 1;
        self.count %= 4;
    }

    pub fn set_mode(&mut self, desired_mode: LedMode) {
        self.current_mode = desired_mode;
    }
}