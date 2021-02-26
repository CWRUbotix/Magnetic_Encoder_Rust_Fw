use core::cmp::Ordering;
use defmt::Format;

use bxcan::Frame;

pub enum FrameConversionError {
    FrameType,
    InvalidFrame(&'static str),
    TooShort { minimum: u8, actual: u8 },
    InvalidIdFormat,
    InvalidCommand { cmd: u16 },
}

impl Format for FrameConversionError {
    fn format(&self, f: &mut defmt::Formatter) {
        use FrameConversionError::*;
        match self {
            FrameType => {
                defmt::write!(f, "Frame type is incorrect (remote vs data frames)");
            }
            InvalidFrame(s) => {
                defmt::write!(f, "Invalid Frame: {:str}", s);
            }
            TooShort { minimum, actual } => {
                defmt::write!(
                    f,
                    "Data frame too short. Recieved dlc = {:u8}, requires length {:u8}",
                    actual,
                    minimum
                );
            }
            InvalidIdFormat => {
                defmt::write!(f, "Invalid id format");
            }
            InvalidCommand { cmd } => {
                defmt::write!(f, "Invalid command {:u16}", cmd);
            }
        }
    }
}

macro_rules! check_frame_size {
    ($req:expr, $actual:expr) => {
        if $actual < $req {
            return Err(FrameConversionError::TooShort {
                actual: $actual,
                minimum: $req,
            });
        }
    };
}

pub trait IntoWithId<T> {
    fn into_with_id(self, id: bxcan::Id) -> T;
}

pub enum IncomingFrame {
    Request,
    Invert(bool),
    SetTicks(i32),
    SetAbsOffset(u16),
}

use core::convert::TryFrom;

impl TryFrom<bxcan::Frame> for IncomingFrame {
    type Error = FrameConversionError;

    fn try_from(frame: bxcan::Frame) -> Result<Self, Self::Error> {
        let dlc = frame.dlc();

        let rx_id: u16 = match frame.id() {
            bxcan::Id::Standard(id) => id.as_raw(),
            bxcan::Id::Extended(id) => id.standard_id().as_raw(),
        };

        let _id = rx_id & 0xFF;
        let cmd = (rx_id >> 8) & 0xFF;

        if frame.is_remote_frame() {
            Ok(IncomingFrame::Request)
        } else {
            use core::convert::TryInto;
            let data = frame.data().unwrap();
            match cmd {
                0x0 => {
                    check_frame_size!(1, dlc);
                    Ok(IncomingFrame::Invert(data[0] > 0))
                }
                0x1 => {
                    check_frame_size!(4, dlc);
                    let value = i32::from_ne_bytes(data[0..4].try_into().unwrap());
                    Ok(IncomingFrame::SetTicks(value))
                }
                0x2 => {
                    check_frame_size!(2, dlc);
                    let value = u16::from_ne_bytes(data[0..2].try_into().unwrap());
                    Ok(IncomingFrame::SetAbsOffset(value))
                }
                _ => Err(FrameConversionError::InvalidCommand { cmd }),
            }
        }
    }
}

pub enum OutgoingFrame {
    Update { ticks: i32, abs_pos: u16 },
    Diagnostics([u8; 8]),
    Error(u8),
}

impl IntoWithId<bxcan::Frame> for OutgoingFrame {
    fn into_with_id(self, id: bxcan::Id) -> bxcan::Frame {
        let mut new_id = match id {
            bxcan::Id::Standard(raw) => raw.as_raw(),
            bxcan::Id::Extended(raw) => raw.standard_id().as_raw(),
        };
        let mut bytes = heapless::Vec::<u8, heapless::consts::U8>::new();

        match self {
            Self::Update { ticks, abs_pos } => {
                new_id |= 0x01 << 8;
                bytes.extend_from_slice(ticks.as_ne_bytes()).unwrap();
                bytes.extend_from_slice(abs_pos.as_ne_bytes()).unwrap();
            }
            Self::Diagnostics(data) => {
                new_id |= 0x02 << 8;
                bytes.extend_from_slice(&data[..]).unwrap();
            }
            Self::Error(error_code) => {
                new_id |= 0x00 << 8;
                bytes.push(error_code).unwrap();
            }
        }

        Frame::new_data(
            bxcan::Id::Standard(bxcan::StandardId::new(new_id).unwrap()),
            bxcan::Data::new(&bytes[..]).unwrap(),
        )
    }
}

#[derive(Debug, Format)]
pub struct PriorityFrame(pub Frame);

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

impl Eq for PriorityFrame {}
impl PartialEq for PriorityFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
