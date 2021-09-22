use bxcan::{Frame, Rx, Tx};
use can_aerospace_lite::{driver::CANAerospaceDriver, message::CANAerospaceFrame};
use stm32f1xx_hal::{can::Can, device::CAN1};

pub struct CANDriver {
    tx: Tx<Can<CAN1>>,
    rx: Rx<Can<CAN1>>,
}

impl CANDriver {
    pub fn new(tx: Tx<Can<CAN1>>, rx: Rx<Can<CAN1>>) -> Self {
        Self { tx, rx }
    }
}

impl CANAerospaceDriver for CANDriver {
    fn send_frame(&mut self, frame: can_aerospace_lite::message::CANAerospaceFrame) {
        if let Ok(_) = self.tx.transmit(&Frame::from(&frame)) {};
    }

    fn recv_frame(&mut self) -> Option<can_aerospace_lite::message::CANAerospaceFrame> {
        loop {
            match self.rx.receive() {
                Ok(frame) => return Some(CANAerospaceFrame::from(frame)),
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => continue, // Ignore overrun errors.
            }
        }
        None
    }
}
