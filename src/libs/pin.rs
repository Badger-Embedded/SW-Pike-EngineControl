use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::{self, ErasedPin, OpenDrain, PushPull};

pub type PINErasedPP = Output<ErasedPin<gpio::Output<PushPull>>, false>;
pub type PINErasedPPInv = Output<ErasedPin<gpio::Output<PushPull>>, true>;
pub type PINErasedOD = Output<ErasedPin<gpio::Output<OpenDrain>>, false>;
pub type PINErasedODInv = Output<ErasedPin<gpio::Output<OpenDrain>>, true>;

pub enum PinError {
    OutputError,
}

#[derive(Debug)]
pub struct Output<IO, const INVERTED: bool>(IO);
// type a = Pxx<_>;
impl<IO: OutputPin, const INVERTED: bool> Output<IO, INVERTED> {
    pub fn new(pin: IO) -> Self {
        Self { 0: pin }
    }

    pub fn enable(&mut self) -> Result<(), PinError> {
        if INVERTED {
            if let Ok(_) = self.0.set_low() {
                Ok(())
            } else {
                Err(PinError::OutputError)
            }
        } else if let Ok(_) = self.0.set_high() {
            Ok(())
        } else {
            Err(PinError::OutputError)
        }
    }
    pub fn disable(&mut self) -> Result<(), PinError> {
        if INVERTED {
            if let Ok(_) = self.0.set_high() {
                Ok(())
            } else {
                Err(PinError::OutputError)
            }
        } else if let Ok(_) = self.0.set_low() {
            Ok(())
        } else {
            Err(PinError::OutputError)
        }
    }
}
