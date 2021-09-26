use heapless::Vec;

use crate::pin::{PINErasedPP, PINErasedPPInv};

#[derive(Debug, PartialEq)]
pub enum PyroChannelName {
    Pyro1,
    Pyro2,
    Ignition,
}

#[derive(Debug, PartialEq)]
pub enum PyroError {
    ControllerIsFull,
    PyroChannelError,
}

pub struct PyroChannel {
    pub name: PyroChannelName,
    pub pin: PINErasedPP,
}

impl PyroChannel {
    pub fn enable(&mut self) -> Result<(), PyroError> {
        if let Ok(_) = self.pin.enable() {
            Ok(())
        } else {
            Err(PyroError::PyroChannelError)
        }
    }
    pub fn disable(&mut self) -> Result<(), PyroError> {
        if let Ok(_) = self.pin.disable() {
            Ok(())
        } else {
            Err(PyroError::PyroChannelError)
        }
    }
}

pub struct PyroController<const N: usize> {
    charge: PINErasedPP,
    discharge: PINErasedPPInv,
    channels: Vec<PyroChannel, N>,
}

impl<const N: usize> PyroController<N> {
    pub fn new(charge: PINErasedPP, discharge: PINErasedPPInv) -> Self {
        Self {
            charge,
            discharge,
            channels: Vec::new(),
        }
    }

    pub fn add_channel(&mut self, channel: PyroChannel) -> Result<(), PyroError> {
        if let Ok(_) = self.channels.push(channel) {
            Ok(())
        } else {
            Err(PyroError::ControllerIsFull)
        }
    }

    pub fn charge(&mut self) {
        self.charge.enable().ok().unwrap();
        self.discharge.disable().ok().unwrap();
    }

    pub fn discharge(&mut self) {
        self.discharge.enable().ok().unwrap();
        self.charge.disable().ok().unwrap();
    }

    pub fn continuous_state(&mut self) {
        self.charge.enable().ok().unwrap();
        self.discharge.enable().ok().unwrap();
    }

    pub fn activate_channel(&mut self, channel: PyroChannelName) {
        self.disable_all_channels();
        // TODO:
    }

    fn disable_all_channels(&mut self) {
        for channel in &mut self.channels {
            channel.disable().unwrap();
        }
    }
}
