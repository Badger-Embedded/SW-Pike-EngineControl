use heapless::Vec;

use crate::{
    event::StateEvent,
    pin::{PINErasedPP, PINErasedPPInv},
    state::MovingState,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PyroState {
    IDLE,
    CHARGING,
    DISCHARGING,
    READY,
    FIRING(PyroChannelName),
}

impl MovingState for PyroState {
    fn get_required_events(&self, state: PyroState) -> Vec<StateEvent, 5_usize> {
        let mut events: Vec<StateEvent, 5> = Vec::new();
        match state {
            PyroState::IDLE => events.push(StateEvent::Pyro(PyroState::IDLE)).unwrap(),
            PyroState::CHARGING => events.push(StateEvent::Pyro(PyroState::CHARGING)).unwrap(),
            PyroState::DISCHARGING => events
                .push(StateEvent::Pyro(PyroState::DISCHARGING))
                .unwrap(),
            PyroState::READY => events.push(StateEvent::Pyro(PyroState::READY)).unwrap(),
            PyroState::FIRING(_) => events
                .push(StateEvent::Pyro(PyroState::FIRING(PyroChannelName::Any)))
                .unwrap(),
        }
        events
    }

    fn is_transition_allowed(&self, state: PyroState) -> bool {
        match self {
            PyroState::IDLE => state == PyroState::CHARGING || state == PyroState::DISCHARGING,
            PyroState::CHARGING => state == PyroState::DISCHARGING || state == PyroState::READY,
            PyroState::DISCHARGING => state == PyroState::CHARGING,
            PyroState::READY => {
                if let PyroState::FIRING(_) = state {
                    true
                } else {
                    PyroState::IDLE == state
                }
            }
            PyroState::FIRING(_) => state == PyroState::CHARGING || state == PyroState::DISCHARGING,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PyroChannelName {
    Pyro1,
    Pyro2,
    Ignition,
    Any,
}

#[derive(Debug, PartialEq)]
pub enum PyroError {
    ControllerIsFull,
    PyroChannelError,
    StateChangeError(PyroState),
}

pub struct PyroChannel {
    pub name: PyroChannelName,
    pub pin: PINErasedPP,
}

impl PyroChannel {
    pub fn enable(&mut self) -> Result<(), PyroError> {
        if self.pin.enable().is_ok() {
            Ok(())
        } else {
            Err(PyroError::PyroChannelError)
        }
    }
    pub fn disable(&mut self) -> Result<(), PyroError> {
        if self.pin.disable().is_ok() {
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
    ready: bool,
    state: PyroState,
}

impl<const N: usize> PyroController<N> {
    pub fn new(charge: PINErasedPP, discharge: PINErasedPPInv) -> Self {
        Self {
            charge,
            discharge,
            channels: Vec::new(),
            ready: false,
            state: PyroState::IDLE,
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
        self.disable_all_channels();
        self.discharge.disable().ok().unwrap();
        self.charge.enable().ok().unwrap();
    }

    pub fn discharge(&mut self) {
        self.disable_all_channels();
        self.charge.disable().ok().unwrap();
        self.discharge.enable().ok().unwrap();
    }

    pub fn continuous_state(&mut self) {
        self.disable_all_channels();
        self.charge.enable().ok().unwrap();
        self.discharge.enable().ok().unwrap();
    }

    pub fn closed_state(&mut self) {
        self.disable_all_channels();
        self.discharge.disable().ok().unwrap();
        self.charge.disable().ok().unwrap();
    }

    pub fn fire(&mut self, channel_name: PyroChannelName) {
        for channel in &mut self.channels {
            if channel.name == channel_name {
                channel.enable().unwrap();
                break;
            }
        }
    }

    pub fn change_state(&mut self, new_state: PyroState) -> Result<bool, PyroError> {
        // TODO
        Ok(false)
    }

    pub fn is_ready(&self) -> bool {
        self.ready
    }

    pub fn set_ready(&mut self, ready: bool) {
        self.ready = ready
    }

    pub fn get_state(&self) -> PyroState {
        self.state
    }

    fn disable_all_channels(&mut self) {
        for channel in &mut self.channels {
            channel.disable().unwrap();
        }
    }
}
