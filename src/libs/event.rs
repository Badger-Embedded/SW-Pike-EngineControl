use crate::pyro::PyroState;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    PyroStateInfo(PyroState),
    StateInfo(StateEvent),
    StateChangeRequest(crate::StateEnum),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StateEvent {
    Pyro(PyroState),
}
