use crate::pyro::PyroState;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    StateInfo(StateEvent),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StateEvent {
    System(crate::StateEnum),
    Pyro(PyroState),
}
