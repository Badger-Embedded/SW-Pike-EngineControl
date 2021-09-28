pub enum Event {
    PyroControllerCharging,
    PyroControllerDischarging,
    PyroControllerReady,
    StateChangeRequest(crate::StateEnum),
}
