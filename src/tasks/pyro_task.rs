use crate::app::pyro_handler;
use heapless::mpmc::Q8;
use pike_enginecontrol::{
    event::{Event, StateEvent},
    pyro::{PyroController, PyroError, PyroState},
    state::StateTransition,
};
use rtic::mutex_prelude::*;

pub(crate) fn pyro_handler(
    mut cx: pyro_handler::Context,
    transition: &mut StateTransition<PyroState, 5>,
) {
    let controller: &mut PyroController<3> = cx.local.pyro_controller;
    // cx.local.led_cont.toggle();
    if !transition.finished() {
        let state = transition.state().unwrap();

        match state {
            PyroState::IDLE => {
                controller.continuous_state();
            }
            PyroState::CHARGING => {
                controller.charge();
            }
            PyroState::DISCHARGING => {
                controller.discharge();
            }
            PyroState::READY => {
                controller.closed_state(); // Disconnects the connection between battery and discharge circuit
            }
            PyroState::FIRING(channel) => {
                controller.fire(channel);
            }
        }
        transition.next();
        if transition.finished() {
            transition.reset();
        }
        cx.shared.event_q.lock(|q: &mut Q8<Event>| {
            q.enqueue(Event::StateInfo(StateEvent::Pyro(state))).ok();
        });
    }
}
