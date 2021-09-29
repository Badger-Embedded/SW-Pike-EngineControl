use core::convert::TryInto;
use heapless::{mpmc::Q8, Vec};
use pike_enginecontrol::{
    event::{Event, StateEvent},
    pyro::PyroState,
    state::{MovingState, StateTransition},
    StateEnum,
};

use rtic::mutex_prelude::*;
use state_governor::{state::State, Governor};

use crate::{app::state_handler, tasks::pyro_task::pyro_handler};

pub(crate) unsafe fn state_handler(
    mut cx: state_handler::Context,
    event: Option<StateEvent>,
    new_state: Option<StateEnum>,
) {
    // let transition: &'static mut TransitionContext = cx.local.context;
    static mut TRANSITION: StateTransition<PyroState, 5> = StateTransition {
        current_index: 0xFF,
        transition_path: Vec::new(),
    };

    if !TRANSITION.finished() {
        if let Some(fired_event) = event {
            let current_state = TRANSITION.state().unwrap();
            if PyroState::CHARGING
                .get_required_events(current_state)
                .contains(&fired_event)
            {
                if TRANSITION.next() {
                    crate::app::pyro_handler::spawn(&TRANSITION).ok().unwrap();
                }
            }
        }
    } else {
        // cx.local.led_cont.toggle();

        let current_state: StateEnum = cx
            .shared
            .governor
            .lock(|g: &mut Governor<5>| g.get_current_state().id())
            .try_into()
            .unwrap();
        if let Some(next_state) = new_state {
            if next_state == StateEnum::READY {
                TRANSITION.add_state(PyroState::CHARGING);
                TRANSITION.add_state(PyroState::READY);
                TRANSITION.add_state(PyroState::IDLE);
                TRANSITION.start();
                crate::app::pyro_handler::spawn(&TRANSITION).ok().unwrap();
            }
        } else {
            // match current_state {
            //     StateEnum::IDLE => crate::app::pyro_handler::spawn(PyroState::IDLE).unwrap(),
            //     StateEnum::READY => crate::app::pyro_handler::spawn(PyroState::READY).unwrap(),
            //     StateEnum::IGNITION => {}
            //     StateEnum::PROPULSION => {}
            //     StateEnum::BURNOUT => {}
            // }
        }
    }
}
