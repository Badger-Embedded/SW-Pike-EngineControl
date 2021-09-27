use crate::app::pyro_handler;
use heapless::spsc::Producer;
use pike_enginecontrol::{event::Event, pyro::PyroController, StateEnum};
use rtic::mutex_prelude::*;

pub(crate) fn pyro_handler(
    mut cx: pyro_handler::Context,
    system_state: StateEnum,
    // poi_channel: PyroChannelName,
) {
    let controller: &mut PyroController<3> = cx.local.pyro_controller;
    match system_state {
        StateEnum::IDLE => {
            controller.continuous_state();
            cx.shared
                .event_write
                .lock(|q: &mut Producer<'static, Event, 10>| {
                    q.enqueue(Event::StateChangeRequest(StateEnum::READY)).ok();
                });
            controller.set_ready(false);
        }
        StateEnum::READY => {
            if !controller.is_ready() {
                controller.charge();

                cx.shared
                    .event_write
                    .lock(|q: &mut Producer<'static, Event, 10>| {
                        q.enqueue(Event::PyroControllerCharging).ok();
                    });
                controller.set_ready(true);
            }
        }
        StateEnum::IGNITION => {
            if controller.is_ready() {
                // controller.closed_state();
                // cx.shared
                //     .event_write
                //     .lock(|q: &mut Producer<'static, Event, 10>| {
                //         q.enqueue(Event::PyroControllerReady).ok();
                //     });
            }
        }
        StateEnum::PROPULSION => {}
        StateEnum::BURNOUT => {}
    }
}
