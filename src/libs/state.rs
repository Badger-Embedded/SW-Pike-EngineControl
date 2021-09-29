use heapless::Vec;

use crate::event::StateEvent;

pub struct StateTransition<S: MovingState + Copy, const N: usize> {
    pub current_index: usize,
    pub transition_path: Vec<S, N>,
}

impl<S: MovingState + Copy, const N: usize> StateTransition<S, N> {
    pub fn new() -> Self {
        Self {
            current_index: 0xFF,
            transition_path: Vec::new(),
        }
    }

    pub fn add_state(&mut self, state: S) {
        self.transition_path.push(state).ok().unwrap();
    }

    pub fn next(&mut self) -> bool {
        if !self.finished() {
            let curr_state: S = self.transition_path[self.current_index];
            let next_state: S = self.transition_path[self.current_index + 1];
            if curr_state.is_transition_allowed(next_state) {
                self.current_index += 1;
                return true;
            } else {
                return false;
            }
        }
        false
    }

    pub fn finished(&self) -> bool {
        self.current_index >= self.transition_path.len()
    }

    pub fn start(&mut self) {
        self.current_index = 0;
    }

    pub fn state(&self) -> Option<S> {
        if !self.finished() {
            return Some(self.transition_path[self.current_index]);
        }
        None
    }
}

impl<S: MovingState + Copy, const N: usize> Default for StateTransition<S, N> {
    fn default() -> Self {
        Self::new()
    }
}
pub trait MovingState {
    fn get_required_events(&self, state: Self) -> Vec<StateEvent, 5>;
    fn is_transition_allowed(&self, state: Self) -> bool;
}
