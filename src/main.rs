#![no_main]
#![no_std]

use core::convert::TryInto;

use panic_halt as _;
use rtic::app;

use state_governor::{create_states, state::State, Governor};
use stm32f1xx_hal::prelude::*;

// https://github.com/talhaHavadar/Badger-Pike#engine-control
create_states!(IDLE, READY, IGNITION, PROPULSION, BURNOUT);

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources<'a> {
        governor: Governor<5>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut governor = Governor::new();
        governor.add_state(State::from(StateEnum::IDLE));
        governor.add_state(State::from(StateEnum::READY));
        governor.add_state(State::from(StateEnum::IGNITION));
        governor.add_state(State::from(StateEnum::PROPULSION));
        governor.add_state(State::from(StateEnum::BURNOUT));

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies
        // in `clocks`
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(64.mhz())
            .hclk(64.mhz())
            .pclk1(16.mhz())
            .pclk2(64.mhz())
            .freeze(&mut flash.acr);

        // Acquire the GPIOC peripheral
        // let mut gpioc = cx.device.GPIOC.split();

        // // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // // function in order to configure the port. For pins 0-7, crl should be passed instead
        // let led = gpioc
        //     .pc13
        //     .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);

        governor.change_state_to(StateEnum::IDLE as u8);
        // Init the static resources to use them later through RTIC
        init::LateResources { governor }
    }

    // Optional.
    //
    // https://rtic.rs/0.5/book/en/by-example/app.html#idle
    // > When no idle function is declared, the runtime sets the SLEEPONEXIT bit and then
    // > sends the microcontroller to sleep after running init.
    #[idle(resources=[governor])]
    fn idle(cx: idle::Context) -> ! {
        let governor: &mut Governor<5> = cx.resources.governor;
        loop {
            match governor.get_current_state().id().try_into() {
                Ok(StateEnum::IDLE) => {}
                Ok(StateEnum::READY) => {}
                Ok(StateEnum::IGNITION) => {}
                Ok(StateEnum::PROPULSION) => {}
                Ok(StateEnum::BURNOUT) => {}
                Err(_) => {}
            }

            cortex_m::asm::nop();
        }
    }
};
