#![no_main]
#![no_std]
use bxcan::{filter::Mask32, Interrupts};
use nb::block;
use panic_halt as _;

use pike_enginecontrol::{
    can_driver::CANDriver,
    pin::{PINErasedPP, PINErasedPPInv},
    pyro::{PyroChannel, PyroChannelName, PyroController},
};
use rtic::app;
use state_governor::state::State;
use stm32f1xx_hal::{
    afio,
    can::Can,
    device::{CAN1, USB},
    gpio::{
        gpioa::{PA11, PA12},
        Alternate, Floating, Input, PushPull,
    },
};
// use mpl3115::MPL3115A2;

const TIMER_FREQ: u32 = 1;

#[app(device = stm32f1xx_hal::pac, peripherals = true,dispatchers = [EXTI0])]
mod APP {
    use can_aerospace_lite::CANAerospaceLite;
    use core::convert::TryInto;
    use pike_enginecontrol::{can_driver::CANDriver, pin::Output, pyro::PyroController};
    use rtic::{rtic_monotonic::Instant, time::duration::*};

    use state_governor::{create_states, state::State, Governor};
    use stm32f1xx_hal::{
        delay::Delay,
        device::TIM1,
        gpio::{self},
        gpio::{gpiob::PB15, gpioc::PC14, PinState, PushPull},
        i2c::{BlockingI2c, Mode},
        prelude::*,
        timer::{self, CountDownTimer, Timer},
    };
    use systick_monotonic::Systick;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<100>; // 100 Hz / 10 ms granularity

    // https://github.com/Badger-Embedded/Badger-Pike#engine-control
    create_states!(IDLE, READY, IGNITION, PROPULSION, BURNOUT);

    #[local]
    struct Local {
        timer: CountDownTimer<TIM1>,
        pyro_controller: PyroController<3>,
        led_heartbeat: PC14<gpio::Output<PushPull>>,
    }

    #[shared]
    struct Shared {
        // delay: Delay,
        governor: Governor<5>,
        can_aerospace: CANAerospaceLite<CANDriver>,

        led_cont: PB15<gpio::Output<PushPull>>,
        // altitude_sensor:
        //     MPL3115A2<BlockingI2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut governor = Governor::new();
        governor.add_state(State::from(StateEnum::IDLE));
        governor.add_state(State::from(StateEnum::READY));
        governor.add_state(State::from(StateEnum::IGNITION));
        governor.add_state(State::from(StateEnum::PROPULSION));
        governor.add_state(State::from(StateEnum::BURNOUT));
        governor.set_state_transition_func(on_state_transition);
        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain();
        // let cp = cortex_m::Peripherals::take().unwrap();

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

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, 64_000_000);
        // let delay = Delay::new(cx.core.SYST, clocks);
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpioc = cx.device.GPIOC.split();
        let mut gpiob = cx.device.GPIOB.split();
        let (pa15, _, _) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        let can_driver = crate::initialize_canbus(
            cx.device.CAN1,
            cx.device.USB,
            can_rx_pin,
            can_tx_pin,
            &mut afio,
        );

        let mut timer =
            Timer::tim1(cx.device.TIM1, &clocks).start_count_down(crate::TIMER_FREQ.hz());
        timer.listen(timer::Event::Update);

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        // TODO: initialize peripherals in separate function
        let led_heartbeat = gpioc
            .pc14
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);
        let led_cont: PB15<gpio::Output<PushPull>> = gpiob
            .pb15
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);

        let pyro_controller = crate::initialize_pyro_controller(
            Output::new(
                gpioc
                    .pc15
                    .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High)
                    .erase(),
            ),
            Output::new(
                gpiob
                    .pb14
                    .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low)
                    .erase(),
            ),
            Output::new(
                gpiob
                    .pb13
                    .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low)
                    .erase(),
            ),
            Output::new(
                pa15.into_push_pull_output_with_state(&mut gpioa.crh, PinState::Low)
                    .erase(),
            ),
            Output::new(
                gpioc
                    .pc13
                    .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low)
                    .erase(),
            ),
        );

        let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        let _i2c = BlockingI2c::i2c2(
            cx.device.I2C2,
            (scl, sda),
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: stm32f1xx_hal::i2c::DutyCycle::Ratio16to9,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );
        // let altitude_sensor = MPL3115A2::new(i2c, mpl3115::PressureAlt::Altitude).unwrap();
        let can_aerospace = CANAerospaceLite::new(0xA, can_driver);
        governor.change_state_to(StateEnum::IDLE as u8);

        // Init the static resources to use them later through RTIC
        (
            Shared {
                // delay,
                governor,
                can_aerospace,
                led_cont,
                // altitude_sensor,
            },
            Local {
                timer,
                pyro_controller,
                led_heartbeat,
            },
            init::Monotonics(mono),
        )
    }

    // Optional.
    //
    // https://rtic.rs/0.5/book/en/by-example/app.html#idle
    // > When no idle function is declared, the runtime sets the SLEEPONEXIT bit and then
    // > sends the microcontroller to sleep after running init.
    #[idle(shared=[governor, can_aerospace])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut toggle = false;
        pyro_task::spawn(true).unwrap();
        let mut start_time: Instant<_> = monotonics::now();
        loop {
            let _message =
                cx.shared
                    .can_aerospace
                    .lock(|can_aerospace: &mut CANAerospaceLite<CANDriver>| {
                        can_aerospace.read_message()
                    });
            let instant: Instant<_> = monotonics::now();
            let duration = instant.checked_duration_since(&start_time).unwrap();
            let milliseconds: Milliseconds<u32> = duration.try_into().unwrap();
            if milliseconds >= Milliseconds(5000u32) {
                pyro_task::spawn(toggle).unwrap();
                start_time = monotonics::now();
                toggle = !toggle;
            }
            cx.shared.governor.lock(|governor| {
                // TODO: prioritize critical messages, create seperate functions for each state
                match governor.get_current_state().id().try_into() {
                    Ok(StateEnum::IDLE) => {
                        // TODO: check continuity
                        // TODO: read sensors and check the system is stable

                        // TODO: idle to ready transition
                        // TODO: check continuity, continuity must be preserved
                        // TODO: disable discharge, charge the capacitor
                        // TODO: after 3 seconds, disable charge
                        governor.change_state_to(StateEnum::READY as u8);
                    }
                    Ok(StateEnum::READY) => {
                        // TODO: check continuity, continuity must be preserved, otherwise mission abort!
                        // TODO: ready to ignition transition
                        // TODO: ready to ignition transition; set ign0
                    }
                    Ok(StateEnum::IGNITION) => {
                        // TODO: check continuity
                        // TODO: clear ign0
                        // TODO: check lift-off; if it is then ignition is successfull, change state to propulsion
                    }
                    Ok(StateEnum::PROPULSION) => {
                        // TODO: set charge for 3 seconds (one-time)
                        // TODO: read accelerometer messages to detect burnout
                        // TODO: read can messages incase of any pyro-action
                    }
                    Ok(StateEnum::BURNOUT) => {
                        // TODO: read can messages incase of any pyro-action
                    }
                    Err(_) => {}
                }
            });

            cortex_m::asm::nop();
        }
    }

    #[task(capacity=5, local=[pyro_controller])]
    fn pyro_task(cx: pyro_task::Context, open: bool) {
        if open {
            cx.local.pyro_controller.continuous_state();
        } else {
            cx.local.pyro_controller.closed_state();
        }
    }

    #[task(binds = TIM1_UP, local= [led_heartbeat, timer])]
    fn tick(cx: tick::Context) {
        let timer: &mut CountDownTimer<TIM1> = cx.local.timer;

        cx.local.led_heartbeat.toggle();

        // Clears the update flag
        timer.clear_update_interrupt_flag();
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [can_aerospace])]
    fn can_rx0(cx: can_rx0::Context) {
        let mut can_aerospace = cx.shared.can_aerospace;

        can_aerospace.lock(|can| can.notify_receive_event());
    }

    fn on_state_transition(curr_state: Option<State>, next_state: Option<State>) -> bool {
        true
    }

    // use pike_enginecontrol::pyro::pyro_task;

    // RTIC docs specify we can modularize the code by using these `extern` blocks.
    // This allows us to specify the tasks in other modules and still work within
    // RTIC's infrastructure.
    extern "Rust" {
        // #[task()]
        // fn pyro_task(context: tim8_cc::Context);
    }
}

fn initialize_pyro_controller(
    charge: PINErasedPP,
    discharge: PINErasedPPInv,
    pyro1: PINErasedPP,
    pyro2: PINErasedPP,
    ignition: PINErasedPP,
) -> PyroController<3> {
    let mut pyro_controller = PyroController::<3>::new(charge, discharge);
    pyro_controller
        .add_channel(PyroChannel {
            name: PyroChannelName::Pyro1,
            pin: pyro1,
        })
        .unwrap();

    pyro_controller
        .add_channel(PyroChannel {
            name: PyroChannelName::Ignition,
            pin: ignition,
        })
        .unwrap();

    pyro_controller
        .add_channel(PyroChannel {
            name: PyroChannelName::Pyro2,
            pin: pyro2,
        })
        .unwrap();
    pyro_controller
}

fn initialize_canbus(
    can: CAN1,
    usb: USB,
    can_rx_pin: PA11<Input<Floating>>,
    can_tx_pin: PA12<Alternate<PushPull>>,
    afio: &mut afio::Parts,
) -> CANDriver {
    let can_peripheral = Can::new(can, usb);
    can_peripheral.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

    let mut can = bxcan::Can::new(can_peripheral);

    // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    can.modify_config().set_bit_timing(0x001c_0000);

    can.modify_filters().enable_bank(0, Mask32::accept_all());

    // Sync to the bus and start normal operation.
    can.enable_interrupts(Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING);
    block!(can.enable()).unwrap();
    let (can_tx, can_rx) = can.split();
    CANDriver::new(can_tx, can_rx)
}
