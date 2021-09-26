#![no_main]
#![no_std]

use core::convert::TryInto;

use bxcan::{filter::Mask32, Interrupts};
use can_aerospace_lite::CANAerospaceLite;
use embedded_hal::digital::v2::OutputPin;
use nb::block;
use panic_halt as _;
use pike_enginecontrol::{
    can_driver::CANDriver,
    pin,
    pyro::{self, PyroChannel, PyroChannelName, PyroController},
};
use rtic::app;

use state_governor::{create_states, state::State, Governor};
use stm32f1xx_hal::{
    afio,
    can::Can,
    device::{CAN1, TIM1, USB},
    gpio::{
        gpioa::{PA11, PA12, PA15},
        gpiob::PB13,
        Alternate, Floating, Input,
    },
    gpio::{
        gpiob::{PB14, PB15},
        gpioc::{PC13, PC14, PC15},
        ErasedPin, Output, PinState, PushPull,
    },
    i2c::{BlockingI2c, Mode},
    prelude::*,
    timer::{self, CountDownTimer, Timer},
};
// use mpl3115::MPL3115A2;

// https://github.com/Badger-Embedded/Badger-Pike#engine-control
create_states!(IDLE, READY, IGNITION, PROPULSION, BURNOUT);

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        governor: Governor<5>,
        can_aerospace: CANAerospaceLite<CANDriver>,
        tim1_handler: CountDownTimer<TIM1>,
        led_heartbeat: PC14<Output<PushPull>>,
        // led_cont: PB15<Output<PushPull>>,
        // charge: PC15<Output<PushPull>>,
        // n_discharge: PB14<Output<PushPull>>,
        ign0: PC13<Output<PushPull>>,
        pyro1: PB13<Output<PushPull>>,
        pyro2: PA15<Output<PushPull>>,
        // altitude_sensor:
        //     MPL3115A2<BlockingI2c<I2C2, (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>)>>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
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

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpioc = cx.device.GPIOC.split();
        let mut gpiob = cx.device.GPIOB.split();
        let (pa15, _, _) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        let can_driver = initialize_canbus(
            cx.device.CAN1,
            cx.device.USB,
            can_rx_pin,
            can_tx_pin,
            &mut afio,
        );

        let mut timer = Timer::tim1(cx.device.TIM1, &clocks).start_count_down(1.hz());
        timer.listen(timer::Event::Update);

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        // TODO: initialize peripherals in separate function
        let led_heartbeat = gpioc
            .pc14
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);
        let led_cont: PB15<Output<PushPull>> = gpiob
            .pb15
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);

        // let mut p_channel = PyroChannel {
        //     name: PyroChannelName::Pyro1,
        //     pin: led_cont,
        // };
        // p_channel.enable().unwrap();

        let charge = pin::Output::new(
            gpioc
                .pc15
                .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High)
                .erase(),
        );

        let n_discharge = pin::Output::new(
            gpiob
                .pb14
                .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low)
                .erase(),
        );
        let mut pyro_controller = PyroController::<3>::new(charge, n_discharge);
        pyro_controller.continuous_state();
        // pyro_controller.add_channel(PyroChannel {
        //     name: PyroChannelName::Pyro1,
        //     pin: pin::Output::new(led_cont),
        // });

        let ign0: PC13<Output<PushPull>> = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);
        let pyro1: PB13<Output<PushPull>> = gpiob
            .pb13
            .into_push_pull_output_with_state(&mut gpiob.crh, PinState::Low);
        let pyro2: PA15<Output<PushPull>> =
            pa15.into_push_pull_output_with_state(&mut gpioa.crh, PinState::Low);

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
        init::LateResources {
            governor,
            can_aerospace,
            tim1_handler: timer,
            led_heartbeat,
            // led_cont,
            // charge,
            // n_discharge,
            ign0,
            pyro1,
            pyro2,
            // altitude_sensor,
        }
    }

    // Optional.
    //
    // https://rtic.rs/0.5/book/en/by-example/app.html#idle
    // > When no idle function is declared, the runtime sets the SLEEPONEXIT bit and then
    // > sends the microcontroller to sleep after running init.
    #[idle(resources=[governor, can_aerospace])]
    fn idle(mut cx: idle::Context) -> ! {
        let governor: &mut Governor<5> = cx.resources.governor;
        loop {
            let _message = cx.resources.can_aerospace.lock(
                |can_aerospace: &mut CANAerospaceLite<CANDriver>| can_aerospace.read_message(),
            );
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
            cortex_m::asm::nop();
        }
    }

    #[task(binds = TIM1_UP, resources = [tim1_handler, led_heartbeat])]
    fn tick(cx: tick::Context) {
        let led: &mut PC14<Output<PushPull>> = cx.resources.led_heartbeat;
        led.toggle();

        // Clears the update flag
        cx.resources.tim1_handler.clear_update_interrupt_flag();
    }

    #[task(binds = USB_LP_CAN_RX0, resources = [can_aerospace])]
    fn can_rx0(cx: can_rx0::Context) {
        let can_aerospace: &mut CANAerospaceLite<CANDriver> = cx.resources.can_aerospace;

        can_aerospace.notify_receive_event();
    }
    extern "C" {
        fn EXTI0();
    }
};

fn on_state_transition(curr_state: Option<State>, next_state: Option<State>) -> bool {
    true
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
