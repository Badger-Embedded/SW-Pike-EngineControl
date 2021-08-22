#![no_main]
#![no_std]

use core::convert::TryInto;

use bxcan::{filter::Mask32, Instance, Interrupts};
use can_aerospace_lite::{
    message::CANAerospaceMessage,
    types::{DataType, MessageType},
    CANAerospaceLite,
};
use nb::block;
use panic_halt as _;
use pike_enginecontrol::can_driver::CANDriver;
use rtic::app;

use state_governor::{create_states, state::State, Governor};
use stm32f1xx_hal::{
    can::Can,
    device::TIM1,
    gpio::{gpioc::PC14, Output, PinState, PushPull},
    prelude::*,
    timer::{self, CountDownTimer, Timer},
};

// https://github.com/Badger-Embedded/Badger-Pike#engine-control
create_states!(IDLE, READY, IGNITION, PROPULSION, BURNOUT);

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        led_heartbeat: PC14<Output<PushPull>>,
        governor: Governor<5>,
        can_aerospace: CANAerospaceLite<CANDriver>,
        tim1_handler: CountDownTimer<TIM1>,
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

        let mut timer = Timer::tim1(cx.device.TIM1, &clocks).start_count_down(1.hz());
        timer.listen(timer::Event::Update);

        let can = Can::new(cx.device.CAN1, cx.device.USB);
        // Select pins for CAN1.
        let mut gpioa = cx.device.GPIOA.split();
        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        let mut afio = cx.device.AFIO.constrain();
        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);
        let mut can = bxcan::Can::new(can);
        // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        can.modify_config().set_bit_timing(0x001c_0000);

        can.modify_filters().enable_bank(0, Mask32::accept_all());

        // Sync to the bus and start normal operation.
        can.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        block!(can.enable()).unwrap();
        let (can_tx, can_rx) = can.split();
        let can_driver = CANDriver::new(can_tx, can_rx);

        // Acquire the GPIOC peripheral
        let mut gpioc = cx.device.GPIOC.split();

        // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the
        // function in order to configure the port. For pins 0-7, crl should be passed instead
        let led = gpioc
            .pc14
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);
        let mut can_aerospace = CANAerospaceLite::new(0xA, can_driver);
        governor.change_state_to(StateEnum::IDLE as u8);

        can_aerospace.send_message(CANAerospaceMessage::new(
            MessageType::EED(1),
            0xA,
            0x0,
            1,
            DataType::ULONG(0xDEAD_BEEF),
        ));

        // Init the static resources to use them later through RTIC
        init::LateResources {
            governor,
            can_aerospace,
            tim1_handler: timer,
            led_heartbeat: led,
        }
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

    #[task(binds = TIM1_UP, priority = 1, resources = [tim1_handler, led_heartbeat])]
    fn tick(cx: tick::Context) {
        // Depending on the application, you could want to delegate some of the work done here to
        // the idle task if you want to minimize the latency of interrupts with same priority (if
        // you have any). That could be done with some kind of machine state, etc.
        let led: &mut PC14<Output<PushPull>> = cx.resources.led_heartbeat;
        // Count used to change the timer update frequency
        led.toggle();
        // Clears the update flag
        cx.resources.tim1_handler.clear_update_interrupt_flag();
    }
};
