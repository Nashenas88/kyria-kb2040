use crate::bsp::hal::pac::{self, interrupt};
use crate::bsp::hal::pio::{PIOExt, SM0};
use crate::bsp::hal::sio::SioFifo;
use crate::bsp::hal::timer::{Alarm, Alarm1};
use crate::bsp::hal::{Sio, Timer};
use crate::bsp::pac::{CorePeripherals, Peripherals, PIO0};
use crate::pins::{Core1Pins, RgbPin, RotaryPin};
use arraydeque::ArrayDeque;
use core::sync::atomic::{AtomicU8, Ordering};
use fugit::{ExtU32, MicrosDurationU32, RateExtU32};
use keyberon::layout::Event;
use kyria_kb2040::anim::{AnimKind, AnimationController, SwitchKind};
use kyria_kb2040::cross_talk::{InterProcessCommand, InterProcessResponse};
use kyria_kb2040::layout::{CustomAction, PressEvent, SerializableEvent};
use kyria_kb2040::log;
use kyria_kb2040::rotary::update_last_rotary;
use rotary_encoder_hal::Rotary;
use smart_leds::brightness;
use ws2812_pio::Ws2812Direct;

const LED_ANIM_HZ: u32 = 60; // 3ms
const LED_ANIM_PERIOD_US: u32 = 1_000_000 / LED_ANIM_HZ;
const LED_BRIGHTNESS: u8 = 64;
/// Number of scan cycles to wait before triggering rotary release event.
const ROTARY_WAIT: u8 = 1;

static mut SIO_FIFO: Option<SioFifo> = None;
static mut IS_RIGHT: bool = false;
static mut ROTARY: Option<Rotary<RotaryPin, RotaryPin>> = None;
static mut LAST_ROTARY: Option<(Event, u8)> = None;
static mut ALARM1: Option<Alarm1> = None;
static mut STATE: Option<&'static AtomicU8> = None;
static mut ANIM_CONTROLLER: Option<AnimationController> = None;
static mut WS: Option<Ws2812Direct<PIO0, SM0, RgbPin>> = None;

pub(crate) fn core1_task() {
    let mut pac = unsafe { Peripherals::steal() };
    let _core = unsafe { CorePeripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let pins = crate::bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The first thing core0 sends us is the system bus frequency.
    // The systick is based on this frequency, so we need that to
    // be accurate when sleeping via cortex_m::delay::Delay
    let _sys_freq = sio.fifo.read_blocking();
    let peripheral_freq = sio.fifo.read_blocking();
    let state_addr = sio.fifo.read_blocking();
    let is_right = sio.fifo.read_blocking() != 0;
    let timer_addr = sio.fifo.read_blocking();

    // Safety: We know that the address is valid because it was sent by core0.
    let state: &'static AtomicU8 = unsafe { core::mem::transmute(state_addr) };

    // Safety: We know that the address is valid because it was sent by core0.
    let timer: &'static Option<Timer> = unsafe { core::mem::transmute(timer_addr) };
    // Copy the timer so we can safely use it mutably.
    let mut timer: Timer = *timer.as_ref().unwrap();

    // Use alarm1 since alarm0 is used for the scan timer on core0.
    let mut alarm1 = timer.alarm_1().unwrap();
    alarm1.schedule(LED_ANIM_PERIOD_US.micros()).unwrap();
    alarm1.enable_interrupt();

    let Core1Pins {
        rgb,
        rotary1,
        rotary2,
    } = crate::pins::core1_pins(pins);
    let mut rotary = if is_right {
        Rotary::<RotaryPin, RotaryPin>::new(rotary1.into_dyn_pin(), rotary2.into_dyn_pin())
    } else {
        // TODO how to make this per-board?
        Rotary::<RotaryPin, RotaryPin>::new(rotary2.into_dyn_pin(), rotary1.into_dyn_pin())
    };
    // Read rotary since sometimes the first read triggers a false positive.
    let _result = rotary.update();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let peripheral_freq = peripheral_freq.Hz();
    let ws = Ws2812Direct::new(rgb, &mut pio, sm0, peripheral_freq);
    let anim_controller = AnimationController::new();

    // Safety: Interrupts are not enabled, nothing else is accessing these fields.
    unsafe {
        IS_RIGHT = is_right;
        STATE = Some(state);
        ALARM1 = Some(alarm1);
        ROTARY = Some(rotary);
        WS = Some(ws);
        ANIM_CONTROLLER = Some(anim_controller);
        SIO_FIFO = Some(sio.fifo);
    }

    // Unmask the alarm and fifo interrups so we can process requests from core 0.
    unsafe {
        // We want sio to have the highest priority
        // core.NVIC.set_priority(
        //     pac::interrupt::SIO_IRQ_PROC1,
        //     0 << (8 - pac::NVIC_PRIO_BITS),
        // );
        // // Followed by the timer.
        // core.NVIC
        //     .set_priority(pac::interrupt::TIMER_IRQ_1, 1 << (8 - pac::NVIC_PRIO_BITS));

        // Unmask the interrupts so they can finally fire.
        pac::NVIC::unmask(pac::interrupt::TIMER_IRQ_1);
        pac::NVIC::unmask(pac::interrupt::SIO_IRQ_PROC1);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

static mut LED_STATES: ArrayDeque<AnimKind, 2> = ArrayDeque::new();

#[interrupt]
fn TIMER_IRQ_1() {
    use smart_leds::SmartLedsWrite;

    let anim_micros: MicrosDurationU32 = LED_ANIM_PERIOD_US.micros();
    let anim_millis = anim_micros.to_millis();

    // Safety: ALARM1 is only accessed here after init.
    let alarm1 = unsafe { ALARM1.as_mut().unwrap() };
    alarm1.clear_interrupt();
    alarm1.schedule(anim_micros).unwrap();

    // Safety: STATE, ANIM_CONTROLLER, WS, and LED_STATES are only accessed here
    // after init.
    let (state, anim_controller, ws, led_states) = unsafe {
        (
            STATE.as_ref().unwrap(),
            ANIM_CONTROLLER.as_mut().unwrap(),
            WS.as_mut().unwrap(),
            &mut LED_STATES,
        )
    };

    let state_val = state.load(Ordering::Acquire);
    state.store(0, Ordering::Release);
    if let Some(event) = SerializableEvent::deserialize(state_val) {
        if let Some(anim_state) = match event.action {
            CustomAction::QwertyLed => Some(AnimKind::Qwerty),
            CustomAction::ColemakLed => Some(AnimKind::Colemak),
            CustomAction::LayerSelectLed => Some(AnimKind::LayerSelect),
            CustomAction::NumpadLed => Some(AnimKind::Num),
            CustomAction::NavLed => Some(AnimKind::Nav),
            CustomAction::SymLed => Some(AnimKind::Sym),
            CustomAction::FunctionLed => Some(AnimKind::Function),
            CustomAction::RainbowLed => Some(AnimKind::Rainbow),
            _ => None,
        } {
            if let (SwitchKind::Momentary, PressEvent::Press) =
                (anim_state.switch_kind(), event.event)
            {
                led_states.push_back(anim_controller.state()).unwrap();
                anim_controller.set_state(anim_state);
            } else if let (SwitchKind::Momentary, PressEvent::Release) =
                (anim_state.switch_kind(), event.event)
            {
                if anim_controller.state() == anim_state {
                    if let Some(anim_state) = led_states.pop_back() {
                        anim_controller.set_state(anim_state);
                    }
                } else if led_states.back() == Some(&anim_state) {
                    led_states.pop_back();
                } else {
                    led_states.clear();
                }
            } else if let PressEvent::Press = event.event {
                anim_controller.set_state(anim_state);
                led_states.clear();
            }
        }
    }

    if anim_controller.tick(anim_millis) {
        if let AnimKind::Boot = anim_controller.state() {
            anim_controller.force_state(AnimKind::Colemak);
        }
    }

    let leds = anim_controller.leds();
    SmartLedsWrite::write(ws, brightness(leds.into_iter(), LED_BRIGHTNESS)).unwrap();
}

#[interrupt]
fn SIO_IRQ_PROC1() {
    // Safety: No other code accesses this static.
    let sio_fifo = unsafe { SIO_FIFO.as_mut().unwrap() };

    let val = sio_fifo.read_blocking();
    let Ok(command) = InterProcessCommand::try_from(val) else {
        log::error!("Invalid command: {}", val);
        sio_fifo.write_blocking(InterProcessResponse::Error.into());
        return;
    };

    let response = match command {
        InterProcessCommand::ReadRotary => InterProcessResponse::Rotary(read_rotary()).into(),
    };
    sio_fifo.write_blocking(response);
}

fn read_rotary() -> Option<Event> {
    let is_right = unsafe { IS_RIGHT };
    let rotary = unsafe { ROTARY.as_mut().unwrap() };
    let last_rotary = unsafe { &mut LAST_ROTARY };
    let dir = rotary.update().unwrap();

    if is_right {
        update_last_rotary::<ROTARY_WAIT>(last_rotary, dir, 14, 12)
    } else {
        update_last_rotary::<ROTARY_WAIT>(last_rotary, dir, 3, 1)
    }
}
