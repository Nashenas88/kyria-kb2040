use crate::anim::{AnimState, AnimationController};
use crate::bsp::hal::pio::PIOExt;
use crate::bsp::hal::Sio;
use crate::bsp::pac::{CorePeripherals, Peripherals};
use crate::ws2812::Ws2812;
use fugit::RateExtU32;
use kyria_kb2040::layout::CustomAction;
use smart_leds::brightness;

const LED_ANIM_TIME_US: u32 = 3_000;
const CORE1_LOOP_US: u32 = 1_000;
const LED_BRIGHTNESS: u8 = 64;

pub(crate) fn core1_task() -> ! {
    let mut pac = unsafe { Peripherals::steal() };
    let core = unsafe { CorePeripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let pins = crate::bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // alarm1.enable_interrupt(&mut timer);

    // The first thing core0 sends us is the system bus frequency.
    // The systick is based on this frequency, so we need that to
    // be accurate when sleeping via cortex_m::delay::Delay
    let sys_freq = sio.fifo.read_blocking();
    let peripheral_freq = sio.fifo.read_blocking();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    let rgb = crate::pins::core1_pins(pins);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let peripheral_freq = peripheral_freq.Hz();
    let mut ws = Ws2812::new(rgb.into_mode(), &mut pio, sm0, peripheral_freq);
    let mut anim_controller = AnimationController::new();

    let mut counter = 0;
    let mut custom_action = None;
    loop {
        use smart_leds::SmartLedsWrite;
        delay.delay_us(CORE1_LOOP_US);
        counter += 1;
        if sio.fifo.is_read_ready() {
            custom_action = sio.fifo.read().map(|i| CustomAction::from(i as u8));
        }
        if counter >= LED_ANIM_TIME_US / CORE1_LOOP_US {
            counter = 0;

            if let Some(anim_state) =
                custom_action
                    .take()
                    .and_then(|custom_action| match custom_action {
                        CustomAction::QwertyLed => Some(AnimState::Qwerty),
                        CustomAction::ColemakLed => Some(AnimState::Colemak),
                        CustomAction::LayerSelectLed => Some(AnimState::LayerSelect),
                        CustomAction::SymLed => Some(AnimState::Sym),
                        _ => None,
                    })
            {
                anim_controller.set_state(anim_state);
            }

            // // Immediately clear the interrupt and schedule the next scan alarm.
            // a.clear_interrupt(t);
            // let _ = a.schedule(LED_ANIM_TIME_US.microseconds());

            // let mut ws = c.shared.ws;
            if anim_controller.tick() {
                anim_controller.set_state(AnimState::Colemak);
            }

            let leds = anim_controller.leds();
            SmartLedsWrite::write(&mut ws, brightness(leds.into_iter(), LED_BRIGHTNESS)).unwrap();
        }
    }
}
