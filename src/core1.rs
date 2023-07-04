use crate::bsp::hal::pac::I2C1;
use crate::bsp::hal::pac::{self, interrupt};
use crate::bsp::hal::pio::{PIOExt, SM0};
use crate::bsp::hal::sio::SioFifo;
use crate::bsp::hal::timer::{Alarm, Alarm1};
use crate::bsp::hal::I2C;
use crate::bsp::hal::{Sio, Timer};
use crate::bsp::pac::{CorePeripherals, Peripherals, PIO0};
use crate::pins::{Core1Pins, DisplayI2cScl, DisplayI2cSda, RgbPin, RotaryPin};
use arraydeque::ArrayDeque;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicI8, AtomicU8, Ordering};
use embedded_graphics::image::Image;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::iso_8859_1::FONT_5X8;
use embedded_graphics::mono_font::{MonoFont, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::{BinaryColor, Rgb565};
use embedded_graphics::prelude::{DrawTargetExt, Point, Size};
use embedded_graphics::primitives::{Primitive, PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, Text};
use embedded_graphics::Drawable;
use fugit::{ExtU32, MicrosDurationU32, RateExtU32};
use keyberon::layout::Event;
use kyria_kb2040::anim::{AnimKind, AnimationController, SwitchKind};
use kyria_kb2040::cross_talk::{InterProcessCommand, InterProcessResponse};
use kyria_kb2040::layout::{
    CustomAction, CustomEventExt, PongEvent, PressEvent, SerializableEvent,
};
use kyria_kb2040::log;
use kyria_kb2040::pong::{
    PlayState, Pong, ScoreState, StartState, StateMachine, BALL_RADIUS, LEFT_PADDLE_X,
    PADDLE_HEIGHT, PADDLE_WIDTH, RIGHT_PADDLE_X, SCREEN_HEIGHT, SCREEN_WIDTH,
};
use kyria_kb2040::rotary::update_last_rotary;
use rotary_encoder_hal::Rotary;
use rp2040_hal::rosc::{Disabled, Enabled, RingOscillator};
use rp2040_hal::timer::Alarm2;
use smart_leds::brightness;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::{DisplayConfig, I2CInterface};
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::Ssd1306;
use tinybmp::Bmp;
use ws2812_pio::Ws2812Direct;

const LED_ANIM_HZ: u32 = 60; // 3ms
const LED_ANIM_PERIOD_US: u32 = 1_000_000 / LED_ANIM_HZ;
const PONG_HZ: u32 = 27;
const PONG_PERIOD_US: u32 = 1_000_000 / PONG_HZ;
const LED_BRIGHTNESS: u8 = 64;
/// Number of scan cycles to wait before triggering rotary release event.
const ROTARY_WAIT: u8 = 1;

type DisplaySize = DisplaySize128x64;
type Display = Ssd1306<
    I2CInterface<I2C<I2C1, (DisplayI2cSda, DisplayI2cScl)>>,
    DisplaySize,
    BufferedGraphicsMode<DisplaySize>,
>;

static mut SIO_FIFO: Option<SioFifo> = None;
static mut IS_RIGHT: bool = false;
static PONG_MODE: AtomicBool = AtomicBool::new(false);
static mut ROTARY: Option<Rotary<RotaryPin, RotaryPin>> = None;
static mut LAST_ROTARY: Option<(Event, u8)> = None;
static mut ALARM1: Option<Alarm1> = None;
static mut ALARM2: Option<Alarm2> = None;
static mut STATE: Option<&'static AtomicU8> = None;
static mut ANIM_CONTROLLER: Option<AnimationController> = None;
static mut WS: Option<Ws2812Direct<PIO0, SM0, RgbPin>> = None;
static mut DISPLAY: Option<Display> = None;
static mut PONG: Option<Pong> = None;
static mut ROSC: Option<RoscState> = None;
static PONG_LEFT: AtomicI8 = AtomicI8::new(0);
static PONG_RIGHT: AtomicI8 = AtomicI8::new(0);

enum RoscState {
    Enabled(RingOscillator<Enabled>),
    Disabled(RingOscillator<Disabled>),
    Empty,
}

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

    let mut alarm2 = timer.alarm_2().unwrap();
    alarm2.enable_interrupt();

    let rosc = RingOscillator::new(pac.ROSC);

    let Core1Pins {
        rgb,
        rotary1,
        rotary2,
        display_sda,
        display_scl,
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

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c1 = I2C::i2c1(
        pac.I2C1,
        display_sda,
        display_scl,
        400.kHz(),
        &mut pac.RESETS,
        peripheral_freq,
    );

    // Create the I²C display interface:
    let interface = ssd1306::I2CDisplayInterface::new(i2c1);

    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    draw_rust(&mut display);

    // Safety: Interrupts are not enabled, nothing else is accessing these fields.
    unsafe {
        IS_RIGHT = is_right;
        STATE = Some(state);
        ALARM1 = Some(alarm1);
        ALARM2 = Some(alarm2);
        ROTARY = Some(rotary);
        WS = Some(ws);
        ANIM_CONTROLLER = Some(anim_controller);
        SIO_FIFO = Some(sio.fifo);
        DISPLAY = Some(display);
        ROSC = Some(RoscState::Disabled(rosc));
    }

    // Unmask the alarm and fifo interrups so we can process requests from core 0.
    unsafe {
        // Unmask the interrupts so they can finally fire.
        pac::NVIC::unmask(pac::interrupt::TIMER_IRQ_1);
        pac::NVIC::unmask(pac::interrupt::TIMER_IRQ_2);
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

    // Safety: ALARM1 is only accessed here after init.
    let alarm1 = unsafe { ALARM1.as_mut().unwrap() };
    alarm1.clear_interrupt();
    alarm1.schedule(anim_micros).unwrap();

    let anim_millis = anim_micros.to_millis();

    // Safety: STATE, ANIM_CONTROLLER, WS, and LED_STATES are only accessed here
    // after init.
    let (state, anim_controller, ws, led_states, is_right) = unsafe {
        (
            STATE.as_ref().unwrap(),
            ANIM_CONTROLLER.as_mut().unwrap(),
            WS.as_mut().unwrap(),
            &mut LED_STATES,
            IS_RIGHT,
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
            CustomAction::PongMode => {
                cortex_m::interrupt::free(|_| {
                    if !PONG_MODE.load(Ordering::Acquire) {
                        // Safety: ALARM2 is not being accessed by the pong timer irq because
                        // PONG_MODE is false.
                        unsafe {
                            ALARM2
                                .as_mut()
                                .unwrap()
                                .schedule(PONG_PERIOD_US.micros())
                                .unwrap();
                        }
                        PONG_MODE.store(true, Ordering::Release);
                    }
                });
                None
            }
            CustomAction::PongEvent(event) => match event {
                PongEvent::Start => Some(AnimKind::PongStart),
                PongEvent::ScoreLeft => Some(AnimKind::PongScoreLeft),
                PongEvent::ScoreRight => Some(AnimKind::PongScoreRight),
                PongEvent::LeftUp => {
                    cortex_m::interrupt::free(|_| {
                        let left = PONG_LEFT.load(Ordering::Acquire);
                        PONG_LEFT.store(left + 1, Ordering::Release);
                    });
                    None
                }
                PongEvent::LeftDown => {
                    cortex_m::interrupt::free(|_| {
                        let left = PONG_LEFT.load(Ordering::Acquire);
                        PONG_LEFT.store(left - 1, Ordering::Release);
                    });
                    None
                }
                PongEvent::RightUp => {
                    cortex_m::interrupt::free(|_| {
                        let right = PONG_RIGHT.load(Ordering::Acquire);
                        PONG_RIGHT.store(right + 1, Ordering::Release);
                    });
                    None
                }
                PongEvent::RightDown => {
                    cortex_m::interrupt::free(|_| {
                        let right = PONG_RIGHT.load(Ordering::Acquire);
                        PONG_RIGHT.store(right - 1, Ordering::Release);
                    });
                    None
                }
            },
            _ => None,
        } {
            let switch_kind = anim_state.switch_kind();
            match (switch_kind, event.event) {
                (SwitchKind::Momentary, PressEvent::Press) => {
                    led_states.push_back(anim_controller.state()).unwrap();
                    anim_controller.set_state(anim_state);
                }
                (SwitchKind::Momentary, PressEvent::Release) => {
                    if anim_controller.state() == anim_state {
                        if let Some(anim_state) = led_states.pop_back() {
                            anim_controller.set_state(anim_state);
                        }
                    } else if led_states.back() == Some(&anim_state) {
                        led_states.pop_back();
                    } else {
                        led_states.clear();
                    }
                }
                (SwitchKind::Toggle, PressEvent::Press) => {
                    if !event.action.is_pong() {
                        cortex_m::interrupt::free(|_| {
                            if PONG_MODE.load(Ordering::Acquire) {
                                PONG_MODE.store(false, Ordering::Release);
                            }
                        });
                    }
                    anim_controller.set_state(anim_state);
                    led_states.clear();
                }
                (SwitchKind::Toggle, PressEvent::Release) => {}
            }
        }
    }

    if anim_controller.tick(anim_millis) {
        if let AnimKind::Boot = anim_controller.state() {
            anim_controller.force_state(AnimKind::Colemak);
        }
    }

    let leds = anim_controller.leds(is_right);
    SmartLedsWrite::write(ws, brightness(leds.into_iter(), LED_BRIGHTNESS)).unwrap();
}

fn draw_rust(display: &mut Display) {
    let bmp =
        Bmp::from_slice(include_bytes!("./assets/rust.bmp")).expect("Failed to load BMP image");

    // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will
    // read the pixels correctly
    let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(32, 0));

    // We use the `color_converted` method here to automatically convert the RGB565 image data
    // into BinaryColor values.
    im.draw(&mut display.color_converted()).unwrap();

    display.flush().unwrap();
}

/// Safety: Only call after STATE has been initialized.
unsafe fn pong_state(event: PongEvent) {
    // Safety: STATE only mutated in init.
    unsafe { STATE.as_ref() }.unwrap().store(
        SerializableEvent {
            action: CustomAction::PongEvent(event),
            event: PressEvent::Press,
        }
        .serialize(),
        Ordering::Release,
    );
}

#[interrupt]
fn TIMER_IRQ_2() {
    let alarm2 = unsafe { ALARM2.as_mut().unwrap() };
    alarm2.clear_interrupt();
    let display = unsafe { DISPLAY.as_mut().unwrap() };
    let pong = unsafe { &mut PONG };
    let rosc = unsafe { ROSC.as_mut().unwrap() };

    // If pong mode was turned off, then disable the display.
    cortex_m::interrupt::free(|_| {
        if !PONG_MODE.load(Ordering::Relaxed) {
            draw_rust(display);
            *pong = None;
            let RoscState::Enabled(r) = core::mem::replace(rosc, RoscState::Empty) else {
                unreachable!();
            };
            *rosc = RoscState::Disabled(r.disable());
            return;
        }
    });

    // Pong mode is still on, reschedule.
    alarm2.schedule(PONG_PERIOD_US.micros()).unwrap();

    let pong = match pong {
        Some(pong) => pong,
        None => {
            *pong = Some(Pong::new());
            let RoscState::Disabled(r) = core::mem::replace(rosc, RoscState::Empty) else {
                unreachable!();
            };
            *rosc = RoscState::Enabled(r.initialize());
            display.clear_buffer();
            pong.as_mut().unwrap()
        }
    };
    let RoscState::Enabled(rosc) = rosc else {
        unreachable!();
    };
    pong.render(display, true, true);

    let (left, right) = cortex_m::interrupt::free(|_| {
        let left = PONG_LEFT.load(Ordering::Acquire);
        let right = PONG_RIGHT.load(Ordering::Acquire);
        PONG_RIGHT.store(0, Ordering::Release);
        PONG_LEFT.store(0, Ordering::Release);
        (left, right)
    });

    if pong.tick(rosc, left, right) {
        match pong.state() {
            StateMachine::Start(_) => unsafe { pong_state(PongEvent::Start) },
            StateMachine::Score(state) => unsafe {
                pong_state(if state.right_scored() {
                    PongEvent::ScoreRight
                } else {
                    PongEvent::ScoreLeft
                })
            },
            _ => {}
        }
    }

    pong.render(display, false, true);

    display.flush().unwrap();
}

trait Render {
    fn render(&self, display: &mut Display, clear: bool, render_ball: bool);
}

impl Render for Pong {
    fn render(&self, display: &mut Display, clear: bool, _render_ball: bool) {
        match self.state() {
            StateMachine::Start(s) => {
                s.render(display, clear, false);
            }
            StateMachine::Playing(p) => p.render(display, clear, true),
            StateMachine::Score(s) => s.render(display, clear, false),
        }
    }
}

const START_FONT: MonoFont = FONT_6X10;
const SCORE_FONT: MonoFont = FONT_5X8;

impl Render for StartState {
    fn render(&self, display: &mut Display, clear: bool, render_ball: bool) {
        if self.play_state().left_score() + self.play_state().right_score() > 0 {
            self.play_state().render(display, clear, render_ball);
        }
        let text_style = MonoTextStyleBuilder::new()
            .font(&START_FONT)
            .text_color(if clear {
                BinaryColor::Off
            } else {
                BinaryColor::On
            })
            .build();

        let mut buf = [0u8; 32];
        let mut wrapper = kyria_kb2040::fmt::Wrapper::new(&mut buf);
        let _ = write!(&mut wrapper, "Turn encoder\nto start");
        let written = wrapper.written();
        Text::with_alignment(
            core::str::from_utf8(&buf[..written]).unwrap(),
            Point {
                x: SCREEN_WIDTH as i32 / 2,
                y: SCREEN_HEIGHT as i32 / 2,
            },
            text_style,
            Alignment::Center,
        )
        .draw(display)
        .unwrap();
    }
}

impl Render for PlayState {
    fn render(&self, display: &mut Display, clear: bool, render_ball: bool) {
        let color = if clear {
            BinaryColor::Off
        } else {
            BinaryColor::On
        };
        if render_ball {
            let ball_center = self.ball().pos;
            Rectangle::with_center(
                Point {
                    x: ball_center.x as i32,
                    y: ball_center.y as i32,
                },
                Size {
                    width: BALL_RADIUS as u32 * 2,
                    height: BALL_RADIUS as u32 * 2,
                },
            )
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(display)
            .unwrap();
        }
        Rectangle::with_center(
            Point {
                x: LEFT_PADDLE_X as i32,
                y: self.left_paddle().y as i32,
            },
            Size {
                width: PADDLE_WIDTH as u32,
                height: PADDLE_HEIGHT as u32,
            },
        )
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display)
        .unwrap();
        Rectangle::with_center(
            Point {
                x: RIGHT_PADDLE_X as i32,
                y: self.right_paddle().y as i32,
            },
            Size {
                width: PADDLE_WIDTH as u32,
                height: PADDLE_HEIGHT as u32,
            },
        )
        .into_styled(PrimitiveStyle::with_fill(color))
        .draw(display)
        .unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&SCORE_FONT)
            .text_color(color)
            .build();

        const PADDING: i32 = 10;

        let mut buf = [0u8; 32];
        let mut wrapper = kyria_kb2040::fmt::Wrapper::new(&mut buf);
        let _ = write!(&mut wrapper, "{}", self.left_score());
        let written = wrapper.written();
        Text::with_baseline(
            core::str::from_utf8(&buf[..written]).unwrap(),
            Point {
                x: PADDING,
                y: SCORE_FONT.character_size.height as i32 + PADDING,
            },
            text_style,
            Baseline::Bottom,
        )
        .draw(display)
        .unwrap();

        let mut wrapper = kyria_kb2040::fmt::Wrapper::new(&mut buf);
        let _ = write!(&mut wrapper, "{}", self.right_score());
        let written = wrapper.written();
        Text::with_baseline(
            core::str::from_utf8(&buf[..written]).unwrap(),
            Point {
                x: SCREEN_WIDTH as i32
                    - 1
                    - PADDING
                    - (SCORE_FONT.character_size.width as i32 * written as i32
                        + SCORE_FONT.character_spacing as i32 * (written - 1) as i32),
                y: SCORE_FONT.character_size.height as i32 + PADDING,
            },
            text_style,
            Baseline::Bottom,
        )
        .draw(display)
        .unwrap();
    }
}

impl Render for ScoreState {
    fn render(&self, display: &mut Display, clear: bool, _render_ball: bool) {
        let text_style = MonoTextStyleBuilder::new()
            .font(&SCORE_FONT)
            .text_color(if clear {
                BinaryColor::Off
            } else {
                BinaryColor::On
            })
            .build();

        const PADDING: i32 = 10;

        let mut buf = [0u8; 32];
        let mut wrapper = kyria_kb2040::fmt::Wrapper::new(&mut buf);
        let _ = write!(&mut wrapper, "{}", self.left_score());
        let written = wrapper.written();
        Text::with_baseline(
            core::str::from_utf8(&buf[..written]).unwrap(),
            Point {
                x: PADDING,
                y: SCORE_FONT.character_size.height as i32 + PADDING,
            },
            text_style,
            Baseline::Bottom,
        )
        .draw(display)
        .unwrap();

        let mut wrapper = kyria_kb2040::fmt::Wrapper::new(&mut buf);
        let _ = write!(&mut wrapper, "{}", self.right_score());
        let written = wrapper.written();
        Text::with_baseline(
            core::str::from_utf8(&buf[..written]).unwrap(),
            Point {
                x: SCREEN_WIDTH as i32
                    - 1
                    - PADDING
                    - (SCORE_FONT.character_size.width as i32 * written as i32
                        + SCORE_FONT.character_spacing as i32 * (written - 1) as i32),
                y: SCORE_FONT.character_size.height as i32 + PADDING,
            },
            text_style,
            Baseline::Bottom,
        )
        .draw(display)
        .unwrap();
    }
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
