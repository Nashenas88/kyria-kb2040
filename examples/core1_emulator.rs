use arraydeque::ArrayDeque;
use druid::keyboard_types::{Key, KeyState};
use druid::widget::prelude::*;
use druid::widget::{Button, Checkbox, Flex, Label, Slider};
use druid::{
    AppLauncher, Color, Data, KeyEvent, Lens, Point, Rect, TimerToken, WidgetExt, WindowDesc,
};
use kyria_kb2040::anim::{AnimKind, AnimationController, SwitchKind};
use kyria_kb2040::layout::{CustomAction, CustomEventExt, PressEvent, SerializableEvent};
use kyria_kb2040::pong::{
    Ball, Paddle, Pong, StateMachine, BALL_RADIUS, PADDLE_HEIGHT, PADDLE_WIDTH,
};
use std::ops::{Index, IndexMut};
use std::sync::atomic::{AtomicI8, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const POOL_SIZE: usize = 10;

const BACKGROUND: Color = Color::grey8(23);

// Default value is BootLed.
static LED_STATE: AtomicU8 = AtomicU8::new(0);
static LEFT: AtomicI8 = AtomicI8::new(0);
static RIGHT: AtomicI8 = AtomicI8::new(0);
const CORE1_LOOP_US: u64 = 1000; // 250us
const LED_ANIM_TIME_US: u32 = 3_000;

#[allow(clippy::rc_buffer)]
#[derive(Clone, Data, PartialEq)]
struct Grid {
    storage: Arc<Vec<Color>>,
    last: Instant,
    counter: u32,
    #[data(ignore)]
    anim_controller: AnimationController,
}

static mut LED_STATES: ArrayDeque<AnimKind, 2> = ArrayDeque::new();

impl Grid {
    pub fn new() -> Grid {
        Grid {
            storage: Arc::new(vec![Color::BLACK; POOL_SIZE]),
            last: Instant::now(),
            counter: 0,
            anim_controller: AnimationController::new(),
        }
    }

    pub fn evolve(&mut self) {
        let now = Instant::now();
        let diff = now - self.last;
        self.last = now;
        std::thread::sleep(Duration::from_micros(CORE1_LOOP_US).saturating_sub(diff));
        self.counter += 1;

        let led_states = unsafe { &mut LED_STATES };

        if self.counter >= LED_ANIM_TIME_US / CORE1_LOOP_US as u32 {
            self.counter = 0;

            let state_val = LED_STATE.swap(0, Ordering::AcqRel);
            if let Some(event) = SerializableEvent::deserialize(state_val) {
                if let Some(anim_state) = match event.action {
                    CustomAction::QwertyLed => Some(AnimKind::Qwerty),
                    CustomAction::ColemakLed => Some(AnimKind::Colemak),
                    CustomAction::LayerSelectLed => Some(AnimKind::LayerSelect),
                    CustomAction::SymLed => Some(AnimKind::Sym),
                    CustomAction::FunctionLed => Some(AnimKind::Function),
                    CustomAction::RainbowLed => Some(AnimKind::Rainbow),
                    CustomAction::Core1Error(e) => Some(AnimKind::Error(e)),
                    _ => None,
                } {
                    if let (SwitchKind::Momentary, PressEvent::Press) =
                        (anim_state.switch_kind(), event.event)
                    {
                        let kind = self.anim_controller.state();
                        tracing::debug!("pushing {kind:?}");
                        led_states.push_back(kind).unwrap();
                        self.anim_controller.set_state(anim_state);
                    } else if let (SwitchKind::Momentary, PressEvent::Release) =
                        (anim_state.switch_kind(), event.event)
                    {
                        if self.anim_controller.state() == anim_state {
                            if let Some(anim_state) = led_states.pop_back() {
                                self.anim_controller.set_state(anim_state);
                            }
                        } else if led_states.back() == Some(&anim_state) {
                            led_states.pop_back();
                        } else {
                            led_states.clear();
                        }
                    } else if let PressEvent::Press = event.event {
                        self.anim_controller.set_state(anim_state);
                        led_states.clear();
                    }
                }
            }

            if self.anim_controller.tick(diff.as_millis() as u32) {
                if let AnimKind::Boot = self.anim_controller.state() {
                    self.anim_controller.force_state(AnimKind::Colemak);
                }
            }

            let leds = self.anim_controller.leds(true);
            for (led, color) in leds.into_iter().enumerate() {
                self[led] = Color::rgb8(color.r, color.g, color.b);
            }
        }
    }
}

#[derive(Clone, Lens, Data)]
struct AppData {
    grid: Grid,
    pong: PongManager,
    ignore: bool,
    layer: bool,
    sym: bool,
    updates_per_second: f64,
}

impl AppData {
    // allows time interval in the range [100ms, 5000ms]
    // equivalently, 0.2 ~ 20ups
    pub fn iter_interval(&self) -> u64 {
        (1000. / self.updates_per_second) as u64
    }
}

struct LedStripWidget {
    timer_id: TimerToken,
    cell_size: Size,
    last_update: Instant,
}

impl Widget<AppData> for LedStripWidget {
    fn event(&mut self, ctx: &mut EventCtx, event: &Event, data: &mut AppData, _env: &Env) {
        match event {
            Event::WindowConnected => {
                ctx.request_paint();
                let deadline = Duration::from_millis(data.iter_interval());
                self.last_update = Instant::now();
                self.timer_id = ctx.request_timer(deadline);
            }
            Event::Timer(id) => {
                if *id == self.timer_id {
                    data.grid.evolve();
                    ctx.request_paint();
                    let deadline = Duration::from_millis(data.iter_interval());
                    self.last_update = Instant::now();
                    self.timer_id = ctx.request_timer(deadline);
                }
            }
            _ => {}
        }
    }

    fn lifecycle(
        &mut self,
        _ctx: &mut LifeCycleCtx,
        _event: &LifeCycle,
        _data: &AppData,
        _env: &Env,
    ) {
    }

    fn update(&mut self, ctx: &mut UpdateCtx, old_data: &AppData, data: &AppData, _env: &Env) {
        if (data.updates_per_second - old_data.updates_per_second).abs() > 0.001 {
            let deadline = Duration::from_millis(data.iter_interval())
                .checked_sub(Instant::now().duration_since(self.last_update))
                .unwrap_or_else(|| Duration::from_secs(0));
            self.timer_id = ctx.request_timer(deadline);
        }
        if data.grid != old_data.grid {
            ctx.request_paint();
        }
    }

    fn layout(
        &mut self,
        _layout_ctx: &mut LayoutCtx,
        bc: &BoxConstraints,
        _data: &AppData,
        _env: &Env,
    ) -> Size {
        let max_size = bc.max();
        let min_side = max_size.height.min(max_size.width);
        Size {
            width: min_side,
            height: min_side,
        }
    }

    fn paint(&mut self, ctx: &mut PaintCtx, data: &AppData, _env: &Env) {
        let size: Size = ctx.size();
        let w0 = size.width / POOL_SIZE as f64;
        let h0 = size.height as f64;
        let cell_size = Size {
            width: w0,
            height: w0,
        };
        self.cell_size = cell_size;
        for led in 0..POOL_SIZE {
            let color = data.grid[led];
            let point = Point {
                x: w0 * led as f64,
                y: h0 / 2.0,
            };
            let rect = Rect::from_origin_size(point, cell_size);
            ctx.fill(rect, &color);
        }
    }
}

#[derive(Clone, Data)]
struct PongManager {
    #[data(ignore)]
    pong: Pong,
    last: Instant,
    counter: u32,
    #[data(ignore)]
    events: Vec<KeyEvent>,
    keys: u8,
}

trait Obj {
    fn center(&self) -> Point;
    fn size(&self) -> Size;
}

impl Obj for Ball {
    fn center(&self) -> Point {
        Point {
            x: self.pos.x as f64,
            y: self.pos.y as f64,
        }
    }

    fn size(&self) -> Size {
        Size {
            width: BALL_RADIUS as f64 * 2.0,
            height: BALL_RADIUS as f64 * 2.0,
        }
    }
}

impl<const X: u8> Obj for Paddle<X> {
    fn center(&self) -> Point {
        Point {
            x: X as f64,
            y: self.y as f64,
        }
    }

    fn size(&self) -> Size {
        Size {
            width: PADDLE_WIDTH as f64,
            height: PADDLE_HEIGHT as f64,
        }
    }
}

impl PongManager {
    fn new() -> Self {
        Self {
            pong: Pong::new(),
            last: Instant::now(),
            counter: 0,
            events: vec![],
            keys: 0,
        }
    }

    pub fn evolve(&mut self) {
        let now = Instant::now();
        let diff = now - self.last;
        self.last = now;
        std::thread::sleep(Duration::from_micros(CORE1_LOOP_US).saturating_sub(diff));
        self.counter += 1;
        const PADDLE_SPEED: i8 = 3;
        const W: u8 = 0b1000_0000;
        const S: u8 = 0b0100_0000;
        const I: u8 = 0b0010_0000;
        const K: u8 = 0b0001_0000;

        if self.counter >= LED_ANIM_TIME_US / CORE1_LOOP_US as u32 {
            self.counter = 0;
            for event in self.events.drain(..) {
                match (event.state, event.key) {
                    (KeyState::Down, Key::Character(w)) if w == "w" => self.keys |= W,
                    (KeyState::Down, Key::Character(s)) if s == "s" => self.keys |= S,
                    (KeyState::Down, Key::Character(i)) if i == "i" => self.keys |= I,
                    (KeyState::Down, Key::Character(k)) if k == "k" => self.keys |= K,
                    (KeyState::Up, Key::Character(w)) if w == "w" => self.keys &= !W,
                    (KeyState::Up, Key::Character(s)) if s == "s" => self.keys &= !S,
                    (KeyState::Up, Key::Character(i)) if i == "i" => self.keys &= !I,
                    (KeyState::Up, Key::Character(k)) if k == "k" => self.keys &= !K,
                    _ => {}
                };
            }

            let mut left = LEFT.swap(0, Ordering::Relaxed);
            let mut right = RIGHT.swap(0, Ordering::Relaxed);
            if left == 0 {
                left = ((self.keys & W) > 0) as i8 * PADDLE_SPEED
                    + ((self.keys & S) > 0) as i8 * -PADDLE_SPEED;
            }
            if right == 0 {
                right = ((self.keys & I) > 0) as i8 * PADDLE_SPEED
                    + ((self.keys & K) > 0) as i8 * -PADDLE_SPEED;
            }
            self.pong.tick(&mut rand::thread_rng(), left, right);
        }
    }

    pub fn render(&self, mut render_rect: impl FnMut(Box<dyn Obj>)) {
        match self.pong.state() {
            StateMachine::Start(_) => {}
            StateMachine::Playing(play_state) => {
                render_rect(Box::new(play_state.ball()) as Box<dyn Obj>);
                render_rect(Box::new(play_state.left_paddle()) as Box<dyn Obj>);
                render_rect(Box::new(play_state.right_paddle()) as Box<dyn Obj>);
            }
            StateMachine::Score(_) => {}
        }
    }
}

struct DisplayWidget {
    timer_id: TimerToken,
    last_update: Instant,
}

impl Widget<AppData> for DisplayWidget {
    fn event(&mut self, ctx: &mut EventCtx, event: &Event, data: &mut AppData, _env: &Env) {
        match event {
            Event::WindowConnected => {
                ctx.request_paint();
                let deadline = Duration::from_millis(data.iter_interval());
                self.last_update = Instant::now();
                self.timer_id = ctx.request_timer(deadline);
            }
            Event::Timer(id) => {
                if *id == self.timer_id {
                    data.pong.evolve();
                    ctx.request_paint();
                    let deadline = Duration::from_millis(data.iter_interval());
                    self.last_update = Instant::now();
                    self.timer_id = ctx.request_timer(deadline);
                }
            }
            Event::KeyDown(event) | Event::KeyUp(event) => {
                tracing::debug!("Got event {event:?}");
                data.pong.events.push(event.clone());
            }
            _ => {}
        }
    }

    fn lifecycle(
        &mut self,
        _ctx: &mut LifeCycleCtx,
        _event: &LifeCycle,
        _data: &AppData,
        _env: &Env,
    ) {
    }

    fn update(&mut self, ctx: &mut UpdateCtx, old_data: &AppData, data: &AppData, _env: &Env) {
        if (data.updates_per_second - old_data.updates_per_second).abs() > 0.001 {
            let deadline = Duration::from_millis(data.iter_interval())
                .checked_sub(Instant::now().duration_since(self.last_update))
                .unwrap_or_else(|| Duration::from_secs(0));
            self.timer_id = ctx.request_timer(deadline);
        }
        if data.grid != old_data.grid {
            ctx.request_paint();
        }
    }

    fn layout(
        &mut self,
        _layout_ctx: &mut LayoutCtx,
        bc: &BoxConstraints,
        _data: &AppData,
        _env: &Env,
    ) -> Size {
        let mut size = bc.max();
        size.height /= 4.0;
        if size.width > size.height * 2.0 {
            size.width = size.height * 2.0;
        } else {
            size.height = size.width / 2.0;
        }
        Size {
            width: size.width,
            height: size.height,
        }
    }

    fn paint(&mut self, ctx: &mut PaintCtx, data: &AppData, _env: &Env) {
        let size: Size = ctx.size();
        // Clear screen in black
        ctx.fill(Rect::from_origin_size(Point::ORIGIN, size), &Color::BLACK);
        // Render each object in white
        let scale = size.width / 128.0;
        data.pong.render(|obj: Box<dyn Obj>| {
            let center = Point {
                x: obj.center().x * scale,
                y: obj.center().y * scale,
            };
            let size = Size {
                width: obj.size().width * scale,
                height: obj.size().height * scale,
            };
            let rect = Rect::from_center_size(center.expand(), size.expand());
            ctx.fill(rect, &Color::WHITE);
        });
    }
}

// struct DisplayEventController<F: Fn(&Event) -> bool> {
//     filter: F,
// }

// impl<W: Widget<AppData>, F: Fn(&Event) -> bool> Controller<AppData, W>
//     for DisplayEventController<F>
// {
//     fn event(
//         &mut self,
//         child: &mut W,
//         ctx: &mut EventCtx,
//         event: &Event,
//         data: &mut AppData,
//         env: &Env,
//     ) {
//         // Every time this controller receives an event we check `f()`.
//         // If `f()` returns true it means that we can add it to the log,
//         // if not then we can skip it.
//         if (self.filter)(event) {
//             println!("Got {event:?}");
//             match event {
//                 Event::KeyDown(event) | Event::KeyUp(event) => {
//                     Arc::make_mut(&mut data.pong.events).push(event.clone());
//                 }
//                 _ => unreachable!("Other event kinds are filtered out"),
//             }
//         }
//         // Always pass on the event!
//         child.event(ctx, event, data, env)
//     }
// }

fn handle_check(action: CustomAction) -> impl Fn(&mut EventCtx, &mut bool, &Env) + 'static {
    move |ctx, data: &mut bool, _: &Env| {
        let event = if !*data {
            tracing::debug!("Data is {data:?}, event is press");
            PressEvent::Press
        } else {
            tracing::debug!("Data is {data:?}, event is release");
            PressEvent::Release
        };
        *data = !*data;

        LED_STATE.store(
            SerializableEvent { action, event }.serialize(),
            Ordering::Release,
        );
        ctx.request_layout();
    }
}

fn handle_click(action: CustomAction) -> impl Fn(&mut EventCtx, &mut bool, &Env) + 'static {
    move |ctx, _data: &mut bool, _: &Env| {
        LED_STATE.store(
            SerializableEvent {
                action,
                event: PressEvent::Press,
            }
            .serialize(),
            Ordering::Release,
        );
        ctx.request_layout();
    }
}

fn handle_stick(
    stick: &'static AtomicI8,
    dir: i8,
) -> impl Fn(&mut EventCtx, &mut bool, &Env) + 'static {
    move |ctx, _data: &mut bool, _: &Env| {
        stick.fetch_add(dir, Ordering::Relaxed);
        ctx.request_layout();
    }
}

fn make_widget() -> impl Widget<AppData> {
    Flex::column()
        .with_flex_child(
            LedStripWidget {
                timer_id: TimerToken::INVALID,
                cell_size: Size {
                    width: 10.0,
                    height: 10.0,
                },
                last_update: Instant::now(),
            },
            1.0,
        )
        .with_child(
            DisplayWidget {
                timer_id: TimerToken::INVALID,
                last_update: Instant::now(),
            }, // .controller(DisplayEventController {
               //     filter: |event| matches!(event, Event::KeyDown(_) | Event::KeyUp(_)),
               // }),
        )
        .with_child(
            Flex::column()
                .with_child(
                    // a row with state buttons
                    Flex::row()
                        .with_flex_child(
                            // Layer button
                            Checkbox::new("Layer")
                                .on_click(handle_check(CustomAction::LayerSelectLed))
                                .lens(AppData::layer)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Colemak button
                            Button::new("Colemak")
                                .on_click(handle_click(CustomAction::ColemakLed))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Qwerty button
                            Button::new("Qwerty")
                                .on_click(handle_click(CustomAction::QwertyLed))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Sym button
                            Checkbox::new("Sym")
                                .on_click(handle_check(CustomAction::SymLed))
                                .lens(AppData::sym)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Function button
                            Button::new("Function")
                                .on_click(handle_click(CustomAction::FunctionLed))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Left Up button
                            Button::new("Left Up")
                                .on_click(handle_stick(&LEFT, 1))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Left Down button
                            Button::new("Left Down")
                                .on_click(handle_stick(&LEFT, -1))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Right Up button
                            Button::new("Right Up")
                                .on_click(handle_stick(&RIGHT, 1))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Right Down button
                            Button::new("Right Down")
                                .on_click(handle_stick(&RIGHT, -1))
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .padding(8.0),
                )
                .with_child(
                    // a row with error
                    Flex::row()
                        .with_flex_child(
                            // Error 1 button
                            Button::new("Error 1")
                                .on_click(handle_click(CustomAction::Core1Error(1)))
                                .lens(AppData::layer)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Error 2 button
                            Button::new("Error 2")
                                .on_click(handle_click(CustomAction::Core1Error(2)))
                                .lens(AppData::layer)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Error 3 button
                            Button::new("Error 3")
                                .on_click(handle_click(CustomAction::Core1Error(3)))
                                .lens(AppData::layer)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Error 4 button
                            Button::new("Error 4")
                                .on_click(handle_click(CustomAction::Core1Error(4)))
                                .lens(AppData::layer)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .padding(8.0),
                )
                .with_child(
                    Flex::row()
                        .with_child(
                            Label::new(|data: &AppData, _env: &_| {
                                format!("{:.2}updates/s", data.updates_per_second)
                            })
                            .padding(3.0),
                        )
                        .with_flex_child(
                            Slider::new()
                                .with_range(0.2, 200.0)
                                .expand_width()
                                .lens(AppData::updates_per_second),
                            1.,
                        )
                        .padding(8.0),
                )
                .background(BACKGROUND),
        )
}

pub fn main() {
    let window = WindowDesc::new(make_widget())
        .window_size(Size {
            width: 960.0,
            height: 800.0,
        })
        .resizable(true)
        .title("Game of Life");
    let grid = Grid::new();
    let pong = PongManager::new();

    AppLauncher::with_window(window)
        .log_to_console()
        .launch(AppData {
            grid,
            pong,
            ignore: true,
            layer: false,
            sym: false,
            updates_per_second: 200.0,
        })
        .expect("launch failed");
}

impl Index<usize> for Grid {
    type Output = Color;
    fn index(&self, idx: usize) -> &Self::Output {
        &self.storage[idx]
    }
}

impl IndexMut<usize> for Grid {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        Arc::make_mut(&mut self.storage).index_mut(idx)
    }
}
