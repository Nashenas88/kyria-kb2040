use druid::widget::prelude::*;
use druid::widget::{Button, Flex, Label, Slider};
use druid::{AppLauncher, Color, Data, Lens, Point, Rect, TimerToken, WidgetExt, WindowDesc};
use kyria_kb2040::anim::{AnimKind, AnimationController};
use kyria_kb2040::layout::CustomAction;
use std::ops::{Index, IndexMut};
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const POOL_SIZE: usize = 10;

const BACKGROUND: Color = Color::grey8(23);

// Default value is BootLed.
static LED_STATE: AtomicU8 = AtomicU8::new(0);
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
        if self.counter >= LED_ANIM_TIME_US / CORE1_LOOP_US as u32 {
            self.counter = 0;

            let state_val = LED_STATE.swap(0, Ordering::AcqRel);
            let custom_action = CustomAction::try_from(state_val).ok();
            if let Some(anim_state) = custom_action.and_then(|custom_action| match custom_action {
                CustomAction::QwertyLed => Some(AnimKind::Qwerty),
                CustomAction::ColemakLed => Some(AnimKind::Colemak),
                CustomAction::LayerSelectLed => Some(AnimKind::LayerSelect),
                CustomAction::SymLed => Some(AnimKind::Sym),
                _ => None,
            }) {
                self.anim_controller.set_state(anim_state);
            }

            if self.anim_controller.tick(diff.as_millis() as u32) {
                if let AnimKind::Boot = self.anim_controller.state() {
                    self.anim_controller.force_state(AnimKind::Colemak);
                }
            }

            let leds = self.anim_controller.leds();
            for (led, color) in leds.into_iter().enumerate() {
                self[led] = Color::rgb8(color.r, color.g, color.b);
            }
        }
    }
}

#[derive(Clone, Lens, Data)]
struct AppData {
    grid: Grid,
    ignore: bool,
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
            Flex::column()
                .with_child(
                    // a row with state buttons
                    Flex::row()
                        .with_flex_child(
                            // Layer button
                            Button::new("Layer")
                                .on_click(|ctx, _data: &mut bool, _: &Env| {
                                    LED_STATE.store(
                                        CustomAction::LayerSelectLed.into(),
                                        Ordering::Release,
                                    );
                                    ctx.request_layout();
                                })
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Colemak button
                            Button::new("Colemak")
                                .on_click(|ctx, _data: &mut bool, _: &Env| {
                                    LED_STATE
                                        .store(CustomAction::ColemakLed.into(), Ordering::Release);
                                    ctx.request_layout();
                                })
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Qwerty button
                            Button::new("Qwerty")
                                .on_click(|ctx, _data: &mut bool, _: &Env| {
                                    LED_STATE
                                        .store(CustomAction::QwertyLed.into(), Ordering::Release);
                                    ctx.request_layout();
                                })
                                .lens(AppData::ignore)
                                .padding((5., 5.)),
                            1.0,
                        )
                        .with_flex_child(
                            // Sym button
                            Button::new("Sym")
                                .on_click(|ctx, _data: &mut bool, _: &Env| {
                                    LED_STATE.store(CustomAction::SymLed.into(), Ordering::Release);
                                    ctx.request_layout();
                                })
                                .lens(AppData::ignore)
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
            width: 800.0,
            height: 800.0,
        })
        .resizable(false)
        .title("Game of Life");
    let grid = Grid::new();

    AppLauncher::with_window(window)
        .log_to_console()
        .launch(AppData {
            grid,
            ignore: true,
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
