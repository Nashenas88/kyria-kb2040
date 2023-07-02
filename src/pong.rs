use rand::RngCore;

pub const BALL_RADIUS: f32 = 1.0;
pub const PADDLE_HEIGHT: f32 = 32.0;
pub const PADDLE_WIDTH: f32 = 2.0;
const LEFT_PADDLE_X: u8 = PADDLE_WIDTH as u8 >> 1;
const RIGHT_PADDLE_X: u8 = SCREEN_WIDTH - LEFT_PADDLE_X;
const SCREEN_HEIGHT: u8 = 64;
const SCREEN_WIDTH: u8 = 128;
// How far from the edge to the angled bounces happen?
const ANGLE_START: f32 = 0.2;
const PADDLE_ANGLE_START: f32 = PADDLE_HEIGHT * ANGLE_START;

#[derive(Clone)]
pub enum StateMachine {
    Start(StartState),
    Playing(PlayState),
    Score(ScoreState),
}

impl<R> Tick<R> for StateMachine
where
    R: RngCore,
{
    fn update(&mut self, rng_core: &mut R, left: i8, right: i8) -> Option<StateMachine> {
        match self {
            StateMachine::Start(s) => s.update(rng_core, left, right),
            StateMachine::Playing(p) => p.update(rng_core, left, right),
            StateMachine::Score(s) => s.update(rng_core, left, right),
        }
    }
}

#[derive(Clone)]
pub struct Pong {
    state: StateMachine,
}

impl Pong {
    pub fn new() -> Self {
        Self {
            state: StateMachine::Start(StartState(PlayState::new())),
        }
    }

    pub fn state(&self) -> &StateMachine {
        &self.state
    }
}

impl Pong {
    pub fn tick<R>(&mut self, rng_core: &mut R, left: i8, right: i8)
    where
        R: RngCore,
    {
        if let Some(state) = self.state.update(rng_core, left, right) {
            self.state = state;
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Paddle<const X: u8> {
    pub y: f32,
}

impl<const X: u8> Paddle<X> {
    fn new() -> Self {
        Self {
            y: SCREEN_HEIGHT as f32 / 2.0,
        }
    }

    fn r#move(&mut self, dy: f32) {
        self.y = (self.y + dy).max(0.0).min(SCREEN_HEIGHT as f32 - 1.0);
    }

    pub fn left(&self) -> f32 {
        X as f32 - PADDLE_WIDTH / 2.0
    }

    pub fn right(&self) -> f32 {
        X as f32 + PADDLE_WIDTH / 2.0
    }

    pub fn top(&self) -> f32 {
        self.y - PADDLE_HEIGHT / 2.0
    }

    pub fn bottom(&self) -> f32 {
        self.y + PADDLE_HEIGHT / 2.0
    }
}

pub trait Tick<R>
where
    R: RngCore,
{
    fn update(&mut self, rng_core: &mut R, left: i8, right: i8) -> Option<StateMachine>;
}

#[derive(Copy, Clone)]
pub struct StartState(PlayState);

impl<R> Tick<R> for StartState
where
    R: RngCore,
{
    fn update(&mut self, rng_core: &mut R, left: i8, right: i8) -> Option<StateMachine> {
        if left != 0 || right != 0 {
            self.0.start(rng_core);
            Some(StateMachine::Playing(self.0))
        } else {
            None
        }
    }
}

#[derive(Clone, Copy)]
pub struct Vector {
    pub x: f32,
    pub y: f32,
}

#[derive(Clone, Copy)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

#[derive(Clone, Copy)]
pub struct Ball {
    pub pos: Point,
    pub vel: Vector,
}

impl Ball {
    fn new(x: f32, y: f32) -> Self {
        Self {
            pos: Point { x, y },
            vel: Vector { x: 0.0, y: 0.0 },
        }
    }

    fn set_velocity(&mut self, vel: Vector) {
        self.vel = vel;
    }

    fn r#move(&mut self) {
        self.pos.x = self.pos.x + self.vel.x;
        self.pos.y = self.pos.y + self.vel.y;
    }

    fn collide_at_angle<const X: u8>(&mut self, paddle: Paddle<X>) {
        if self.top() < paddle.top() + PADDLE_ANGLE_START {
            self.vel.y += (self.top() - paddle.top()) / PADDLE_ANGLE_START;
        } else if self.bottom() > paddle.bottom() - PADDLE_ANGLE_START {
            self.vel.y -= (self.bottom() - paddle.bottom()) / PADDLE_ANGLE_START;
        }
    }

    fn collide(
        &mut self,
        left_paddle: Paddle<LEFT_PADDLE_X>,
        right_paddle: Paddle<RIGHT_PADDLE_X>,
    ) {
        if self.left() < left_paddle.right()
            && self.bottom() > left_paddle.top()
            && self.top() < left_paddle.bottom()
        {
            self.collide_at_angle(left_paddle);
            self.vel.x *= -1.0;
        } else if self.right() > right_paddle.left()
            && self.bottom() > right_paddle.top()
            && self.top() < right_paddle.bottom()
        {
            self.collide_at_angle(right_paddle);
            self.vel.x *= -1.0;
        }

        if self.bottom() < 0.0 || self.top() > SCREEN_HEIGHT as f32 {
            self.vel.y *= -1.0;
        }
    }

    pub fn left(&self) -> f32 {
        self.pos.x as f32 - BALL_RADIUS
    }

    pub fn right(&self) -> f32 {
        self.pos.x as f32 + BALL_RADIUS
    }

    pub fn top(&self) -> f32 {
        self.pos.y as f32 - BALL_RADIUS
    }

    pub fn bottom(&self) -> f32 {
        self.pos.y as f32 + BALL_RADIUS
    }
}

#[derive(Copy, Clone)]

pub struct PlayState {
    left_score: u8,
    right_score: u8,
    ball: Ball,
    left_paddle: Paddle<LEFT_PADDLE_X>,
    right_paddle: Paddle<RIGHT_PADDLE_X>,
}

impl PlayState {
    pub fn new() -> Self {
        Self::from_scores(0, 0)
    }

    pub fn from_scores(left_score: u8, right_score: u8) -> Self {
        Self {
            left_score,
            right_score,
            ball: Ball::new(SCREEN_WIDTH as f32 / 2.0, SCREEN_HEIGHT as f32 / 2.0),
            left_paddle: Paddle::new(),
            right_paddle: Paddle::new(),
        }
    }

    pub fn left_score(&self) -> u8 {
        self.left_score
    }

    pub fn right_score(&self) -> u8 {
        self.right_score
    }

    pub fn ball(&self) -> Ball {
        self.ball
    }

    pub fn left_paddle(&self) -> Paddle<LEFT_PADDLE_X> {
        self.left_paddle
    }

    pub fn right_paddle(&self) -> Paddle<RIGHT_PADDLE_X> {
        self.right_paddle
    }

    pub fn start(&mut self, r: &mut impl RngCore) {
        let x = if rand_bool(r) { 1.0 } else { -1.0 };
        let y = rand(r, -0.7, 0.7);
        self.ball.set_velocity(Vector { x, y });
    }
}

fn rand_bool(r: &mut impl RngCore) -> bool {
    let mut x: u8 = 0;
    r.fill_bytes(core::slice::from_mut(&mut x));
    (x & 1) == 0
}

fn rand(r: &mut impl RngCore, min: f32, max: f32) -> f32 {
    let x = r.next_u32();
    (x as f32 / u32::MAX as f32) * (max - min) + min
}

fn abs(f: f32) -> f32 {
    if f < 0.0 {
        f * -1.0
    } else {
        f
    }
}

impl<R> Tick<R> for PlayState
where
    R: RngCore,
{
    fn update(&mut self, _: &mut R, left: i8, right: i8) -> Option<StateMachine> {
        self.left_paddle.r#move(left as f32);
        self.right_paddle.r#move(right as f32);
        self.ball.r#move();

        if abs(self.ball.pos.x) < core::f32::EPSILON {
            self.left_score += 1;
            Some(StateMachine::Score(ScoreState::new(
                self.left_score,
                self.right_score,
            )))
        } else if self.ball.pos.x == SCREEN_WIDTH as f32 - 1.0 {
            self.right_score += 1;
            Some(StateMachine::Score(ScoreState::new(
                self.left_score,
                self.right_score,
            )))
        } else {
            self.ball.collide(self.left_paddle, self.right_paddle);
            None
        }
    }
}

const COUNT_DOWN_S: u8 = 3;

#[derive(Copy, Clone)]
pub struct ScoreState {
    left_score: u8,
    right_score: u8,
    count_down: u8,
}

impl ScoreState {
    fn new(left: u8, right: u8) -> Self {
        Self {
            left_score: left,
            right_score: right,
            count_down: COUNT_DOWN_S,
        }
    }
}

impl<R> Tick<R> for ScoreState
where
    R: RngCore,
{
    fn update(&mut self, _r: &mut R, _left: i8, _right: i8) -> Option<StateMachine> {
        self.count_down = self.count_down.saturating_sub(1);
        if self.count_down == 0 {
            Some(StateMachine::Start(StartState(PlayState::from_scores(
                self.left_score,
                self.right_score,
            ))))
        } else {
            None
        }
    }
}
