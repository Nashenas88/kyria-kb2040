//! A modified version of ws2812-pio's Ws2812 struct. This one does not take
//! a [Timer] to construct a [Ws2812], but instead requests a [Timer] when
//! building its [Writer] struct. This minimizes the borrow on the Timer struct,
//! allowing it to be used in an rtic app.

#![allow(dead_code)]

use adafruit_kb2040 as bsp;
use bsp::hal::gpio::{Function, FunctionConfig, Pin, PinId, ValidPinMode};
use bsp::hal::pio::{PIOExt, StateMachineIndex, Tx, UninitStateMachine, PIO};
use embedded_time::fixed_point::FixedPoint;
use smart_leds::{SmartLedsWrite, RGB8};

pub struct Ws2812<P, SM, I>
where
    I: PinId,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    tx: Tx<(P, SM)>,
    _pin: Pin<I, Function<P>>,
}

impl<P, SM, I> Ws2812<P, SM, I>
where
    I: PinId,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    /// Creates a new instance of this driver.
    pub fn new(
        pin: Pin<I, Function<P>>,
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        clock_freq: embedded_time::rate::Hertz,
    ) -> Ws2812<P, SM, I> {
        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;
        const FREQ: u32 = 800_000;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 2, 0);
        a.bind(&mut wrap_source);
        let program = a.assemble_with_wrap(wrap_source, wrap_target);

        // Install the program into PIO instruction memory.
        let installed = pio.install(&program).unwrap();

        // Configure the PIO state machine.
        let div = clock_freq.integer() as f32 / (FREQ as f32 * CYCLES_PER_BIT as f32);

        let (mut sm, _, tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
            // only use TX FIFO
            .buffers(bsp::hal::pio::Buffers::OnlyTx)
            // Pin configuration
            .side_set_pin_base(I::DYN.num)
            // OSR config
            .out_shift_direction(bsp::hal::pio::ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(24)
            .clock_divisor(div)
            .build(sm);

        // Prepare pin's direction.
        sm.set_pindirs([(I::DYN.num, bsp::hal::pio::PinDir::Output)]);

        sm.start();

        Self { tx, _pin: pin }
    }
}

impl<'a, P, SM, I> SmartLedsWrite for Ws2812<P, SM, I>
where
    I: PinId,
    P: PIOExt + FunctionConfig,
    Function<P>: ValidPinMode<I>,
    SM: StateMachineIndex,
{
    type Error = ();
    type Color = RGB8;

    // Can only be called once every 60us!
    fn write<T, J>(&mut self, iterator: T) -> Result<(), ()>
    where
        T: Iterator<Item = J>,
        J: Into<RGB8>,
    {
        for item in iterator {
            let color: RGB8 = item.into();
            let word =
                (u32::from(color.g) << 24) | (u32::from(color.r) << 16) | (u32::from(color.b) << 8);

            while !self.tx.write(word) {
                cortex_m::asm::nop();
            }
        }
        Ok(())
    }
}
