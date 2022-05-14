//! Keyboard firmware for Kyria split keyboard on an Adafruit kb2040.

#![no_std]
#![no_main]

mod anim;
mod fmt;
mod layout;
mod leds;
mod matrix;
mod mouse;
mod ws2812;

use crate::anim::{AnimState, AnimationController};
use crate::fmt::Wrapper;
use crate::layout::{CustomAction, NCOLS, NLAYERS, NROWS};
use crate::leds::UsualLeds;
use crate::mouse::MouseReportExt;
use crate::ws2812::Ws2812;
#[cfg(feature = "kb2040")]
use adafruit_kb2040 as bsp;
use arraydeque::{behavior, ArrayDeque};
use bsp::hal::clocks::init_clocks_and_plls;
#[cfg(feature = "kb2040")]
use bsp::hal::gpio::bank0::{
    Gpio0, Gpio1, Gpio11, Gpio12, Gpio13, Gpio2, Gpio25, Gpio28, Gpio29, Gpio3,
};
#[cfg(feature = "pico")]
use bsp::hal::gpio::bank0::{
    Gpio15, Gpio16, Gpio18, Gpio19, Gpio20, Gpio21, Gpio22, Gpio25, Gpio27, Gpio28,
};
use bsp::hal::gpio::{
    DynPin, FunctionI2C, Interrupt, Pin, PullDownDisabled, PullUpInput, PushPullOutput,
};
use bsp::hal::i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator};
use bsp::hal::multicore::{Multicore, Stack};
use bsp::hal::pio::PIOExt;
use bsp::hal::sio::SioFifo;
use bsp::hal::timer::Timer;
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::{Clock, Sio, I2C};
use bsp::pac::{I2C0, I2C1};
use bsp::XOSC_CRYSTAL_FREQ;
use core::fmt::Write;
#[cfg(feature = "pico")]
use defmt_rtt as _;
use embedded_graphics::draw_target::DrawTargetExt;
use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::Point;
use embedded_graphics::Drawable;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::prelude::*;
use embedded_time::duration::Extensions as _;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::{Extensions as _, Hertz};
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::PressedKeys;
#[cfg(feature = "kb2040")]
use panic_halt as _;
#[cfg(feature = "pico")]
use panic_probe as _;
use pimoroni_trackball::{
    I2CInterface as TrackballInterface, Interrupt as _, TrackballBuilder, TrackballData,
};
use pimoroni_trackball_driver as pimoroni_trackball;
use rotary_encoder_hal::{Direction, Rotary};
#[cfg(feature = "pico")]
use rp_pico as bsp;
use smart_leds::{brightness, gamma};
use ssd1306::mode::{DisplayConfig, TerminalMode};
use ssd1306::prelude::I2CInterface;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::Ssd1306;
use tinybmp::Bmp;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

#[cfg(feature = "kb2040")]
type Trackball = pimoroni_trackball::Trackball<
    I2C<I2C0, (Pin<Gpio12, FunctionI2C>, Pin<Gpio1, FunctionI2C>)>,
    Pin<Gpio13, PullUpInput>,
>;
#[cfg(feature = "pico")]
type Trackball = pimoroni_trackball::Trackball<
    I2C<I2C0, (Pin<Gpio20, FunctionI2C>, Pin<Gpio21, FunctionI2C>)>,
    Pin<Gpio22, PullUpInput>,
>;

#[cfg(feature = "kb2040")]
type Display = Ssd1306<
    I2CInterface<I2C<I2C1, (Pin<Gpio2, FunctionI2C>, Pin<Gpio3, FunctionI2C>)>>,
    DisplaySize128x64,
    TerminalMode,
>;
#[cfg(feature = "pico")]
type Display = Ssd1306<
    I2CInterface<I2C<I2C1, (Pin<Gpio18, FunctionI2C>, Pin<Gpio19, FunctionI2C>)>>,
    DisplaySize128x64,
    TerminalMode,
>;
type RotaryEncoder = Rotary<DynPin, DynPin>;
#[cfg(feature = "kb2040")]
type I2CPeripheral =
    I2CPeripheralEventIterator<I2C0, (Pin<Gpio12, FunctionI2C>, Pin<Gpio1, FunctionI2C>)>;
#[cfg(feature = "pico")]
type I2CPeripheral =
    I2CPeripheralEventIterator<I2C0, (Pin<Gpio20, FunctionI2C>, Pin<Gpio21, FunctionI2C>)>;
type I2CController = Trackball;
type ScannedKeys = ArrayDeque<[u8; 16], behavior::Wrapping>;

#[cfg(feature = "kb2040")]
type BootButton = Pin<Gpio11, PullUpInput>;
#[cfg(feature = "pico")]
type BootButton = Pin<Gpio28, PullUpInput>;

// #[cfg(feature = "kb2040")]
// mod defmt {
//     #[macro_export]
//     macro_rules! unreachable {
//         () => {
//             core::unreachable!()
//         };
//     }
//     #[macro_export]
//     macro_rules! info {
//         ($($tt:tt)*) => {};
//     }
// }

const I2C_PERIPHERAL_ADDR: u8 = 0x56;

/// The amount of time between each scan of the matrix for switch presses.
/// This is always the time from the *start* of the previous scan.
const SCAN_TIME_US: u32 = 1000;
const LED_ANIM_TIME_US: u32 = 3_000;

pub enum Either<T, U> {
    Left(T),
    Right(U),
}

struct DropGuard<T>
where
    T: FnMut(),
{
    drop_func: T,
}

impl<T> DropGuard<T>
where
    T: FnMut(),
{
    fn new(drop_func: T) -> Self {
        Self { drop_func }
    }
}

impl<T> Drop for DropGuard<T>
where
    T: FnMut(),
{
    fn drop(&mut self) {
        (self.drop_func)()
    }
}

#[cfg(feature = "kb2040")]
fn core1_pins(pins: bsp::Pins) -> Pin<Gpio0, PullDownDisabled> {
    pins.tx
}

#[cfg(feature = "pico")]
fn core1_pins(pins: bsp::Pins) -> Pin<Gpio27, PullDownDisabled> {
    pins.gpio27
}

static mut CORE1_STACK: Stack<4096> = Stack::new();
const CORE1_LOOP_US: u32 = 1_000;
fn core1_task() -> ! {
    let mut pac = unsafe { bsp::pac::Peripherals::steal() };
    let core = unsafe { bsp::pac::CorePeripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    // The alarm used to trigger the led animations scan.
    // let mut alarm1 = timer.alarm_1().unwrap();
    // let _ = alarm1.schedule(LED_ANIM_TIME_US.microseconds());
    // alarm1.enable_interrupt(&mut timer);

    // The first thing core0 sends us is the system bus frequency.
    // The systick is based on this frequency, so we need that to
    // be accurate when sleeping via cortex_m::delay::Delay
    let sys_freq = sio.fifo.read_blocking();
    let peripheral_freq = sio.fifo.read_blocking();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    let rgb = core1_pins(pins);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let peripheral_freq = Hertz::new(peripheral_freq);
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
        if counter == LED_ANIM_TIME_US / CORE1_LOOP_US {
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
            let _ = SmartLedsWrite::write(&mut ws, brightness(gamma(leds.into_iter()), 32));
        }
    }
}

#[cfg(feature = "kb2040")]
struct KeyboardPins {
    is_right: bool,
    rotary1: Pin<Gpio29, PullUpInput>,
    rotary2: Pin<Gpio28, PullUpInput>,
    sda0: Pin<Gpio12, FunctionI2C>,
    scl0: Pin<Gpio1, FunctionI2C>,
    sda1: Pin<Gpio2, FunctionI2C>,
    scl1: Pin<Gpio3, FunctionI2C>,
    trackball_irq: Pin<Gpio13, PullUpInput>,
    boot_button: Pin<Gpio11, PullUpInput>,
    col0: DynPin,
    col1: DynPin,
    col2: DynPin,
    col3: DynPin,
    col4: DynPin,
    col5: DynPin,
    col6: DynPin,
    col7: DynPin,
    row0: DynPin,
    row1: DynPin,
    row2: DynPin,
    row3: DynPin,
}

#[cfg(feature = "pico")]
struct KeyboardPins {
    is_right: bool,
    rotary1: Pin<Gpio15, PullUpInput>,
    rotary2: Pin<Gpio16, PullUpInput>,
    sda0: Pin<Gpio20, FunctionI2C>,
    scl0: Pin<Gpio21, FunctionI2C>,
    sda1: Pin<Gpio18, FunctionI2C>,
    scl1: Pin<Gpio19, FunctionI2C>,
    trackball_irq: Pin<Gpio22, PullUpInput>,
    boot_button: Pin<Gpio28, PullUpInput>,
    led: Pin<Gpio25, PushPullOutput>,
    col0: DynPin,
    col1: DynPin,
    col2: DynPin,
    col3: DynPin,
    col4: DynPin,
    col5: DynPin,
    col6: DynPin,
    col7: DynPin,
    row0: DynPin,
    row1: DynPin,
    row2: DynPin,
    row3: DynPin,
}

#[cfg(feature = "kb2040")]
fn get_pins(pins: bsp::Pins) -> KeyboardPins {
    // Used on core1. Don't use
    let _rgb = pins.tx;

    let mut d7 = pins.d7.into_push_pull_output();
    let miso = pins.miso.into_pull_up_input();
    let is_right = d7
        .set_low()
        .and_then(|()| {
            cortex_m::asm::delay(100);
            miso.is_low()
        })
        .and_then(|is_right| d7.set_high().map(|()| is_right))
        .map(|is_right| {
            cortex_m::asm::delay(100);
            is_right
        })
        .unwrap_or(false);

    let (col0, col1, col2, col3, col4, col5, col6, col7, row0, row1, row2, row3) = if is_right {
        (
            // cols
            pins.d8.into_pull_up_input().into(),
            pins.d9.into_pull_up_input().into(),
            pins.d10.into_pull_up_input().into(),
            pins.mosi.into_pull_up_input().into(),
            miso.into_pull_up_input().into(),
            pins.sclk.into_pull_up_input().into(),
            pins.a0.into_pull_up_input().into(),
            pins.a1.into_pull_up_input().into(),
            // rows
            pins.d4.into_push_pull_output().into(),
            pins.d5.into_push_pull_output().into(),
            pins.d6.into_push_pull_output().into(),
            d7.into_push_pull_output().into(),
        )
    } else {
        (
            // cols
            pins.mosi.into_pull_up_input().into(),
            pins.d10.into_pull_up_input().into(),
            pins.d9.into_pull_up_input().into(),
            pins.d8.into_pull_up_input().into(),
            d7.into_pull_up_input().into(),
            pins.d6.into_pull_up_input().into(),
            pins.d5.into_pull_up_input().into(),
            pins.d4.into_pull_up_input().into(),
            // rows
            pins.a1.into_push_pull_output().into(),
            pins.a0.into_push_pull_output().into(),
            pins.sclk.into_push_pull_output().into(),
            miso.into_push_pull_output().into(),
        )
    };

    KeyboardPins {
        is_right,
        rotary1: pins.a3.into_pull_up_input(),
        rotary2: pins.a2.into_pull_up_input(),
        sda0: pins.sda.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        scl0: pins.rx.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        sda1: pins.d2.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        scl1: pins.d3.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        trackball_irq: pins.scl.into_pull_up_input(),
        boot_button: pins.d11.into_pull_up_input(),
        col0,
        col1,
        col2,
        col3,
        col4,
        col5,
        col6,
        col7,
        row0,
        row1,
        row2,
        row3,
    }
}

#[cfg(feature = "pico")]
fn get_pins(pins: bsp::Pins) -> KeyboardPins {
    // Used on core1. Don't use.
    let _rgb = pins.gpio27;

    // Debugging pins, don't use!
    let _debugger1 = pins.gpio0;
    let _debugger2 = pins.gpio1;

    KeyboardPins {
        is_right: true,
        rotary1: pins.gpio15.into_pull_up_input(),
        rotary2: pins.gpio16.into_pull_up_input(),
        sda0: pins.gpio20.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        scl0: pins.gpio21.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        sda1: pins.gpio18.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        scl1: pins.gpio19.into_mode::<bsp::hal::gpio::FunctionI2C>(),
        trackball_irq: pins.gpio22.into_pull_up_input(),
        boot_button: pins.gpio28.into_pull_up_input(),
        led: pins.led.into_push_pull_output(),
        col0: pins.gpio2.into_pull_up_input().into(),
        col1: pins.gpio3.into_pull_up_input().into(),
        col2: pins.gpio4.into_pull_up_input().into(),
        col3: pins.gpio5.into_pull_up_input().into(),
        col4: pins.gpio6.into_pull_up_input().into(),
        col5: pins.gpio7.into_pull_up_input().into(),
        col6: pins.gpio8.into_pull_up_input().into(),
        col7: pins.gpio9.into_pull_up_input().into(),
        row0: pins.gpio10.into_push_pull_output().into(),
        row1: pins.gpio11.into_push_pull_output().into(),
        row2: pins.gpio12.into_push_pull_output().into(),
        row3: pins.gpio13.into_push_pull_output().into(),
    }
}

// Rtic entry point. Uses the kb2040's Peripheral Access API (pac), and uses the
// PIO0_IRQ_0 interrupt to dispatch to the handlers.
#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use super::*;

    /// The number of times a switch needs to be in the same state for the
    /// debouncer to consider it a press.
    const DEBOUNCER_MIN_STATE_COUNT: u16 = 30;

    const PRESSED_BIT_MASK: u8 = 0b0100_0000;
    const PRESSED_SHIFT: u8 = 6;
    const ROW_BIT_MASK: u8 = 0b0011_1000;
    const ROW_SHIFT: u8 = 3;
    const COL_BIT_MASK: u8 = 0b0000_0111;
    const COL_SHIFT: u8 = 0;

    const NCOL_PINS: usize = 8;
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;

    #[shared]
    struct Shared {
        usb_dev: Option<usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>>,
        keyboard_class: Option<
            keyberon::hid::HidClass<
                'static,
                bsp::hal::usb::UsbBus,
                keyberon::keyboard::Keyboard<UsualLeds>,
            >,
        >,
        mouse_class: Option<usbd_hid::hid_class::HIDClass<'static, bsp::hal::usb::UsbBus>>,
        mouse_data: TrackballData,
        sio_fifo: SioFifo,
        i2c0: Either<I2CPeripheral, I2CController>,
        rotary: RotaryEncoder,
        last_rotary: Option<Event>,
        display: Display,
        timer: bsp::hal::timer::Timer,
        alarm: bsp::hal::timer::Alarm0,
        #[lock_free]
        watchdog: bsp::hal::watchdog::Watchdog,
        #[lock_free]
        matrix: matrix::Matrix<DynPin, DynPin, { NCOL_PINS }, { NROWS }>,
        layout: Layout<{ NCOLS }, { NROWS }, { NLAYERS }, CustomAction>,
        #[lock_free]
        debouncer: Debouncer<PressedKeys<{ NCOL_PINS }, { NROWS }>>,
        transform: fn(keyberon::layout::Event) -> keyberon::layout::Event,
        is_right: bool,
        boot_button: BootButton,
        scanned_events: ScannedKeys,
    }

    #[local]
    struct Local {
        #[cfg(feature = "pico")]
        led: Pin<Gpio25, PushPullOutput>,
    }

    /// Setup for the keyboard.
    #[init]
    fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let mut resets = c.device.RESETS;

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        defmt::info!("setting up multicore");
        let mut sio = Sio::new(c.device.SIO);
        let mut mc = Multicore::new(&mut c.device.PSM, &mut c.device.PPB, &mut sio);
        let cores = mc.cores();
        let core1 = &mut cores[1];
        let _test = core1.spawn(core1_task, unsafe { &mut CORE1_STACK.mem });
        // Let core1 know how fast the system clock is running
        let sys_freq = clocks.system_clock.freq().integer();
        let peripheral_freq = clocks.peripheral_clock.freq().integer();
        sio.fifo.write_blocking(sys_freq);
        sio.fifo.write_blocking(peripheral_freq);

        defmt::info!("getting pins");
        let pins = bsp::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let keyboard_pins = get_pins(pins);
        let is_right = keyboard_pins.is_right;

        defmt::info!("Configuring rotary");
        // TODO how to make this per-board?
        // The rotaries are wired differently on the left and right keyboards.
        let rotary = if is_right {
            Rotary::new(keyboard_pins.rotary1.into(), keyboard_pins.rotary2.into())
        } else {
            // TODO how to make this per-board?
            Rotary::new(keyboard_pins.rotary2.into(), keyboard_pins.rotary1.into())
        };

        defmt::info!("Configuring trackball and i2c comms");
        // Right functions as the I2C controller for communication between the halves and for USB.
        let mut i2c0 = if is_right {
            Either::Right({
                let i2c = bsp::hal::I2C::i2c0(
                    c.device.I2C0,
                    keyboard_pins.sda0,
                    keyboard_pins.scl0,
                    100.kHz(),
                    &mut resets,
                    clocks.peripheral_clock.freq(),
                );
                let i2c = TrackballInterface::new(i2c);
                let mut trackball = TrackballBuilder::<_, Pin<_, _>>::new(i2c)
                    .interrupt_pin(keyboard_pins.trackball_irq)
                    .build();
                let _ = trackball.init();
                trackball
                    .interrupt()
                    .set_interrupt_enabled(Interrupt::EdgeLow, true);
                trackball
            })
        } else {
            Either::Left(bsp::hal::I2C::new_peripheral_event_iterator(
                c.device.I2C0,
                keyboard_pins.sda0,
                keyboard_pins.scl0,
                &mut resets,
                I2C_PERIPHERAL_ADDR as u16,
            ))
        };

        defmt::info!("Configuring display");
        // Create the I²C driver, using the two pre-configured pins. This will fail
        // at compile time if the pins are in the wrong mode, or if this I²C
        // peripheral isn't available on these pins!
        let i2c1 = bsp::hal::I2C::i2c1(
            c.device.I2C1,
            keyboard_pins.sda1,
            keyboard_pins.scl1,
            400.kHz(),
            &mut resets,
            clocks.peripheral_clock.freq(),
        );

        // Create the I²C display interface:
        let interface = ssd1306::I2CDisplayInterface::new(i2c1);

        // Create a driver instance and initialize:
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate180)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        let bmp =
            Bmp::from_slice(include_bytes!("./assets/rust.bmp")).expect("Failed to load BMP image");

        // The image is an RGB565 encoded BMP, so specifying the type as `Image<Bmp<Rgb565>>` will
        // read the pixels correctly
        let im: Image<Bmp<Rgb565>> = Image::new(&bmp, Point::new(32, 0));

        // We use the `color_converted` method here to automatically convert the RGB565 image data
        // into BinaryColor values.
        im.draw(&mut display.color_converted()).unwrap();

        display.flush().unwrap();

        let mut display = display.into_terminal_mode();
        display.init().unwrap();

        defmt::info!("Configuring timer");
        let mut timer = Timer::new(c.device.TIMER, &mut resets);

        cortex_m::asm::delay(1000);

        // The Kyria PCB counts columns starting from 0 closest to the MCU.
        let transform: fn(keyberon::layout::Event) -> keyberon::layout::Event = if is_right {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 8 + j) })
        } else {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 7 - j) })
        };

        // Build the matrix that will inform which specific combinations of row and col switches
        // are being pressed.
        let matrix: matrix::Matrix<DynPin, DynPin, { NCOL_PINS }, { NROWS }> =
            cortex_m::interrupt::free(move |_cs| {
                if is_right {
                    matrix::Matrix::new(
                        [
                            keyboard_pins.col0,
                            keyboard_pins.col1,
                            keyboard_pins.col2,
                            keyboard_pins.col3,
                            keyboard_pins.col4,
                            keyboard_pins.col5,
                            keyboard_pins.col6,
                            keyboard_pins.col7,
                        ],
                        [
                            keyboard_pins.row0,
                            keyboard_pins.row1,
                            keyboard_pins.row2,
                            keyboard_pins.row3,
                        ],
                    )
                } else {
                    defmt::unreachable!()
                }
            })
            .unwrap();

        // Load the defined layout which is used later to map the matrix to specific keycodes.
        let layout =
            Layout::<{ NCOLS }, { NROWS }, { NLAYERS }, CustomAction>::new(&layout::LAYERS);
        // The debouncer prevents very tiny bounces from being registered as multiple switch
        // presses.
        let debouncer: keyberon::debounce::Debouncer<
            keyberon::matrix::PressedKeys<NCOL_PINS, NROWS>,
        > = Debouncer::new(
            PressedKeys::default(),
            PressedKeys::default(),
            DEBOUNCER_MIN_STATE_COUNT,
        );

        // The alarm used to trigger the matrix scan.
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

        defmt::info!("Creating usb bus...");

        // The bus that is used to manage the device and class below.
        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));

        // We store the bus in a static to make the borrows satisfy the rtic model, since rtic
        // needs all references to be 'static.
        unsafe {
            USB_BUS = Some(usb_bus);
        }

        // The class which specifies this device supports HID Keyboard reports.
        let keyboard_class =
            is_right.then(|| keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, UsualLeds));
        // The class which specifies this device supports HID Mouse reports.
        let mouse_class = is_right.then(|| {
            HIDClass::new(
                unsafe { USB_BUS.as_ref().unwrap() },
                MouseReport::desc(),
                60,
            )
        });
        // The device which represents the device to the system as being a "Keyberon"
        // keyboard.
        let usb_dev = is_right.then(|| keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() }));

        if is_right {
            display.write_str("Right\n").unwrap();
            if let Either::Right(trackball) = &mut i2c0 {
                let _ = trackball.set_red(128);
            }
        } else {
            display.write_str("Left\n").unwrap();
        }

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(10_000.microseconds());

        (
            Shared {
                sio_fifo: sio.fifo,
                rotary,
                last_rotary: None,
                keyboard_class,
                mouse_class,
                mouse_data: TrackballData {
                    up: 0,
                    down: 0,
                    left: 0,
                    right: 0,
                    switch_changed: false,
                    switch_pressed: false,
                },
                usb_dev,
                display,
                i2c0,
                timer,
                alarm,
                watchdog,
                matrix,
                layout,
                debouncer,
                transform,
                is_right,
                boot_button: keyboard_pins.boot_button,
                scanned_events: ArrayDeque::new(),
            },
            #[cfg(feature = "kb2040")]
            Local {},
            #[cfg(feature = "pico")]
            Local {
                led: keyboard_pins.led,
            },
            init::Monotonics(),
        )
    }

    /// Usb interrupt handler. Runs every time the host requests new data.
    #[task(
        binds = USBCTRL_IRQ,
        priority = 3,
        shared = [usb_dev, keyboard_class, mouse_class, &is_right]
    )]
    fn usb_rx(c: usb_rx::Context) {
        let usb_d = c.shared.usb_dev;
        let keyboard_class = c.shared.keyboard_class;
        let mouse_class = c.shared.mouse_class;
        let is_right = c.shared.is_right;
        if !is_right {
            return;
        }
        (usb_d, keyboard_class, mouse_class).lock(|d, k, m| {
            let _ = d
                .as_mut()
                .unwrap()
                .poll(&mut [k.as_mut().unwrap(), m.as_mut().unwrap()]);
        });
    }

    /// Process events for the layout then generate and send the Keyboard HID Report.
    #[task(
        priority = 2,
        capacity = 8,
        shared = [usb_dev, keyboard_class, i2c0, layout, display, rotary, sio_fifo],
        local = [led]
    )]
    fn handle_event(c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        let _dropg = {
            #[cfg(feature = "pico")]
            {
                c.local.led.set_high().unwrap();
                DropGuard::new(|| c.local.led.set_low().unwrap())
            }
        };
        let mut layout = c.shared.layout;
        let mut display = c.shared.display;
        let mut keyboard_class = c.shared.keyboard_class;
        let mut usb_dev = c.shared.usb_dev;
        let i2c0 = c.shared.i2c0;
        let sio_fifo = c.shared.sio_fifo;

        // If there's an event, process it with the layout and return early. We use `None`
        // to signify that the current scan is done with its events.
        if let Some(e) = event {
            let mut buf = [0u8; 64];
            let (typ, x, y) = match e {
                Event::Press(x, y) => ('P', x, y),
                Event::Release(x, y) => ('R', x, y),
            };
            let mut wrapper = Wrapper::new(&mut buf);
            let _ = writeln!(wrapper, "{}: ({},{})", typ, x, y);
            let written = wrapper.written();
            drop(wrapper);
            display.lock(|d| {
                let _ = d.write_str(unsafe { core::str::from_utf8_unchecked(&buf[..written]) });
            });
            layout.lock(|l| l.event(e));
            return;
        }

        // "Tick" the layout so that it gets to a consistent state, then read the keycodes into
        // a Keyboard HID Report.
        let report: key_code::KbHidReport = (layout, i2c0, sio_fifo).lock(|l, i2c, sf| {
            if let keyberon::layout::CustomEvent::Press(&custom_action) = l.tick() {
                let serialized = custom_action.into();
                if let Either::Right(trackball) = i2c {
                    let trackball: &mut Trackball = trackball;
                    let _ = trackball
                        .i2c()
                        .write(I2C_PERIPHERAL_ADDR, &[SET_UI, serialized]);
                }
                sf.write_blocking(serialized as u32);
                // update led state, communicate to other half
            }
            l.keycodes().collect()
        });

        // Set the keyboard report on the usb class.
        let was_report_modified = keyboard_class.lock(|k| {
            k.as_mut()
                // This function is never called on the left half. keyboard_class is always
                // populated on the right half.
                .unwrap()
                .device_mut()
                .set_keyboard_report(report.clone())
        });
        if !was_report_modified {
            return;
        }

        // If the device is not configured yet, we need to bail out.
        if usb_dev.lock(|d| d.as_ref().unwrap().state()) != UsbDeviceState::Configured {
            return;
        }

        // Watchdog will prevent the keyboard from getting stuck in this loop.
        while let Ok(0) = keyboard_class.lock(|k| k.as_mut().unwrap().write(report.as_bytes())) {}
    }

    #[task(binds = IO_IRQ_BANK0, priority = 1, shared = [i2c0, mouse_data, watchdog, display])]
    fn trackball_irq(c: trackball_irq::Context) {
        defmt::info!("trackball irq");
        let mut i2c0 = c.shared.i2c0;
        let mut mouse_data = c.shared.mouse_data;
        let mut display = c.shared.display;

        // Feed the watchdog so it knows we haven't frozen/crashed.
        // c.shared.watchdog.feed();
        let data = i2c0.lock(|i2c0| {
            if let Either::Right(trackball) = i2c0 {
                defmt::info!(
                    "interrupt is low? {}",
                    trackball.interrupt().is_low().unwrap()
                );
                let data = trackball.read().unwrap();
                let interrupt = trackball.interrupt();
                defmt::info!("interrupt is low? {}", interrupt.is_low().unwrap());
                interrupt.clear_interrupt(Interrupt::EdgeLow);
                defmt::info!("interrupt is still low? {}", interrupt.is_low().unwrap());
                return data;
            };
            defmt::unreachable!()
        });

        let mut n = 0;
        while i2c0.lock(|i2c0| {
            if let Either::Right(trackball) = i2c0 {
                trackball.interrupt().is_low().unwrap()
            } else {
                defmt::unreachable!()
            }
        }) && n < 10
        {
            n += 1;
            defmt::info!("stuck?");
            i2c0.lock(|i2c0| {
                if let Either::Right(trackball) = i2c0 {
                    trackball.interrupt().clear_interrupt(Interrupt::EdgeLow)
                }
            });
            cortex_m::asm::delay(100);
        }
        if n == 10 {
            defmt::info!("Got stuck, bailing early");
            return;
        }

        let mut buf = [0u8; 64];
        let mut wrapper = Wrapper::new(&mut buf);
        let _ = write!(
            wrapper,
            "{}{}{}{}{}{}\r",
            data.up,
            data.down,
            data.left,
            data.right,
            data.switch_changed as u8,
            data.switch_pressed as u8
        );
        let written = wrapper.written();
        drop(wrapper);

        display.lock(|d| {
            let _ = d.write_str(unsafe { core::str::from_utf8_unchecked(&buf[..written]) });
        });
        let should_spawn = mouse_data.lock(|m| {
            if m.is_dead_and_same(&data) {
                false
            } else {
                *m = data;
                true
            }
        });
        if should_spawn {
            let _ = handle_mouse_event::spawn(data);
        }
    }

    #[task(priority = 2, shared = [usb_dev, mouse_class])]
    fn handle_mouse_event(
        c: handle_mouse_event::Context,
        event: pimoroni_trackball::TrackballData,
    ) {
        let mut usb_dev = c.shared.usb_dev;
        let mut mouse_class = c.shared.mouse_class;

        let x = (event.left as i8 - event.right as i8) * 3;
        let y = (event.up as i8 - event.down as i8) * 3;

        let report = MouseReport {
            x: x.saturating_mul(x).saturating_mul(x.signum()),
            y: y.saturating_mul(y).saturating_mul(y.signum()),
            buttons: if event.switch_pressed { 1 } else { 0 },
            pan: 0,
            wheel: 0,
        };

        // If the device is not configured yet, we need to bail out.
        if usb_dev.lock(|d| d.as_ref().unwrap().state()) != UsbDeviceState::Configured {
            return;
        }

        // Watchdog will prevent the keyboard from getting stuck in this loop.
        while let Ok(0) = mouse_class.lock(|m: &mut Option<HIDClass<_>>| {
            m.as_mut()
                // This function is never called on the left half. keyboard_class is always
                // populated on the right half.
                .unwrap()
                .push_input(&report)
        }) {}
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [
            matrix,
            debouncer,
            watchdog,
            timer,
            alarm,
            &transform,
            &is_right,
            rotary,
            last_rotary,
            &boot_button,
            scanned_events,
            i2c0,
            sio_fifo
        ],
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        let timer = c.shared.timer;
        let alarm = c.shared.alarm;
        let mut rotary = c.shared.rotary;
        let mut last_rotary = c.shared.last_rotary;
        let boot_button = c.shared.boot_button;
        let mut scanned_events = c.shared.scanned_events;
        let mut i2c0 = c.shared.i2c0;
        let is_right = *c.shared.is_right;
        let sio_fifo = c.shared.sio_fifo;

        if boot_button.is_low().unwrap() {
            bsp::hal::rom_data::reset_to_usb_boot(0, 0);
        }

        // Immediately clear the interrupt and schedule the next scan alarm.
        (timer, alarm).lock(|t, a| {
            a.clear_interrupt(t);
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        // Feed the watchdog so it knows we haven't frozen/crashed.
        c.shared.watchdog.feed();

        // Get debounced, pressed keys.
        let matrix = c.shared.matrix;
        let keys_pressed = matrix.get().unwrap();
        let deb_events = c
            .shared
            .debouncer
            .events(keys_pressed)
            .map(c.shared.transform);

        let rotary_update = rotary.lock(|r| r.update().unwrap());

        // If we're on the right side of the keyboard, just send the events to the event handler
        // so they can be sent over USB.
        if is_right {
            // Try to read data from the other side first.
            let data = i2c0.lock(
                |i2c: &mut Either<I2CPeripheral, I2CController>| -> Result<_, bsp::hal::i2c::Error> {
                    if let Either::Right(trackball) = i2c {
                        let mut buf = [0u8; 16];
                        trackball.i2c().write_read(I2C_PERIPHERAL_ADDR, &[READ_KEYS], &mut buf)?;
                        Ok(buf)
                    } else {
                        defmt::unreachable!()
                    }
                },
            );

            if let Ok(data) = data {
                for d in data {
                    // Zeros indicate no key event.
                    if d == 0 {
                        break;
                    }

                    let i = (d & ROW_BIT_MASK) >> ROW_SHIFT;
                    let j = (d & COL_BIT_MASK) >> COL_SHIFT;
                    let is_pressed = (d & PRESSED_BIT_MASK) > 0;

                    handle_event::spawn(Some(if is_pressed {
                        Event::Press(i, j)
                    } else {
                        Event::Release(i, j)
                    }))
                    .unwrap();
                }
            }

            for event in deb_events {
                handle_event::spawn(Some(event)).unwrap();
            }

            if let Some(event) = last_rotary.lock(|l| l.take()) {
                match event {
                    Event::Press(x, y) => handle_event::spawn(Some(Event::Release(x, y))).unwrap(),
                    Event::Release(..) => defmt::unreachable!(),
                }
            }

            let rotary_event = match rotary_update {
                Direction::Clockwise => Some(Event::Press(3, 14)),
                Direction::CounterClockwise => Some(Event::Press(3, 12)),
                Direction::None => None,
            };
            last_rotary.lock(|l| *l = rotary_event);
            if rotary_event.is_some() {
                handle_event::spawn(rotary_event).unwrap();
            }

            handle_event::spawn(None).unwrap();
        } else {
            // coordinate and press/release is encoded in a single byte
            // the first 6 bits are the coordinate and therefore cannot go past 63
            // The last bit is to signify if it is the last byte to be sent, but
            // this is not currently used as serial rx is the highest priority
            // end? press=1/release=0 key_number
            //   7         6            543210
            let mut es: [Option<keyberon::layout::Event>; 16] = [None; 16];
            let mut last = 0;
            for (i, e) in deb_events.enumerate() {
                es[i] = Some(e);
                last = i;
            }

            if let Some(event) = last_rotary.lock(|l| l.take()) {
                match event {
                    Event::Press(x, y) => {
                        es[last] = Some(Event::Release(x, y));
                        last += 1;
                    }
                    Event::Release(..) => defmt::unreachable!(),
                }
            }

            let rotary_event = match rotary_update {
                Direction::Clockwise => Some(Event::Press(3, 3)),
                Direction::CounterClockwise => Some(Event::Press(3, 1)),
                Direction::None => None,
            };
            last_rotary.lock(|l| *l = rotary_event);
            if rotary_event.is_some() {
                es[last] = rotary_event;
                last += 1;
            }

            let stop_index = last + 1;
            let mut byte: u8;
            for (idx, e) in es.iter().enumerate().take(stop_index) {
                if let Some(ev) = e {
                    let (i, j) = ev.coord();
                    // 7 is the max value we can encode, and the highest row/col count on this board
                    if i > 7 || j > 7 {
                        continue;
                    }

                    byte = (j << COL_SHIFT) & COL_BIT_MASK;
                    byte |= (i << ROW_SHIFT) & ROW_BIT_MASK;
                    byte |= (ev.is_press() as u8) << PRESSED_SHIFT;
                    if idx + 1 == stop_index {
                        byte |= 0b1000_0000;
                    }

                    scanned_events.lock(|s: &mut ArrayDeque<[u8; 16], behavior::Wrapping>| {
                        let _ = s.push_back(byte);
                    });
                }
            }

            (i2c0, scanned_events, sio_fifo).lock(|i2c, s, sf| {
                if let Either::Left(i2c) = i2c {
                    i2c_peripheral_event_loop(i2c, s, sf);
                } else {
                    defmt::unreachable!()
                }
            });
        }
    }

    const READ_KEYS: u8 = 0x80;
    const SET_UI: u8 = 0x81;

    fn i2c_peripheral_event_loop(
        i2c: &mut I2CPeripheral,
        scanned_keys: &mut ScannedKeys,
        sio_fifo: &mut SioFifo,
    ) {
        while let Some(event) = i2c.next() {
            match event {
                I2CEvent::Start | I2CEvent::Restart => {}
                I2CEvent::TransferRead => {
                    let mut buf = [0; 16];
                    for (e, b) in scanned_keys.drain(..).zip(buf.iter_mut()) {
                        *b = e;
                    }
                    let mut sent = 0;
                    while sent < buf.len() {
                        sent += i2c.write(&buf[sent..]);
                    }
                }
                I2CEvent::TransferWrite => {
                    let mut command = 0;
                    if i2c.read(core::slice::from_mut(&mut command)) == 1 {
                        match command {
                            READ_KEYS => {
                                let mut buf = [0u8; 16];
                                i2c.read(&mut buf);
                            }
                            SET_UI => {
                                let mut layer = 0;
                                i2c.read(core::slice::from_mut(&mut layer));
                                sio_fifo.write_blocking(layer as u32)
                            }
                            _ => {
                                // No-op, we have no idea what this is.
                            }
                        }
                    }
                }
                I2CEvent::Stop => {
                    break;
                }
            }
        }
    }
}
