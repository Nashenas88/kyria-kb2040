//! Keyboard firmware for Kyria split keyboard on an Adafruit kb2040.

#![no_std]
#![no_main]

use crate::fmt::Wrapper;
use crate::layout::{CustomAction, NCOLS, NLAYERS, NROWS};
use crate::leds::UsualLeds;
#[cfg(feature = "trackball")]
use crate::mouse::MouseReportExt;
use crate::pins::get_pins;
#[cfg(feature = "kb2040")]
use adafruit_kb2040 as bsp;
use arraydeque::{behavior, ArrayDeque};
use bsp::hal::clocks::init_clocks_and_plls;
#[cfg(all(feature = "kb2040", feature = "trackball"))]
use bsp::hal::gpio::bank0::Gpio13;
#[cfg(all(feature = "pico", feature = "trackball"))]
use bsp::hal::gpio::bank0::Gpio22;
#[cfg(feature = "kb2040")]
use bsp::hal::gpio::bank0::{Gpio1, Gpio11, Gpio12, Gpio2, Gpio25, Gpio3};
#[cfg(feature = "sf2040")]
use bsp::hal::gpio::bank0::{Gpio1, Gpio16, Gpio2, Gpio25, Gpio3};
#[cfg(feature = "pico")]
use bsp::hal::gpio::bank0::{Gpio18, Gpio19, Gpio20, Gpio21, Gpio25, Gpio28};
#[cfg(feature = "trackball")]
use bsp::hal::gpio::Interrupt;
#[cfg(any(feature = "kb2040", feature = "pico"))]
use bsp::hal::gpio::PullUpInput;
use bsp::hal::gpio::{DynPin, FunctionI2C, Pin, PushPullOutput};
use bsp::hal::i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator};
use bsp::hal::multicore::{Multicore, Stack};
use bsp::hal::sio::SioFifo;
use bsp::hal::timer::{Alarm, Timer};
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
#[cfg(any(feature = "kb2040", feature = "pico"))]
use embedded_hal::digital::v2::InputPin;
#[cfg(feature = "pico")]
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use embedded_time::duration::Extensions as _;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions as _;
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::PressedKeys;
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
use mock_defmt as defmt;
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
use panic_halt as _;
#[cfg(feature = "pico")]
use panic_probe as _;
#[cfg(feature = "trackball")]
use pimoroni_trackball::{I2CInterface as TrackballInterface, TrackballBuilder, TrackballData};
#[cfg(feature = "trackball")]
use pimoroni_trackball_driver as pimoroni_trackball;
use rotary_encoder_hal::{Direction, Rotary};
#[cfg(feature = "pico")]
use rp_pico as bsp;
#[cfg(feature = "sf2040")]
use sparkfun_pro_micro_rp2040 as bsp;
use ssd1306::mode::{DisplayConfig, TerminalMode};
use ssd1306::prelude::I2CInterface;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::Ssd1306;
use tinybmp::Bmp;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
#[cfg(feature = "trackball")]
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};
#[cfg(feature = "trackball")]
use usbd_hid::hid_class::HIDClass;

mod anim;
mod core1;
mod fmt;
mod layout;
mod leds;
mod matrix;
#[cfg(feature = "trackball")]
mod mouse;
mod pins;
mod ws2812;

#[cfg(feature = "kb2040")]
type I2C0Controller = I2C<I2C0, (Pin<Gpio12, FunctionI2C>, Pin<Gpio1, FunctionI2C>)>;
#[cfg(feature = "sf2040")]
type I2C0Controller = I2C<I2C0, (Pin<Gpio16, FunctionI2C>, Pin<Gpio1, FunctionI2C>)>;
#[cfg(feature = "pico")]
type I2C0Controller = I2C<I2C0, (Pin<Gpio20, FunctionI2C>, Pin<Gpio21, FunctionI2C>)>;

#[cfg(all(feature = "kb2040", feature = "trackball"))]
type Trackball = pimoroni_trackball::Trackball<I2C0Controller, Pin<Gpio13, PullUpInput>>;
#[cfg(all(feature = "pico", feature = "trackball"))]
type Trackball = pimoroni_trackball::Trackball<I2C0Controller, Pin<Gpio22, PullUpInput>>;

#[cfg(feature = "kb2040")]
type Display = Ssd1306<
    I2CInterface<I2C<I2C1, (Pin<Gpio2, FunctionI2C>, Pin<Gpio3, FunctionI2C>)>>,
    DisplaySize128x64,
    TerminalMode,
>;
#[cfg(feature = "sf2040")]
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

#[cfg(feature = "kb2040")]
type I2CPeripheral =
    I2CPeripheralEventIterator<I2C0, (Pin<Gpio12, FunctionI2C>, Pin<Gpio1, FunctionI2C>)>;
#[cfg(feature = "sf2040")]
type I2CPeripheral =
    I2CPeripheralEventIterator<I2C0, (Pin<Gpio16, FunctionI2C>, Pin<Gpio1, FunctionI2C>)>;
#[cfg(feature = "pico")]
type I2CPeripheral =
    I2CPeripheralEventIterator<I2C0, (Pin<Gpio20, FunctionI2C>, Pin<Gpio21, FunctionI2C>)>;

#[cfg(feature = "trackball")]
type I2CController = Trackball;
#[cfg(not(feature = "trackball"))]
type I2CController = I2C0Controller;

type RotaryEncoder = Rotary<DynPin, DynPin>;

type ScannedKeys = ArrayDeque<[u8; BOARD_MAX_KEYS], behavior::Wrapping>;

#[cfg(feature = "kb2040")]
type BootButton = Pin<Gpio11, PullUpInput>;
#[cfg(feature = "sf2040")]
type BootButton = ();
#[cfg(feature = "pico")]
type BootButton = Pin<Gpio28, PullUpInput>;

const I2C_PERIPHERAL_ADDR: u8 = 0x56;

/// The amount of time between each scan of the matrix for switch presses.
/// This is always the time from the *start* of the previous scan.
const SCAN_TIME_US: u32 = 1000;
const COMMS_CHECK_US: u32 = 500;
const BOARD_MAX_KEYS: usize = 8;

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
    #[allow(dead_code)]
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

static mut CORE1_STACK: Stack<4096> = Stack::new();
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
        #[cfg(feature = "trackball")]
        mouse_class: Option<usbd_hid::hid_class::HIDClass<'static, bsp::hal::usb::UsbBus>>,
        #[cfg(feature = "trackball")]
        mouse_data: TrackballData,
        sio_fifo: SioFifo,
        i2c0: Either<I2CPeripheral, I2CController>,
        rotary: RotaryEncoder,
        last_rotary: Option<Event>,
        display: Display,
        timer: Timer,
        alarm0: bsp::hal::timer::Alarm0,
        alarm1: Option<bsp::hal::timer::Alarm1>,
        #[lock_free]
        watchdog: bsp::hal::watchdog::Watchdog,
        #[lock_free]
        matrix: matrix::Matrix<DynPin, DynPin, { NCOL_PINS }, { NROWS }>,
        layout: Layout<{ NCOLS }, { NROWS }, { NLAYERS }, CustomAction>,
        #[lock_free]
        debouncer: Debouncer<PressedKeys<{ NCOL_PINS }, { NROWS }>>,
        transform: fn(keyberon::layout::Event) -> keyberon::layout::Event,
        is_right: bool,
        scanned_events: ScannedKeys,
    }

    #[local]
    struct Local {
        #[cfg(any(feature = "pico", feature = "kb2040"))]
        boot_button: BootButton,
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
        let _test = core1.spawn(crate::core1::core1_task, unsafe { &mut CORE1_STACK.mem });
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
        #[allow(unused_mut)]
        let mut i2c0 = if is_right {
            Either::Right({
                let i2c = bsp::hal::I2C::i2c0(
                    c.device.I2C0,
                    keyboard_pins.sda0,
                    keyboard_pins.scl0,
                    #[cfg(feature = "trackball")]
                    100.kHz(),
                    #[cfg(not(feature = "trackball"))]
                    400.kHz(),
                    &mut resets,
                    clocks.peripheral_clock.freq(),
                );
                #[cfg(feature = "trackball")]
                {
                    let i2c = TrackballInterface::new(i2c);
                    let mut trackball = TrackballBuilder::<_, Pin<_, _>>::new(i2c)
                        .interrupt_pin(keyboard_pins.trackball_irq)
                        .build();
                    let _ = trackball.init();
                    trackball
                        .interrupt()
                        .set_interrupt_enabled(Interrupt::EdgeLow, true);
                    trackball
                }
                #[cfg(not(feature = "trackball"))]
                {
                    i2c
                }
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
        let mut alarm0 = timer.alarm_0().unwrap();
        let _ = alarm0.schedule(SCAN_TIME_US.microseconds());
        alarm0.enable_interrupt();

        let alarm1 = if is_right {
            None
        } else {
            let mut alarm1 = timer.alarm_1().unwrap();
            let _ = alarm1.schedule(COMMS_CHECK_US.microseconds());
            alarm1.enable_interrupt();
            Some(alarm1)
        };

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
        #[cfg(feature = "trackball")]
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
            #[cfg(feature = "trackball")]
            if let Either::Right(trackball) = &mut i2c0 {
                let _ = trackball.set_red(128);
            }
        } else {
            display.write_str("Left\n").unwrap();
        }

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start((16777215 >> 1).microseconds()); // 10_000.microseconds());

        (
            Shared {
                sio_fifo: sio.fifo,
                rotary,
                last_rotary: None,
                keyboard_class,
                #[cfg(feature = "trackball")]
                mouse_class,
                #[cfg(feature = "trackball")]
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
                alarm0,
                alarm1,
                watchdog,
                matrix,
                layout,
                debouncer,
                transform,
                is_right,
                scanned_events: ArrayDeque::new(),
            },
            #[cfg(feature = "sf2040")]
            Local {},
            #[cfg(feature = "kb2040")]
            Local {
                boot_button: keyboard_pins.boot_button,
            },
            #[cfg(feature = "pico")]
            Local {
                boot_button: keyboard_pins.boot_button,
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
        #[cfg(feature = "trackball")]
        let mouse_class = c.shared.mouse_class;
        let is_right = c.shared.is_right;
        if !is_right {
            return;
        }
        #[cfg(feature = "trackball")]
        (usb_d, keyboard_class, mouse_class).lock(|d, k, m| {
            let _ = d
                .as_mut()
                .unwrap()
                .poll(&mut [k.as_mut().unwrap(), m.as_mut().unwrap()]);
        });
        #[cfg(not(feature = "trackball"))]
        (usb_d, keyboard_class).lock(|d, k| {
            let _ = d.as_mut().unwrap().poll(&mut [k.as_mut().unwrap()]);
        });
    }

    /// Process events for the layout then generate and send the Keyboard HID Report.
    #[task(
        priority = 2,
        capacity = 8,
        shared = [usb_dev, keyboard_class, i2c0, layout, rotary, sio_fifo],
    )]
    fn handle_event(c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        let mut layout = c.shared.layout;
        let mut keyboard_class = c.shared.keyboard_class;
        let mut usb_dev = c.shared.usb_dev;
        let i2c0 = c.shared.i2c0;
        let sio_fifo = c.shared.sio_fifo;

        // If there's an event, process it with the layout and return early. We use `None`
        // to signify that the current scan is done with its events.
        if let Some(e) = event {
            #[cfg(feature = "pico")]
            match e {
                Event::Press(x, y) => defmt::info!("p {} {}", x, y),
                Event::Release(x, y) => defmt::info!("r {} {}", x, y),
            };
            layout.lock(|l| l.event(e));
            return;
        }

        // "Tick" the layout so that it gets to a consistent state, then read the keycodes into
        // a Keyboard HID Report.
        let report: key_code::KbHidReport = (layout, i2c0, sio_fifo).lock(|l, i2c, sf| {
            if let keyberon::layout::CustomEvent::Press(&custom_action) = l.tick() {
                let serialized = custom_action.into();
                #[cfg(feature = "trackball")]
                if let Either::Right(trackball) = i2c {
                    let trackball: &mut Trackball = trackball;
                    let _ = trackball
                        .i2c()
                        .write(I2C_PERIPHERAL_ADDR, &[SET_UI, serialized]);
                }
                #[cfg(not(feature = "trackball"))]
                if let Either::Right(i2c) = i2c {
                    let _ = i2c.write(I2C_PERIPHERAL_ADDR, &[SET_UI, serialized]);
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

    // #[cfg(feature = "trackball")]
    // #[task(binds = IO_IRQ_BANK0, priority = 1, shared = [i2c0, mouse_data, watchdog, display])]
    // fn trackball_irq(c: trackball_irq::Context) {
    //     defmt::info!("trackball irq");
    //     let mut i2c0 = c.shared.i2c0;
    //     let mut mouse_data = c.shared.mouse_data;
    //     let mut display = c.shared.display;

    //     // Feed the watchdog so it knows we haven't frozen/crashed.
    //     // c.shared.watchdog.feed();
    //     let data = i2c0.lock(|i2c0| {
    //         if let Either::Right(trackball) = i2c0 {
    //             defmt::info!(
    //                 "interrupt is low? {}",
    //                 trackball.interrupt().is_low().unwrap()
    //             );
    //             let data = trackball.read().unwrap();
    //             let interrupt = trackball.interrupt();
    //             defmt::info!("interrupt is low? {}", interrupt.is_low().unwrap());
    //             interrupt.clear_interrupt(Interrupt::EdgeLow);
    //             defmt::info!("interrupt is still low? {}", interrupt.is_low().unwrap());
    //             return data;
    //         };
    //         defmt::unreachable!()
    //     });

    //     let mut n = 0;
    //     while i2c0.lock(|i2c0| {
    //         if let Either::Right(trackball) = i2c0 {
    //             trackball.interrupt().is_low().unwrap()
    //         } else {
    //             defmt::unreachable!()
    //         }
    //     }) && n < 10
    //     {
    //         n += 1;
    //         defmt::info!("stuck?");
    //         i2c0.lock(|i2c0| {
    //             if let Either::Right(trackball) = i2c0 {
    //                 trackball.interrupt().clear_interrupt(Interrupt::EdgeLow)
    //             }
    //         });
    //         cortex_m::asm::delay(100);
    //     }
    //     if n == 10 {
    //         defmt::info!("Got stuck, bailing early");
    //         return;
    //     }

    //     let mut buf = [0u8; 64];
    //     let mut wrapper = Wrapper::new(&mut buf);
    //     let _ = write!(
    //         wrapper,
    //         "{}{}{}{}{}{}\r",
    //         data.up,
    //         data.down,
    //         data.left,
    //         data.right,
    //         data.switch_changed as u8,
    //         data.switch_pressed as u8
    //     );
    //     let written = wrapper.written();
    //     drop(wrapper);

    //     display.lock(|d| {
    //         let _ = d.write_str(unsafe { core::str::from_utf8_unchecked(&buf[..written]) });
    //     });
    //     let should_spawn = mouse_data.lock(|m| {
    //         if m.is_dead_and_same(&data) {
    //             false
    //         } else {
    //             *m = data;
    //             true
    //         }
    //     });
    //     if should_spawn {
    //         let _ = handle_mouse_event::spawn(data);
    //     }
    // }

    // #[cfg(feature = "trackball")]
    // #[task(priority = 2, shared = [usb_dev, mouse_class])]
    // fn handle_mouse_event(
    //     c: handle_mouse_event::Context,
    //     event: pimoroni_trackball::TrackballData,
    // ) {
    //     let mut usb_dev = c.shared.usb_dev;
    //     let mut mouse_class = c.shared.mouse_class;

    //     let x = (event.left as i8 - event.right as i8).saturating_mul(3);
    //     let y = (event.up as i8 - event.down as i8).saturating_mul(3);

    //     let report = MouseReport {
    //         x: x.saturating_mul(x).saturating_mul(x.signum()),
    //         y: y.saturating_mul(y).saturating_mul(y.signum()),
    //         buttons: if event.switch_pressed { 1 } else { 0 },
    //         pan: 0,
    //         wheel: 0,
    //     };

    //     // If the device is not configured yet, we need to bail out.
    //     if usb_dev.lock(|d| d.as_ref().unwrap().state()) != UsbDeviceState::Configured {
    //         return;
    //     }

    //     // Watchdog will prevent the keyboard from getting stuck in this loop.
    //     while let Ok(0) = mouse_class.lock(|m: &mut Option<HIDClass<_>>| {
    //         m.as_mut()
    //             // This function is never called on the left half. keyboard_class is always
    //             // populated on the right half.
    //             .unwrap()
    //             .push_input(&report)
    //     }) {}
    // }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [
            matrix,
            debouncer,
            watchdog,
            timer,
            alarm0,
            &transform,
            &is_right,
            rotary,
            last_rotary,
            scanned_events,
            i2c0,
        ],
        local = [boot_button, led]
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        #[cfg(feature = "pico")]
        let mut timer = c.shared.timer;
        let mut alarm0 = c.shared.alarm0;
        let mut rotary = c.shared.rotary;
        let mut last_rotary = c.shared.last_rotary;
        #[cfg(any(feature = "kb2040", feature = "pico"))]
        let boot_button = c.local.boot_button;
        let mut scanned_events = c.shared.scanned_events;
        let mut i2c0 = c.shared.i2c0;
        let is_right = *c.shared.is_right;

        #[cfg(any(feature = "kb2040", feature = "pico"))]
        if boot_button.is_low().unwrap() {
            bsp::hal::rom_data::reset_to_usb_boot(0, 0);
        }

        // Immediately clear the interrupt and schedule the next scan alarm0.
        alarm0.lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        // let _timer_dropg = {
        //     #[cfg(feature = "pico")]
        //     {
        //         let start = timer.lock(|t| t.get_counter());
        //         DropGuard::new(move || {
        //             let end = timer.lock(|t| t.get_counter());
        //             defmt::info!("board comms took {}", end.saturating_sub(start));
        //         })
        //     }
        // };

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
            let dropg = {
                #[cfg(feature = "pico")]
                {
                    c.local.led.set_high().unwrap();
                    DropGuard::new(|| c.local.led.set_low().unwrap())
                }
            }; // Try to read data from the other side first.
            let _timer_dropg = {
                #[cfg(feature = "pico")]
                {
                    let start = timer.lock(|t| t.get_counter());
                    defmt::info!("starting comms timer at {}", start);
                    DropGuard::new(move || {
                        let end = timer.lock(|t| t.get_counter());
                        defmt::info!("board comms took {}", end.saturating_sub(start));
                    })
                }
            };
            let data = i2c0.lock(
                |i2c: &mut Either<I2CPeripheral, I2CController>| -> Result<_, bsp::hal::i2c::Error> {
                    #[cfg(feature="trackball")]
                    if let Either::Right(trackball) = i2c {
                        let mut buf = [0u8; BOARD_MAX_KEYS];
                        trackball.i2c().write_read(I2C_PERIPHERAL_ADDR, &[READ_KEYS], &mut buf)?;
                        Ok(buf)
                    } else {
                        defmt::unreachable!()
                    }
                    #[cfg(not(feature="trackball"))]
                    if let Either::Right(i2c) = i2c {
                        let mut buf = [0u8; BOARD_MAX_KEYS];
                        i2c.write_read(I2C_PERIPHERAL_ADDR, &[READ_KEYS], &mut buf)?;
                        Ok(buf)
                    } else {
                        defmt::unreachable!()
                    }
                },
            );
            #[allow(clippy::drop_copy)]
            drop(_timer_dropg);
            #[allow(clippy::drop_copy)]
            drop(dropg);

            match data {
                Ok(data) => {
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
                #[cfg(feature = "pico")]
                Err(e) => {
                    defmt::info!(
                        "Could not read other side: {}",
                        match e {
                            bsp::hal::i2c::Error::Abort(_) => "abort",
                            bsp::hal::i2c::Error::InvalidReadBufferLength => "invalid read",
                            bsp::hal::i2c::Error::InvalidWriteBufferLength => "invalid right",
                            bsp::hal::i2c::Error::AddressOutOfRange(_) => "addr out of range",
                            bsp::hal::i2c::Error::AddressReserved(_) => "addr reserved",
                            _ => "unexpected",
                        }
                    );
                }
                #[cfg(not(feature = "pico"))]
                _ => {}
            }

            for event in deb_events {
                defmt::info!("spawning some event");
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
                defmt::info!("spawning rotary event");
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
            let mut es: [Option<keyberon::layout::Event>; BOARD_MAX_KEYS] = [None; BOARD_MAX_KEYS];
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

                    scanned_events.lock(
                        |s: &mut ArrayDeque<[u8; BOARD_MAX_KEYS], behavior::Wrapping>| {
                            let _ = s.push_back(byte);
                        },
                    );
                }
            }

            // (i2c0, scanned_events, sio_fifo).lock(|i2c, s, sf| {
            //     if let Either::Left(i2c) = i2c {
            //         i2c_peripheral_event_loop(i2c, s, sf);
            //     } else {
            //         defmt::unreachable!()
            //     }
            // });
        }
    }

    const READ_KEYS: u8 = 0x80;
    const SET_UI: u8 = 0x81;

    #[task(
        binds = TIMER_IRQ_1,
        priority = 2,
        shared = [
            scanned_events,
            i2c0,
            sio_fifo,
            alarm1
        ]
    )]
    fn i2c_peripheral_event_loop(c: i2c_peripheral_event_loop::Context) {
        let scanned_events = c.shared.scanned_events;
        let i2c0 = c.shared.i2c0;
        let sio_fifo = c.shared.sio_fifo;
        let mut alarm1 = c.shared.alarm1;

        // Immediately clear the interrupt and schedule the next scan alarm0.
        alarm1.lock(|a| {
            // Panic safe since we can only enter this function if this is not None;
            let a = a.as_mut().unwrap();
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US.microseconds());
        });

        (i2c0, scanned_events, sio_fifo).lock(|i2c0, scanned_events, sio_fifo| {
            let i2c = if let Either::Left(i2c) = i2c0 {
                i2c
            } else {
                defmt::unreachable!();
            };

            while let Some(event) = i2c.next() {
                match event {
                    I2CEvent::Start | I2CEvent::Restart => {}
                    I2CEvent::TransferRead => {
                        let mut buf = [0; BOARD_MAX_KEYS];
                        for (e, b) in scanned_events.drain(..).zip(buf.iter_mut()) {
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
                                    let mut buf = [0u8; BOARD_MAX_KEYS];
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
        });
    }
}
