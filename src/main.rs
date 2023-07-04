//! Keyboard firmware for Kyria split keyboard on an Adafruit kb2040.

#![no_std]
#![no_main]

use crate::pins::{
    get_pins, BootButton, ColPin, CommsI2cScl, CommsI2cSda, ControllerI2cScl, ControllerI2cSda,
    LedPin, RowPin,
};
#[cfg(feature = "kb2040")]
use adafruit_kb2040 as bsp;
use arraydeque::{behavior, ArrayDeque};
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator};
use bsp::hal::multicore::{Multicore, Stack};
use bsp::hal::sio::SioFifo;
use bsp::hal::timer::{Alarm, Timer};
use bsp::hal::usb::UsbBus;
use bsp::hal::watchdog::Watchdog;
use bsp::hal::{Clock, Sio, I2C};
use bsp::pac::I2C0;
use bsp::XOSC_CRYSTAL_FREQ;
use core::sync::atomic::{AtomicU8, Ordering};
#[cfg(feature = "pico")]
use defmt_rtt as _;
#[cfg(any(feature = "kb2040", feature = "pico"))]
use embedded_hal::digital::v2::InputPin;
#[cfg(feature = "pico")]
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use fugit::{ExtU32, RateExtU32};
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::Matrix;
use kyria_kb2040::cross_talk::{InterProcessCommand, InterProcessResponse};
use kyria_kb2040::layout::CustomEventExt;
use kyria_kb2040::layout::{CustomAction, NCOLS, NLAYERS, NROWS};
use kyria_kb2040::leds::UsualLeds;
use kyria_kb2040::{cross_talk, log, media};
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
use panic_halt as _;
#[cfg(feature = "pico")]
use panic_probe as _;
#[cfg(feature = "pico")]
use rp_pico as bsp;
#[cfg(feature = "sf2040")]
use sparkfun_pro_micro_rp2040 as bsp;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceState;
use usbd_hid::descriptor::{MediaKey, MediaKeyboardReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

mod core1;
mod pins;

type I2C0Controller = I2C<I2C0, (ControllerI2cSda, ControllerI2cScl)>;
type I2CPeripheral = I2CPeripheralEventIterator<I2C0, (CommsI2cSda, CommsI2cScl)>;
type I2CController = I2C0Controller;
type ScannedKeys = ArrayDeque<u8, BOARD_MAX_KEYS, behavior::Wrapping>;

const NCOL_PINS: usize = 8;
type PressedKeys = [[bool; NCOL_PINS]; NROWS];

const I2C_PERIPHERAL_ADDR: u8 = 0x56;

/// The amount of time between each scan of the matrix for switch presses.
/// This is always the time from the *start* of the previous scan.
const SCAN_TIME_US: u32 = 1000;
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
static LED_STATE: AtomicU8 = AtomicU8::new(0);
static LED_STATE_REF: &'static AtomicU8 = &LED_STATE;
static mut TIMER: Option<Timer> = None;
static TIMER_ADDR: &'static Option<Timer> = unsafe { &TIMER };

// Rtic entry point. Uses the kb2040's Peripheral Access API (pac), and uses the
// PIO0_IRQ_0 interrupt to dispatch to the handlers.
#[rtic::app(device = bsp::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use super::*;

    /// The number of times a switch needs to be in the same state for the
    /// debouncer to consider it a press.
    const DEBOUNCER_MIN_STATE_COUNT: u16 = 30;

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
        media_class: Option<HIDClass<'static, bsp::hal::usb::UsbBus>>,
        media_report: MediaKeyboardReport,
        i2c0: Either<I2CPeripheral, I2CController>,
        alarm0: bsp::hal::timer::Alarm0,
        #[lock_free]
        watchdog: bsp::hal::watchdog::Watchdog,
        #[lock_free]
        matrix: Matrix<ColPin, RowPin, { NCOL_PINS }, { NROWS }>,
        layout: Layout<{ NCOLS }, { NROWS }, { NLAYERS }, CustomAction>,
        #[lock_free]
        debouncer: Debouncer<PressedKeys>,
        transform: fn(keyberon::layout::Event) -> keyberon::layout::Event,
        is_right: bool,
        scanned_events: ScannedKeys,
    }

    #[local]
    struct Local {
        #[cfg(any(feature = "pico", feature = "kb2040"))]
        boot_button: BootButton,
        #[cfg(feature = "pico")]
        led: LedPin,
        sio_fifo: SioFifo,
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

        let mut sio = Sio::new(c.device.SIO);
        let mut mc = Multicore::new(&mut c.device.PSM, &mut c.device.PPB, &mut sio.fifo);
        let cores = mc.cores();
        let core1 = &mut cores[1];
        let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, crate::core1::core1_task);
        // Let core1 know how fast the system clock is running
        let sys_freq = clocks.system_clock.freq().to_Hz();
        let peripheral_freq = clocks.peripheral_clock.freq().to_Hz();
        sio.fifo.write_blocking(sys_freq);
        sio.fifo.write_blocking(peripheral_freq);
        sio.fifo.write_blocking(LED_STATE_REF as *const _ as u32);

        let pins = bsp::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let keyboard_pins = get_pins(pins);
        let is_right = keyboard_pins.is_right;
        sio.fifo.write_blocking(is_right as u32);

        // Right functions as the I2C controller for communication between the halves and for USB.
        let i2c0 = if is_right {
            Either::Right({
                let i2c = bsp::hal::I2C::i2c0(
                    c.device.I2C0,
                    keyboard_pins.sda0,
                    keyboard_pins.scl0,
                    100.kHz(),
                    &mut resets,
                    clocks.peripheral_clock.freq(),
                );
                // Prevent interrupts from firing on the right side.
                bsp::pac::NVIC::mask(bsp::pac::interrupt::I2C0_IRQ);
                i2c.disable_interrupts();
                i2c
            })
        } else {
            Either::Left(bsp::hal::I2C::new_peripheral_event_iterator_with_irq(
                c.device.I2C0,
                keyboard_pins.sda0,
                keyboard_pins.scl0,
                &mut resets,
                I2C_PERIPHERAL_ADDR as u16,
            ))
        };

        let mut timer = Timer::new(c.device.TIMER, &mut resets, &clocks);

        cortex_m::asm::delay(1000);

        // The Kyria PCB counts columns starting from 0 closest to the MCU.
        let transform: fn(keyberon::layout::Event) -> keyberon::layout::Event = if is_right {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 8 + j) })
        } else {
            |e| e.transform(|i: u8, j: u8| -> (u8, u8) { (i, 7 - j) })
        };

        // Build the matrix that will inform which specific combinations of row and col switches
        // are being pressed.
        let matrix: Matrix<ColPin, RowPin, { NCOL_PINS }, { NROWS }> =
            cortex_m::interrupt::free(move |_cs| {
                Matrix::new(
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
        let layout = Layout::<{ NCOLS }, { NROWS }, { NLAYERS }, CustomAction>::new(
            &kyria_kb2040::layout::LAYERS,
        );
        // The debouncer prevents very tiny bounces from being registered as multiple switch
        // presses.
        let debouncer: Debouncer<PressedKeys> = Debouncer::new(
            PressedKeys::default(),
            PressedKeys::default(),
            DEBOUNCER_MIN_STATE_COUNT,
        );

        // The alarm used to trigger the matrix scan.
        let mut alarm0 = timer.alarm_0().unwrap();
        let _ = alarm0.schedule(SCAN_TIME_US.micros());
        alarm0.enable_interrupt();

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
        // The class which specifies this device supports HID Media reports.
        let media_class = is_right.then(|| {
            HIDClass::new(
                unsafe { USB_BUS.as_ref().unwrap() },
                MediaKeyboardReport::desc(),
                60,
            )
        });
        // The device which represents the device to the system as being a "Keyberon"
        // keyboard.
        let usb_dev = is_right.then(|| keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() }));

        unsafe {
            TIMER = Some(timer);
        }
        sio.fifo.write_blocking(TIMER_ADDR as *const _ as u32);
        let sio_fifo = sio.fifo;

        // Do not enable watchdog when debugging.
        #[cfg(not(feature = "debug"))]
        {
            watchdog.start(40_000.micros());
            // Start watchdog and feed it with the lowest priority task at 1000hz
            // watchdog.start(10_000.micros());
        }

        (
            Shared {
                keyboard_class,
                media_class,
                media_report: MediaKeyboardReport {
                    usage_id: MediaKey::Zero.into(),
                },
                usb_dev,
                i2c0,
                alarm0,
                watchdog,
                matrix,
                layout,
                debouncer,
                transform,
                is_right,
                scanned_events: ArrayDeque::new(),
            },
            #[cfg(feature = "sf2040")]
            Local { sio_fifo },
            #[cfg(feature = "kb2040")]
            Local {
                sio_fifo,
                boot_button: keyboard_pins.boot_button,
            },
            #[cfg(feature = "pico")]
            Local {
                sio_fifo,
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
        shared = [usb_dev, keyboard_class, media_class, &is_right]
    )]
    fn usb_rx(c: usb_rx::Context) {
        let usb_d = c.shared.usb_dev;
        let keyboard_class = c.shared.keyboard_class;
        let media_class = c.shared.media_class;
        let is_right = c.shared.is_right;
        if !is_right {
            return;
        }
        (usb_d, keyboard_class, media_class).lock(|d, k, e| {
            let _ = d
                .as_mut()
                .unwrap()
                .poll(&mut [k.as_mut().unwrap(), e.as_mut().unwrap()]);
        });
    }

    /// Process events for the layout then generate and send the Keyboard HID Report.
    #[task(
        priority = 2,
        capacity = 8,
        shared = [usb_dev, keyboard_class, media_class, media_report, i2c0, layout], //, rotary],
    )]
    fn handle_event(c: handle_event::Context, event: Option<keyberon::layout::Event>) {
        let mut layout = c.shared.layout;
        let mut keyboard_class = c.shared.keyboard_class;
        let media_class = c.shared.media_class;
        let mut usb_dev = c.shared.usb_dev;
        let mut last_media_report = c.shared.media_report;
        let i2c0 = c.shared.i2c0;

        // If there's an event, process it with the layout and return early. We use `None`
        // to signify that the current scan is done with its events.
        if let Some(e) = event {
            match e {
                Event::Press(x, y) => log::info!("p {} {}", x, y),
                Event::Release(x, y) => log::info!("r {} {}", x, y),
            };
            layout.lock(|l| l.event(e));
            return;
        }

        // "Tick" the layout so that it gets to a consistent state, then read the keycodes into
        // a Keyboard HID Report.
        let (report, media_report): (key_code::KbHidReport, Option<MediaKeyboardReport>) =
            (layout, i2c0).lock(|l, i2c| {
                let media_report = match l.tick() {
                    // update led state, communicate to other half
                    ref event @ keyberon::layout::CustomEvent::Press(ref custom_action) => {
                        if custom_action.is_led() {
                            let serialized = event.serialize();
                            let Either::Right(i2c) = i2c else {
                                log::unreachable!();
                            };
                            if let Err(e) = i2c.write(I2C_PERIPHERAL_ADDR, &[SET_UI, serialized]) {
                                log::error!("Failed to write to left side over i2c: {:?}", e);
                            }
                            LED_STATE.store(serialized, Ordering::Relaxed);

                            None
                        } else {
                            media::media_report_for_action(**custom_action)
                        }
                    }
                    ref event @ keyberon::layout::CustomEvent::Release(ref custom_action) => {
                        if custom_action.is_led() {
                            let serialized = event.serialize();
                            let Either::Right(i2c) = i2c else {
                                log::unreachable!();
                            };
                            if let Err(e) = i2c.write(I2C_PERIPHERAL_ADDR, &[SET_UI, serialized]) {
                                log::error!("Failed to write to left side over i2c: {:?}", e);
                            }
                            LED_STATE.store(serialized, Ordering::Relaxed);
                            None
                        } else {
                            Some(media::release_for_media_action())
                        }
                    }
                    _ => None, // no-op for no-event
                };
                (l.keycodes().collect(), media_report)
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
        let media_report_modified = {
            if let Some(media_report) = media_report {
                last_media_report.lock(|l| {
                    if l.usage_id == media_report.usage_id {
                        false
                    } else {
                        *l = media_report;
                        true
                    }
                })
            } else {
                false
            }
        };
        if !(was_report_modified || media_report_modified) {
            return;
        }

        // If the device is not configured yet, we need to bail out.
        if usb_dev.lock(|d| d.as_ref().unwrap().state()) != UsbDeviceState::Configured {
            return;
        }

        if was_report_modified {
            // Watchdog will prevent the keyboard from getting stuck in this loop.
            while let Ok(0) = keyboard_class.lock(|k| k.as_mut().unwrap().write(report.as_bytes()))
            {
            }
        }

        if media_report_modified {
            let mut locker = (last_media_report, media_class);
            // Watchdog will prevent the keyboard from getting stuck in this loop.
            while let Ok(0) = locker.lock(|l, m| {
                m.as_mut().unwrap().push_input(l).or_else(|e| {
                    if let usb_device::UsbError::WouldBlock = e {
                        Ok(0)
                    } else {
                        Err(e)
                    }
                })
            }) {}
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [
            matrix,
            debouncer,
            watchdog,
            alarm0,
            &transform,
            &is_right,
            scanned_events,
            i2c0,
        ],
        local = [sio_fifo, boot_button, led]
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        let mut alarm0 = c.shared.alarm0;
        #[cfg(any(feature = "kb2040", feature = "pico"))]
        let boot_button = c.local.boot_button;
        let mut scanned_events = c.shared.scanned_events;
        let mut i2c0 = c.shared.i2c0;
        let is_right = *c.shared.is_right;
        let sio_fifo = c.local.sio_fifo;

        #[cfg(any(feature = "kb2040", feature = "pico"))]
        if boot_button.is_low().unwrap() {
            bsp::hal::rom_data::reset_to_usb_boot(0, 0);
        }

        // Immediately clear the interrupt and schedule the next scan alarm0.
        alarm0.lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US.micros());
        });

        // Feed the watchdog so it knows we haven't frozen/crashed.
        c.shared.watchdog.feed();

        sio_fifo.write_blocking(InterProcessCommand::ReadRotary.into());

        // Get debounced, pressed keys.
        let matrix = c.shared.matrix;
        let keys_pressed = matrix.get_with_delay(|| cortex_m::asm::delay(100)).unwrap();
        let deb_events = c
            .shared
            .debouncer
            .events(keys_pressed)
            .map(c.shared.transform);

        // If we're on the right side of the keyboard, just send the events to the event handler
        // so they can be sent over USB.
        if is_right {
            let dropg = {
                #[cfg(feature = "pico")]
                {
                    c.local.led.set_high().unwrap();
                    DropGuard::new(|| c.local.led.set_low().unwrap())
                }
            };
            let data = i2c0.lock(
                |i2c: &mut Either<I2CPeripheral, I2CController>| -> Result<_, bsp::hal::i2c::Error> {
                    let Either::Right(i2c) = i2c else {
                        log::unreachable!()
                    };
                    let mut buf = [0u8; BOARD_MAX_KEYS];
                    i2c.write_read(I2C_PERIPHERAL_ADDR, &[READ_KEYS, 42], &mut buf)?;
                    Ok(buf)
                },
            );
            #[allow(clippy::drop_copy)]
            drop(dropg);

            if let Ok(data) = data {
                for d in data {
                    let event = cross_talk::deserialize(d);
                    if event.is_some() {
                        handle_event::spawn(event).unwrap();
                    }
                }
            }

            for event in deb_events {
                handle_event::spawn(Some(event)).unwrap();
            }

            let result: Result<InterProcessResponse, u32> = sio_fifo.read_blocking().try_into();
            match result {
                Ok(InterProcessResponse::Rotary(Some(rotary_event))) => {
                    handle_event::spawn(Some(rotary_event)).unwrap();
                }
                Ok(InterProcessResponse::Rotary(None)) => {}
                Ok(InterProcessResponse::Error) => {
                    log::error!("Unexpected rotary value");
                }
                Err(e) => {
                    log::error!("Unexpected rotary value: {}", e);
                }
            };

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

            let result: Result<InterProcessResponse, u32> = sio_fifo.read_blocking().try_into();
            match result {
                Ok(InterProcessResponse::Rotary(Some(rotary_event))) => {
                    es[last] = Some(rotary_event);
                    last += 1;
                }
                Ok(InterProcessResponse::Rotary(None)) => {}
                Ok(InterProcessResponse::Error) => {
                    log::error!("Unexpected rotary value");
                }
                Err(e) => {
                    log::error!("Unexpected rotary value: {}", e);
                }
            };

            let stop_index = last + 1;
            for e in es.iter().take(stop_index).flatten() {
                let Some(byte) = cross_talk::serialize(*e) else {
                    continue;
                };
                scanned_events.lock(
                    |s: &mut ArrayDeque<u8, BOARD_MAX_KEYS, behavior::Wrapping>| {
                        let _ = s.push_back(byte);
                    },
                );
            }
        }
    }

    const READ_KEYS: u8 = 0x80;
    const SET_UI: u8 = 0x81;

    #[task(
        binds = I2C0_IRQ,
        priority = 4,
        shared = [
            scanned_events,
            i2c0,
        ]
    )]
    fn i2c_peripheral_event_loop(c: i2c_peripheral_event_loop::Context) {
        let scanned_events = c.shared.scanned_events;
        let i2c0 = c.shared.i2c0;

        (i2c0, scanned_events).lock(|i2c0, scanned_events| {
            let i2c = match i2c0 {
                Either::Left(i2c) => i2c,
                Either::Right(_) => {
                    // Don't panic. Sometimes the tx_empty interrupt triggers even
                    // though it's masked. It can't be cleared with a read, so just return.
                    return;
                }
            };

            let mut read_count = 0;
            let mut command = [0u8; 2];
            while let Some(event) = i2c.next() {
                match event {
                    I2CEvent::Start | I2CEvent::Restart => {
                        read_count = 0;
                        command = [0u8; 2];
                    }
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
                        while read_count < command.len() {
                            read_count += i2c.read(&mut command[read_count..]);
                        }
                        read_count = 0;
                        match command {
                            [READ_KEYS, _] => {
                                // no-op
                            }
                            [SET_UI, layer] => {
                                LED_STATE.store(layer, Ordering::Relaxed);
                            }
                            _ => {
                                log::info!("Unknown command: {:?}", command);
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
