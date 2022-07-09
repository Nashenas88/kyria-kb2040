#[cfg(feature = "kb2040")]
use crate::bsp::hal::gpio::bank0::{
    Gpio0, Gpio1, Gpio11, Gpio12, Gpio13, Gpio2, Gpio28, Gpio29, Gpio3,
};
#[cfg(feature = "sf2040")]
use crate::bsp::hal::gpio::bank0::{Gpio0, Gpio1, Gpio16, Gpio2, Gpio28, Gpio29, Gpio3};
#[cfg(feature = "pico")]
use crate::bsp::hal::gpio::bank0::{
    Gpio17, Gpio18, Gpio19, Gpio20, Gpio21, Gpio25, Gpio26, Gpio27, Gpio28,
};
#[cfg(feature = "pico")]
use crate::bsp::hal::gpio::PushPullOutput;
use crate::bsp::hal::gpio::{DynPin, FunctionI2C, Pin, PullDownDisabled, PullUpInput};
use embedded_hal::digital::v2::InputPin;
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "kb2040")]
pub(crate) struct KeyboardPins {
    pub(crate) is_right: bool,
    pub(crate) rotary1: Pin<Gpio29, PullUpInput>,
    pub(crate) rotary2: Pin<Gpio28, PullUpInput>,
    pub(crate) sda0: Pin<Gpio12, FunctionI2C>,
    pub(crate) scl0: Pin<Gpio1, FunctionI2C>,
    pub(crate) sda1: Pin<Gpio2, FunctionI2C>,
    pub(crate) scl1: Pin<Gpio3, FunctionI2C>,
    pub(crate) boot_button: Pin<Gpio11, PullUpInput>,
    pub(crate) col0: DynPin,
    pub(crate) col1: DynPin,
    pub(crate) col2: DynPin,
    pub(crate) col3: DynPin,
    pub(crate) col4: DynPin,
    pub(crate) col5: DynPin,
    pub(crate) col6: DynPin,
    pub(crate) col7: DynPin,
    pub(crate) row0: DynPin,
    pub(crate) row1: DynPin,
    pub(crate) row2: DynPin,
    pub(crate) row3: DynPin,
}
#[cfg(feature = "sf2040")]
pub(crate) struct KeyboardPins {
    pub(crate) is_right: bool,
    pub(crate) rotary1: Pin<Gpio29, PullUpInput>,
    pub(crate) rotary2: Pin<Gpio28, PullUpInput>,
    pub(crate) sda0: Pin<Gpio16, FunctionI2C>,
    pub(crate) scl0: Pin<Gpio1, FunctionI2C>,
    pub(crate) sda1: Pin<Gpio2, FunctionI2C>,
    pub(crate) scl1: Pin<Gpio3, FunctionI2C>,
    pub(crate) col0: DynPin,
    pub(crate) col1: DynPin,
    pub(crate) col2: DynPin,
    pub(crate) col3: DynPin,
    pub(crate) col4: DynPin,
    pub(crate) col5: DynPin,
    pub(crate) col6: DynPin,
    pub(crate) col7: DynPin,
    pub(crate) row0: DynPin,
    pub(crate) row1: DynPin,
    pub(crate) row2: DynPin,
    pub(crate) row3: DynPin,
}

#[cfg(feature = "pico")]
pub(crate) struct KeyboardPins {
    pub(crate) is_right: bool,
    pub(crate) rotary1: Pin<Gpio26, PullUpInput>,
    pub(crate) rotary2: Pin<Gpio27, PullUpInput>,
    pub(crate) sda0: Pin<Gpio20, FunctionI2C>,
    pub(crate) scl0: Pin<Gpio21, FunctionI2C>,
    pub(crate) sda1: Pin<Gpio18, FunctionI2C>,
    pub(crate) scl1: Pin<Gpio19, FunctionI2C>,
    pub(crate) boot_button: Pin<Gpio28, PullUpInput>,
    pub(crate) led: Pin<Gpio25, PushPullOutput>,
    pub(crate) col0: DynPin,
    pub(crate) col1: DynPin,
    pub(crate) col2: DynPin,
    pub(crate) col3: DynPin,
    pub(crate) col4: DynPin,
    pub(crate) col5: DynPin,
    pub(crate) col6: DynPin,
    pub(crate) col7: DynPin,
    pub(crate) row0: DynPin,
    pub(crate) row1: DynPin,
    pub(crate) row2: DynPin,
    pub(crate) row3: DynPin,
}

#[cfg(feature = "kb2040")]
pub(crate) fn get_pins(pins: crate::bsp::Pins) -> KeyboardPins {
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
        sda0: pins.sda.into_mode::<FunctionI2C>(),
        scl0: pins.rx.into_mode::<FunctionI2C>(),
        sda1: pins.d2.into_mode::<FunctionI2C>(),
        scl1: pins.d3.into_mode::<FunctionI2C>(),
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

#[cfg(feature = "sf2040")]
pub(crate) fn get_pins(pins: crate::bsp::Pins) -> KeyboardPins {
    // Used on core1. Don't use
    let _rgb = pins.tx0;

    let mut gpio7 = pins.gpio7.into_push_pull_output();
    let miso = pins.cipo.into_pull_up_input();
    let is_right = gpio7
        .set_low()
        .and_then(|()| {
            cortex_m::asm::delay(100);
            miso.is_low()
        })
        .and_then(|is_right| gpio7.set_high().map(|()| is_right))
        .map(|is_right| {
            cortex_m::asm::delay(100);
            is_right
        })
        .unwrap_or(false);

    let (col0, col1, col2, col3, col4, col5, col6, col7, row0, row1, row2, row3) = if is_right {
        (
            // cols
            pins.tx1.into_pull_up_input().into(),
            pins.rx1.into_pull_up_input().into(),
            pins.ncs.into_pull_up_input().into(),
            pins.copi.into_pull_up_input().into(),
            miso.into_pull_up_input().into(),
            pins.sck.into_pull_up_input().into(),
            pins.adc0.into_pull_up_input().into(),
            pins.adc1.into_pull_up_input().into(),
            // rows
            pins.gpio4.into_push_pull_output().into(),
            pins.gpio5.into_push_pull_output().into(),
            pins.gpio6.into_push_pull_output().into(),
            gpio7.into_push_pull_output().into(),
        )
    } else {
        (
            // cols
            pins.copi.into_pull_up_input().into(),
            pins.ncs.into_pull_up_input().into(),
            pins.rx1.into_pull_up_input().into(),
            pins.tx1.into_pull_up_input().into(),
            gpio7.into_pull_up_input().into(),
            pins.gpio6.into_pull_up_input().into(),
            pins.gpio5.into_pull_up_input().into(),
            pins.gpio4.into_pull_up_input().into(),
            // rows
            pins.adc1.into_push_pull_output().into(),
            pins.adc0.into_push_pull_output().into(),
            pins.sck.into_push_pull_output().into(),
            miso.into_push_pull_output().into(),
        )
    };

    KeyboardPins {
        is_right,
        rotary1: pins.adc3.into_pull_up_input(),
        rotary2: pins.adc2.into_pull_up_input(),
        sda0: pins.sda.into_mode::<FunctionI2C>(),
        scl0: pins.rx0.into_mode::<FunctionI2C>(),
        sda1: pins.gpio2.into_mode::<FunctionI2C>(),
        scl1: pins.gpio3.into_mode::<FunctionI2C>(),
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
pub(crate) fn get_pins(pins: crate::bsp::Pins) -> KeyboardPins {
    // Used on core1. Don't use.
    let _rgb = pins.gpio17;

    // Debugging pins, don't use!
    let _debugger1 = pins.gpio0;
    let _debugger2 = pins.gpio1;

    let pin16 = pins.gpio15.into_pull_up_input();

    KeyboardPins {
        is_right: pin16.is_low().unwrap(),
        rotary1: pins.gpio26.into_pull_up_input(),
        rotary2: pins.gpio27.into_pull_up_input(),
        sda0: pins.gpio20.into_mode::<FunctionI2C>(),
        scl0: pins.gpio21.into_mode::<FunctionI2C>(),
        sda1: pins.gpio18.into_mode::<FunctionI2C>(),
        scl1: pins.gpio19.into_mode::<FunctionI2C>(),
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

#[cfg(feature = "kb2040")]
pub(crate) fn core1_pins(pins: crate::bsp::Pins) -> Pin<Gpio0, PullDownDisabled> {
    pins.tx
}

#[cfg(feature = "sf2040")]
pub(crate) fn core1_pins(pins: crate::bsp::Pins) -> Pin<Gpio0, PullDownDisabled> {
    pins.tx0
}

#[cfg(feature = "pico")]
pub(crate) fn core1_pins(pins: crate::bsp::Pins) -> Pin<Gpio17, PullDownDisabled> {
    pins.gpio17
}
