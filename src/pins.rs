#[cfg(feature = "kb2040")]
use crate::bsp::hal::gpio::bank0::{Gpio0, Gpio1, Gpio11, Gpio12, Gpio2, Gpio28, Gpio29, Gpio3};
#[cfg(feature = "sf2040")]
use crate::bsp::hal::gpio::bank0::{Gpio0, Gpio1, Gpio16, Gpio2, Gpio28, Gpio29, Gpio3};
#[cfg(feature = "pico")]
use crate::bsp::hal::gpio::bank0::{
    Gpio17, Gpio18, Gpio19, Gpio20, Gpio21, Gpio25, Gpio26, Gpio27, Gpio28,
};
use crate::bsp::hal::gpio::{
    DynPinId, FunctionI2C, FunctionPio0, FunctionSioInput, FunctionSioOutput, Pin, PullDown,
    PullNone, PullUp,
};
use embedded_hal::digital::v2::InputPin;
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
use embedded_hal::digital::v2::OutputPin;

pub(crate) type RowPin = Pin<DynPinId, FunctionSioOutput, PullNone>;
pub(crate) type ColPin = Pin<DynPinId, FunctionSioInput, PullUp>;
pub(crate) type RotaryPin = Pin<DynPinId, FunctionSioInput, PullUp>;
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
pub(crate) type RgbPin = Pin<Gpio0, FunctionPio0, PullDown>;
#[cfg(feature = "pico")]
pub(crate) type RgbPin = Pin<Gpio17, FunctionPio0, PullDown>;

#[cfg(any(feature = "kb2040", feature = "sf2040"))]
pub(crate) type DisplayI2cSda = Pin<Gpio2, FunctionI2C, PullUp>;
#[cfg(any(feature = "kb2040", feature = "sf2040"))]
pub(crate) type DisplayI2cScl = Pin<Gpio3, FunctionI2C, PullUp>;
#[cfg(feature = "pico")]
pub(crate) type DisplayI2cSda = Pin<Gpio18, FunctionI2C, PullUp>;
#[cfg(feature = "pico")]
pub(crate) type DisplayI2cScl = Pin<Gpio19, FunctionI2C, PullUp>;

#[cfg(feature = "kb2040")]
pub(crate) type CommsI2cSda = Pin<Gpio12, FunctionI2C, PullUp>;
#[cfg(feature = "kb2040")]
pub(crate) type CommsI2cScl = Pin<Gpio1, FunctionI2C, PullUp>;
#[cfg(feature = "sf2040")]
pub(crate) type CommsI2cSda = Pin<Gpio16, FunctionI2C, PullUp>;
#[cfg(feature = "sf2040")]
pub(crate) type CommsI2cScl = Pin<Gpio1, FunctionI2C, PullUp>;
#[cfg(feature = "pico")]
pub(crate) type CommsI2cSda = Pin<Gpio20, FunctionI2C, PullUp>;
#[cfg(feature = "pico")]
pub(crate) type CommsI2cScl = Pin<Gpio21, FunctionI2C, PullUp>;

#[cfg(feature = "kb2040")]
pub(crate) type ControllerI2cSda = Pin<Gpio12, FunctionI2C, PullUp>;
#[cfg(feature = "kb2040")]
pub(crate) type ControllerI2cScl = Pin<Gpio1, FunctionI2C, PullUp>;
#[cfg(feature = "sf2040")]
pub(crate) type ControllerI2cSda = Pin<Gpio16, FunctionI2C, PullUp>;
#[cfg(feature = "sf2040")]
pub(crate) type ControllerI2cScl = Pin<Gpio1, FunctionI2C, PullUp>;
#[cfg(feature = "pico")]
pub(crate) type ControllerI2cSda = Pin<Gpio20, FunctionI2C, PullUp>;
#[cfg(feature = "pico")]
pub(crate) type ControllerI2cScl = Pin<Gpio21, FunctionI2C, PullUp>;

#[cfg(feature = "kb2040")]
pub(crate) type BootButton = Pin<Gpio11, FunctionSioInput, PullUp>;
#[cfg(feature = "sf2040")]
pub(crate) type BootButton = ();
#[cfg(feature = "pico")]
pub(crate) type BootButton = Pin<Gpio28, FunctionSioInput, PullUp>;

#[cfg(feature = "pico")]
pub(crate) type LedPin = Pin<Gpio25, FunctionSioOutput, PullNone>;
#[cfg(not(feature = "pico"))]
pub(crate) type LedPin = ();

#[cfg(feature = "kb2040")]
pub(crate) struct KeyboardPins {
    pub(crate) is_right: bool,
    pub(crate) sda0: Pin<Gpio12, FunctionI2C, PullUp>,
    pub(crate) scl0: Pin<Gpio1, FunctionI2C, PullUp>,
    pub(crate) sda1: Pin<Gpio2, FunctionI2C, PullUp>,
    pub(crate) scl1: Pin<Gpio3, FunctionI2C, PullUp>,
    pub(crate) boot_button: Pin<Gpio11, FunctionSioInput, PullUp>,
    pub(crate) col0: ColPin,
    pub(crate) col1: ColPin,
    pub(crate) col2: ColPin,
    pub(crate) col3: ColPin,
    pub(crate) col4: ColPin,
    pub(crate) col5: ColPin,
    pub(crate) col6: ColPin,
    pub(crate) col7: ColPin,
    pub(crate) row0: RowPin,
    pub(crate) row1: RowPin,
    pub(crate) row2: RowPin,
    pub(crate) row3: RowPin,
}

#[cfg(feature = "kb2040")]
pub(crate) struct Core1Pins {
    pub(crate) rgb: RgbPin,
    pub(crate) rotary1: Pin<Gpio29, FunctionSioInput, PullUp>,
    pub(crate) rotary2: Pin<Gpio28, FunctionSioInput, PullUp>,
}

#[cfg(feature = "sf2040")]
pub(crate) struct KeyboardPins {
    pub(crate) is_right: bool,
    pub(crate) sda0: Pin<Gpio16, FunctionI2C, PullUp>,
    pub(crate) scl0: Pin<Gpio1, FunctionI2C, PullUp>,
    pub(crate) sda1: Pin<Gpio2, FunctionI2C, PullUp>,
    pub(crate) scl1: Pin<Gpio3, FunctionI2C, PullUp>,
    pub(crate) col0: ColPin,
    pub(crate) col1: ColPin,
    pub(crate) col2: ColPin,
    pub(crate) col3: ColPin,
    pub(crate) col4: ColPin,
    pub(crate) col5: ColPin,
    pub(crate) col6: ColPin,
    pub(crate) col7: ColPin,
    pub(crate) row0: RowPin,
    pub(crate) row1: RowPin,
    pub(crate) row2: RowPin,
    pub(crate) row3: RowPin,
}

#[cfg(feature = "sf2040")]
pub(crate) struct Core1Pins {
    pub(crate) rgb: RgbPin,
    pub(crate) rotary1: Pin<Gpio29, FunctionSioInput, PullUp>,
    pub(crate) rotary2: Pin<Gpio28, FunctionSioInput, PullUp>,
}

#[cfg(feature = "pico")]
pub(crate) struct KeyboardPins {
    pub(crate) is_right: bool,
    pub(crate) sda0: Pin<Gpio20, FunctionI2C, PullUp>,
    pub(crate) scl0: Pin<Gpio21, FunctionI2C, PullUp>,
    pub(crate) sda1: Pin<Gpio18, FunctionI2C, PullUp>,
    pub(crate) scl1: Pin<Gpio19, FunctionI2C, PullUp>,
    pub(crate) boot_button: Pin<Gpio28, FunctionSioInput, PullUp>,
    pub(crate) led: Pin<Gpio25, FunctionSioOutput, PullNone>,
    pub(crate) col0: ColPin,
    pub(crate) col1: ColPin,
    pub(crate) col2: ColPin,
    pub(crate) col3: ColPin,
    pub(crate) col4: ColPin,
    pub(crate) col5: ColPin,
    pub(crate) col6: ColPin,
    pub(crate) col7: ColPin,
    pub(crate) row0: RowPin,
    pub(crate) row1: RowPin,
    pub(crate) row2: RowPin,
    pub(crate) row3: RowPin,
}

#[cfg(feature = "pico")]
pub(crate) struct Core1Pins {
    pub(crate) rgb: RgbPin,
    pub(crate) rotary1: Pin<Gpio26, FunctionSioInput, PullUp>,
    pub(crate) rotary2: Pin<Gpio27, FunctionSioInput, PullUp>,
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
            pins.d8.into_pull_up_input().into_dyn_pin(),
            pins.d9.into_pull_up_input().into_dyn_pin(),
            pins.d10.into_pull_up_input().into_dyn_pin(),
            pins.mosi.into_pull_up_input().into_dyn_pin(),
            miso.into_pull_up_input().into_dyn_pin(),
            pins.sclk.into_pull_up_input().into_dyn_pin(),
            pins.a0.into_pull_up_input().into_dyn_pin(),
            pins.a1.into_pull_up_input().into_dyn_pin(),
            // rows
            pins.d4
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.d5
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.d6
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            d7.into_pull_type().into_push_pull_output().into_dyn_pin(),
        )
    } else {
        (
            // cols
            pins.mosi.into_pull_up_input().into_dyn_pin(),
            pins.d10.into_pull_up_input().into_dyn_pin(),
            pins.d9.into_pull_up_input().into_dyn_pin(),
            pins.d8.into_pull_up_input().into_dyn_pin(),
            d7.into_pull_up_input().into_dyn_pin(),
            pins.d6.into_pull_up_input().into_dyn_pin(),
            pins.d5.into_pull_up_input().into_dyn_pin(),
            pins.d4.into_pull_up_input().into_dyn_pin(),
            // rows
            pins.a1
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.a0
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.sclk
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            miso.into_pull_type().into_push_pull_output().into_dyn_pin(),
        )
    };

    KeyboardPins {
        is_right,
        sda0: pins.sda.into_pull_type().into_function(),
        scl0: pins.rx.into_pull_type().into_function(),
        sda1: pins.d2.into_pull_type().into_function(),
        scl1: pins.d3.into_pull_type().into_function(),
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
            pins.tx1.into_pull_up_input().into_dyn_pin(),
            pins.rx1.into_pull_up_input().into_dyn_pin(),
            pins.ncs.into_pull_up_input().into_dyn_pin(),
            pins.copi.into_pull_up_input().into_dyn_pin(),
            miso.into_pull_up_input().into_dyn_pin(),
            pins.sck.into_pull_up_input().into_dyn_pin(),
            pins.adc0.into_pull_up_input().into_dyn_pin(),
            pins.adc1.into_pull_up_input().into_dyn_pin(),
            // rows
            pins.gpio4
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.gpio5
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.gpio6
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            gpio7
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
        )
    } else {
        (
            // cols
            pins.copi.into_pull_up_input().into_dyn_pin(),
            pins.ncs.into_pull_up_input().into_dyn_pin(),
            pins.rx1.into_pull_up_input().into_dyn_pin(),
            pins.tx1.into_pull_up_input().into_dyn_pin(),
            gpio7.into_pull_up_input().into_dyn_pin(),
            pins.gpio6.into_pull_up_input().into_dyn_pin(),
            pins.gpio5.into_pull_up_input().into_dyn_pin(),
            pins.gpio4.into_pull_up_input().into_dyn_pin(),
            // rows
            pins.adc1
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.adc0
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            pins.sck
                .into_pull_type()
                .into_push_pull_output()
                .into_dyn_pin(),
            miso.into_pull_type().into_push_pull_output().into_dyn_pin(),
        )
    };

    KeyboardPins {
        is_right,
        sda0: pins.sda.into_pull_type().into_function(),
        scl0: pins.rx0.into_pull_type().into_function(),
        sda1: pins.gpio2.into_pull_type().into_function(),
        scl1: pins.gpio3.into_pull_type().into_function(),
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
        sda0: pins.gpio20.into_pull_type().into_function(),
        scl0: pins.gpio21.into_pull_type().into_function(),
        sda1: pins.gpio18.into_pull_type().into_function(),
        scl1: pins.gpio19.into_pull_type().into_function(),
        boot_button: pins.gpio28.into_pull_up_input(),
        led: pins.led.into_pull_type().into_push_pull_output(),
        col0: pins.gpio2.into_pull_up_input().into_dyn_pin(),
        col1: pins.gpio3.into_pull_up_input().into_dyn_pin(),
        col2: pins.gpio4.into_pull_up_input().into_dyn_pin(),
        col3: pins.gpio5.into_pull_up_input().into_dyn_pin(),
        col4: pins.gpio6.into_pull_up_input().into_dyn_pin(),
        col5: pins.gpio7.into_pull_up_input().into_dyn_pin(),
        col6: pins.gpio8.into_pull_up_input().into_dyn_pin(),
        col7: pins.gpio9.into_pull_up_input().into_dyn_pin(),
        row0: pins
            .gpio10
            .into_pull_type()
            .into_push_pull_output()
            .into_dyn_pin(),
        row1: pins
            .gpio11
            .into_pull_type()
            .into_push_pull_output()
            .into_dyn_pin(),
        row2: pins
            .gpio12
            .into_pull_type()
            .into_push_pull_output()
            .into_dyn_pin(),
        row3: pins
            .gpio13
            .into_pull_type()
            .into_push_pull_output()
            .into_dyn_pin(),
    }
}

#[cfg(feature = "kb2040")]
pub(crate) fn core1_pins(pins: crate::bsp::Pins) -> Core1Pins {
    Core1Pins {
        rgb: pins.tx.into_function(),
        rotary1: pins.a3.into_pull_up_input(),
        rotary2: pins.a2.into_pull_up_input(),
    }
}

#[cfg(feature = "sf2040")]
pub(crate) fn core1_pins(pins: crate::bsp::Pins) -> Core1Pins {
    Core1Pins {
        rgb: pins.tx0.into_function(),
        rotary1: pins.adc3.into_pull_up_input(),
        rotary2: pins.adc2.into_pull_up_input(),
    }
}

#[cfg(feature = "pico")]
pub(crate) fn core1_pins(pins: crate::bsp::Pins) -> Core1Pins {
    Core1Pins {
        rgb: pins.gpio17.into_function(),
        rotary1: pins.gpio26.into_pull_up_input(),
        rotary2: pins.gpio27.into_pull_up_input(),
    }
}
