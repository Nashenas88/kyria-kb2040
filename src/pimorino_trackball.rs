#![allow(dead_code)]

//! I2C interface factory

//! Generic I2C interface for display drivers
use embedded_hal as hal;
use hal::blocking::i2c::{Operation, Transactional, Write, WriteRead};

/// I2C communication interface
pub struct I2CInterface<I2C> {
    i2c: I2C,
    addr: u8,
    data_byte: u8,
}

impl<I2C> I2CInterface<I2C>
where
    I2C: Write,
{
    /// Create new I2C interface for communication with a trackball driver
    pub fn new(i2c: I2C, addr: u8, data_byte: u8) -> Self {
        Self {
            i2c,
            addr,
            data_byte,
        }
    }

    /// Consume the trackball interface and return
    /// the underlying peripherial driver
    pub fn release(self) -> I2C {
        self.i2c
    }
}

const SWITCH_STATE_MASK: u8 = 0b1000_0000;

impl<I2C> I2CInterface<I2C>
where
    I2C: Write<Error = <I2C as WriteRead>::Error>
        + WriteRead<Error = <I2C as Transactional>::Error>
        + Transactional,
{
    fn chip_id(&mut self) -> Result<u16, <I2C as Write>::Error> {
        let mut byte = [0u8; 1];
        self.i2c.write_read(self.addr, &[CHIP_ID_H], &mut byte)?;
        let mut chip_id = (*unsafe { byte.get_unchecked(0) } as u16) << 8;
        self.i2c.write_read(self.addr, &[CHIP_ID_L], &mut byte)?;
        chip_id |= (*unsafe { byte.get_unchecked(0) }) as u16;
        Ok(chip_id)
    }

    fn set_red(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[RED_REGISTER, val])
    }

    fn set_green(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[GREEN_REGISTER, val])
    }

    fn set_blue(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[BLUE_REGISTER, val])
    }

    fn set_white(&mut self, val: u8) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(self.addr, &[WHITE_REGISTER, val])
    }

    fn read(&mut self) -> Result<Data, <I2C as Write>::Error> {
        let mut left = 0;
        let mut right = 0;
        let mut up = 0;
        let mut down = 0;
        let mut switch_state = 0;
        self.i2c.exec(
            self.addr,
            &mut [
                Operation::Write(&[LEFT]),
                Operation::Read(core::slice::from_mut(&mut left)),
                Operation::Write(&[RIGHT]),
                Operation::Read(core::slice::from_mut(&mut right)),
                Operation::Write(&[UP]),
                Operation::Read(core::slice::from_mut(&mut up)),
                Operation::Write(&[DOWN]),
                Operation::Read(core::slice::from_mut(&mut down)),
                Operation::Write(&[SWITCH]),
                Operation::Read(core::slice::from_mut(&mut switch_state)),
            ],
        )?;
        let switch_changed = (switch_state & !SWITCH_STATE_MASK) > 0;
        let switch_pressed = (switch_state & SWITCH_STATE_MASK) > 0;

        Ok(Data {
            left,
            right,
            up,
            down,
            switch_changed,
            switch_pressed,
        })
    }
}

/// Helper struct to create preconfigured I2C interfaces for the trackball.
#[derive(Debug, Copy, Clone)]
pub struct I2CTrackballInterface(());

impl I2CTrackballInterface {
    /// Create new builder with a default I2C address of 0x0A.
    pub fn new_interface<I>(i2c: I) -> I2CInterface<I>
    where
        I: embedded_hal::blocking::i2c::Write,
    {
        Self::new_custom_address(i2c, 0x0A)
    }

    /// Create a new I2C interface with the alternate address 0x0B as specified in the datasheet.
    pub fn new_alternate_address<I>(i2c: I) -> I2CInterface<I>
    where
        I: embedded_hal::blocking::i2c::Write,
    {
        Self::new_custom_address(i2c, 0x0B)
    }

    /// Create a new I2C interface with a custom address.
    pub fn new_custom_address<I>(i2c: I, address: u8) -> I2CInterface<I>
    where
        I: embedded_hal::blocking::i2c::Write,
    {
        I2CInterface::new(i2c, address, 0x40)
    }
}

const CHIP_ID: u16 = 0xBA11;
const VERSION: u8 = 1;
const DEFAULT_TIMEOUT: u32 = 5;

struct Data {
    left: u8,
    right: u8,
    up: u8,
    down: u8,
    switch_changed: bool,
    switch_pressed: bool,
}

struct Trackball<I> {
    i2c: I2CInterface<I>,
    interrupt_enabled: bool,
}

//--------------------------------------------------
// Variables
//--------------------------------------------------
//   private:
//     uint interrupt      = PIN_UNUSED;
//     uint32_t timeout    = DEFAULT_TIMEOUT;

// enum Command {
//     SetRed(u8),
//     SetGreen(u8),
//     SetBlue(u8),
//     SetWhite(u8),
//     ReadChipId,
//     EnableInterrupt,
// }

// impl Command {
//     fn send<I>(self, i2c: I) -> Result<([u8; 2], u8), <I as hal::blocking::i2c::Write>::Error>
//     where
//         I: hal::blocking::i2c::Write<Error = <I as hal::blocking::i2c::WriteRead>::Error>
//             + hal::blocking::i2c::WriteRead,
//     {
//     }
// }

const RED_REGISTER: u8 = 0x00;
const GREEN_REGISTER: u8 = 0x01;
const BLUE_REGISTER: u8 = 0x02;
const WHITE_REGISTER: u8 = 0x03;

const LEFT: u8 = 0x04;
const RIGHT: u8 = 0x05;
const UP: u8 = 0x06;
const DOWN: u8 = 0x07;
const SWITCH: u8 = 0x08;

const USER_FLASH: u8 = 0xD0;
const FLASH_PAGE: u8 = 0xF0;
const INTERRUPT: u8 = 0xF9;

const CHIP_ID_L: u8 = 0xFA;
const CHIP_ID_H: u8 = 0xFB;
const VERSION_REG: u8 = 0xFC;
const I2C_ADDR: u8 = 0xFD;
const CTRL: u8 = 0xFE;

const INTERRUPT_OUT_ENABLE_MASK: u8 = 0b0000_0010;
const INTERRUPT_TRIGGERED_MASK: u8 = 0b0000_0001;

enum TrackballError<I>
where
    I: Write,
{
    I2C(I::Error),
    ChipId,
}

impl<I> Trackball<I>
where
    I: Write<Error = <I as WriteRead>::Error>
        + WriteRead<Error = <I as Transactional>::Error>
        + Transactional,
{
    pub fn new(i2c: I2CInterface<I>, interrupt_enabled: bool) -> Self {
        Self {
            i2c,
            interrupt_enabled,
        }
    }

    pub fn init(&mut self) -> Result<(), TrackballError<I>> {
        let chip_id = self.i2c.chip_id().map_err(TrackballError::I2C)?;
        if chip_id == CHIP_ID {
            self.enable_interrupt().map_err(TrackballError::I2C)?;
            Ok(())
        } else {
            Err(TrackballError::ChipId)
        }
    }

    // pub fn change_address(
    //     &mut self,
    //     new_address: u8,
    // ) -> Result<(), <I as hal::blocking::i2c::Write>::Error> {
    //     self.i2c.i2c.write(self.i2c.addr, I2C_ADDR, new_address)?;
    //     wait_for_flash();
    //     Ok(())
    // }

    pub fn enable_interrupt(&mut self) -> Result<(), <I as Write>::Error> {
        let mut byte = [0u8; 1];
        self.i2c
            .i2c
            .write_read(self.i2c.addr, &[INTERRUPT], &mut byte)?;
        let mut value = *unsafe { byte.get_unchecked(0) };
        if self.interrupt_enabled {
            value |= INTERRUPT_OUT_ENABLE_MASK;
        } else {
            value &= !INTERRUPT_OUT_ENABLE_MASK;
        }

        self.i2c.i2c.write(self.i2c.addr, &[INTERRUPT, value])
    }

    pub fn get_interrupt(&self) -> bool {
        self.interrupt_enabled
    }

    pub fn set_rgbw(&mut self, r: u8, g: u8, b: u8, w: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_red(r)?;
        self.i2c.set_green(g)?;
        self.i2c.set_blue(b)?;
        self.i2c.set_white(w)?;
        Ok(())
    }

    #[inline]
    pub fn set_red(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_red(val)
    }

    #[inline]
    pub fn set_green(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_green(val)
    }

    #[inline]
    pub fn set_blue(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_blue(val)
    }

    #[inline]
    pub fn set_white(&mut self, val: u8) -> Result<(), <I as Write>::Error> {
        self.i2c.set_white(val)
    }

    #[inline]
    pub fn read(&mut self) -> Result<Data, <I as Write>::Error> {
        self.i2c.read()
    }
}

//   private:
//     void wait_for_flash();

//     uint32_t millis();
//   };
