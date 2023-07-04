//! Formatting module for helping with logging over a serial connection. This
//! is useful on the kb2040 since there are no debugging pins exposed on the
//! board.

use core::fmt;

pub struct Wrapper<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> Wrapper<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        Wrapper { buf, offset: 0 }
    }

    pub fn written(&self) -> usize {
        self.offset
    }
}

impl<'a> fmt::Write for Wrapper<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();

        // Skip over already-copied data.
        let remainder = &mut self.buf[self.offset..];
        // Check if there is space remaining (return error instead of panicking).
        if remainder.len() < bytes.len() {
            return Err(core::fmt::Error);
        }

        // Make the two slices the same length.
        let remainder = &mut remainder[..bytes.len()];
        // Copy.
        remainder.copy_from_slice(bytes);

        // Update offset to avoid overwriting.
        self.offset += bytes.len();

        Ok(())
    }
}

#[macro_export]
macro_rules! dwrite_ {
    ($display:expr, $len:expr, $($args:expr),* $(,)?) => {{
        let mut buf = [0u8; $len];
        let mut wrapper = $crate::fmt::Wrapper::new(&mut buf);
        let _ = writeln!(&mut wrapper, $($args),*);
        let written = wrapper.written();
        $display.write_str(unsafe {
            core::str::from_utf8_unchecked(&buf[..written])
        })
    }};
}

pub use dwrite_ as dwrite;

#[test]
fn test_written() {
    use std::fmt::Write;

    let data = 5;
    let mut buf = [0u8; 32];
    let mut wrapper = Wrapper::new(&mut buf);
    let _ = writeln!(&mut wrapper, "set_ui {}", data);
    let written = wrapper.written();
    let s = unsafe { core::str::from_utf8_unchecked(&buf[..written]) };
    assert_eq!(s, "set_ui 5\n");
}
