//! Log system for BT0
// essentially copied from sunxi/nezha

use core::fmt;
use embedded_hal::serial::{ErrorType,nb::Write};
use nb::block;

pub trait Serial: ErrorType + Write {
}

/// Error types that may happen when serial transfer
#[derive(Debug)]
pub struct Error {
    pub kind: embedded_hal::serial::ErrorKind,
}

impl embedded_hal::serial::Error for Error {
    #[inline]
    fn kind(&self) -> embedded_hal::serial::ErrorKind {
        self.kind
    }
}

pub type SerialLogger = &'static mut dyn Serial<Error=Error>;

#[doc(hidden)]
pub(crate) static mut LOGGER: Option<SerialLogger> = None;

impl<'a> fmt::Write for SerialLogger {
    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), fmt::Error> {
        for byte in s.as_bytes() {
            block!(self.write(*byte)).unwrap();
        }
        block!(self.flush()).unwrap();
        Ok(())
    }
}

#[inline]
pub fn set_logger(serial: SerialLogger + 'static) {
    unsafe {
        LOGGER = Some(serial);
    }
}

#[inline]
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    unsafe {
        if let Some(l) = &mut LOGGER {
             l.write_fmt(args).unwrap();
        }
    }
}

/*
#[inline]
#[doc(hidden)]
pub fn _debug(num: u8) {
    unsafe {
        if let Some(l) = &mut LOGGER {
            l.inner.debug(num);
        }
    }
}
*/

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::log::_print(core::format_args!($($arg)*));
    }
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\r\n"));
    ($($arg:tt)*) => {
        $crate::log::_print(core::format_args!($($arg)*));
        $crate::print!("\r\n");
    }
}
