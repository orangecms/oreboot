//! Log system for BT0
// essentially copied from sunxi/nezha

use crate::init::Serial;
use core::fmt;
use embedded_hal::serial::nb::Write;
use nb::block;

#[doc(hidden)]
pub(crate) static LOGGER: Option<Logger> = None;

type S = Wrap<Serial>;

// type `Serial` is declared outside this crate, avoid orphan rule
pub(crate) struct Wrap<T>(T);

#[doc(hidden)]
pub(crate) struct Logger {
    pub(crate) inner: Option<S>,
}

impl fmt::Write for S {
    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), fmt::Error> {
        for byte in s.as_bytes() {
            block!(self.0.write(*byte)).ok();
        }
        block!(self.0.flush()).ok();
        Ok(())
    }
}

#[inline]
pub fn set_logger(serial: Serial) {
    crate::init::uart_write('L');
    LOGGER = Some(Logger {
        inner: Some(Wrap(serial)),
    });
}

#[inline]
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    LOGGER?.inner?.write_fmt(args).ok()
}

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
