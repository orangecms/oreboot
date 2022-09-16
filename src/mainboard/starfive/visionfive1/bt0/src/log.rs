//! Log system for BT0
// essentially copied from sunxi/nezha

use crate::init::{Error, Serial};
use core::fmt;
use embedded_hal::serial::nb::Write;
use nb::block;
use spin::{Mutex, Once};

#[doc(hidden)]
pub(crate) static LOGGER: Once<LockedLogger> = Once::new();

type S = Wrap<Serial>;

// type `Serial` is declared outside this crate, avoid orphan rule
pub(crate) struct Wrap<T>(T);

#[doc(hidden)]
pub(crate) struct LockedLogger {
    pub(crate) inner: Mutex<S>,
}

impl fmt::Write for S {
    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), fmt::Error> {
        for byte in s.as_bytes() {
            block!(self.0.write(*byte)).unwrap();
        }
        block!(self.0.flush()).unwrap();
        Ok(())
    }
}

#[inline]
pub fn set_logger(serial: Serial) {
    crate::init::uart_write('L');
    // FIXME: this here seems broken...
    let l = LOGGER.try_call_once(|| -> Result<LockedLogger, u8> {
        let m = Mutex::new(Wrap(serial));
        Ok(LockedLogger { inner: m })
    });
    match l {
        Ok(_) => crate::init::uart_write('O'),
        Err(_) => crate::init::uart_write('E'),
    }
    crate::init::uart_write('L');
}

#[inline]
#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    LOGGER.wait().inner.lock().write_fmt(args).unwrap();
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
