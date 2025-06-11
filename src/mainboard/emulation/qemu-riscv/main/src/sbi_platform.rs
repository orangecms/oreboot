// FIXME: does not work just like that when removing rustsbi from Cargo.toml
// use oreboot_arch::riscv64::rustsbi::{self, HartMask, RustSBI, SbiRet};
use rustsbi::{HartMask, RustSBI, SbiRet};

use oreboot_arch::riscv64::riscv::register::{self as reg, mhartid, mie, mip};

use crate::util::write64;

const DEBUG: bool = false;
const DEBUG_TIMER: bool = true;

#[derive(RustSBI)]
pub struct PlatSbi {
    timer: Timer,
}

pub fn init() -> PlatSbi {
    init_pmp();
    PlatSbi { timer: Timer }
}

fn init_pmp() {
    // Protect pmpaddr1-2 - this is our code
    let cfg = 0x0000_0000_000f_0f0fusize;
    reg::pmpcfg0::write(cfg);
    reg::pmpcfg2::write(0); // nothing active here
    reg::pmpaddr0::write(0);
    reg::pmpaddr1::write(0x0000_0000_8000_0000usize >> 2);
    reg::pmpaddr2::write(0x0000_0000_8020_0000usize >> 2);
    reg::pmpaddr3::write(0xffff_ffff_0000_0000usize >> 2);
}

struct Timer;
impl rustsbi::Timer for Timer {
    fn set_timer(&self, stime_value: u64) {
        if DEBUG && DEBUG_TIMER {
            println!("[SBI] setTimer {stime_value}");
        }
        // Clear any pending timer
        unsafe { mip::clear_stimer() };
        // Set new value for this hart
        let hartid = mhartid::read();
        write64(crate::mem_map::MTIME_COMPARE + 4 * hartid, stime_value);
        // Reenable the interrupt
        unsafe { mie::set_mtimer() }
    }
}
