use core::arch::asm;
use riscv::register::{mhartid, mie, mip};
use rustsbi::spec::binary::SbiRet;
use rustsbi::{HartMask, RustSBI};

use oreboot_arch::riscv64::xuantie::get_mtime_compare_reg;
use util::mmio::{read32, write32, write64le};

#[derive(RustSBI)]
pub struct PlatSbi {
    //  ipi: Ipi,
    //  reset: Reset,
    timer: Timer,
}

pub fn init() -> PlatSbi {
    oreboot_arch::riscv64::xuantie::init_plic();
    init_pmp();
    PlatSbi {
        // ipi: Ipi,
        // reset: Reset,
        timer: Timer,
    }
}

const DRAM_BASE: usize = 0x00000000;
const PAYLOAD_BASE: usize = DRAM_BASE + 0x0020_0000;
const END: usize = 0x0000_00ff_ffff_ffff;

fn init_pmp() {
    use riscv::register::*;
    let cfg = 0x0000_0000_0f0f_090f;
    pmpcfg0::write(cfg);
    pmpcfg2::write(0); // nothing active here
    pmpaddr0::write(DRAM_BASE >> 2);
    pmpaddr1::write(PAYLOAD_BASE >> 2);
    pmpaddr2::write(END >> 2);
}

// XuanTie specific second mapping for S-mode
const STIME_OFFSET: usize = 0xFFF8;

const DEBUG: bool = false;

struct Timer;
impl rustsbi::Timer for Timer {
    fn set_timer(&self, value: u64) {
        if DEBUG {
            println!("[SBI] set timer: {value:016x}");
        }
        // Clear any pending timer
        unsafe { mip::clear_stimer() };

        // Set new value for this hart
        let hartid = mhartid::read();
        let mtime_cmp = get_mtime_compare_reg() + 4 * hartid;
        write64le(mtime_cmp, value);

        // Reenable the interrupt
        unsafe { mie::set_mtimer() }
    }
}
