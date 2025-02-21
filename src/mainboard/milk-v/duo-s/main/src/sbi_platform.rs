use core::arch::asm;
use riscv::register::{self as reg, mhartid, mie, mip};
use rustsbi::{HartMask, RustSBI, SbiRet};

use oreboot_arch::riscv64::xuantie;
use util::{read64x, write64x};

#[derive(RustSBI)]
pub struct PlatSbi {
    // ipi: Ipi,
    // reset: Reset,
    // rfence: Rfence,
    timer: Timer,
}

pub fn init() -> PlatSbi {
    xuantie::init_plic();
    init_pmp();
    PlatSbi {
        // ipi: Ipi,
        // reset: Reset,
        // rfence: Rfence,
        timer: Timer,
    }
}

const DRAM_BASE: usize = 0x8000_0000;
const PAYLOAD_BASE: usize = DRAM_BASE + 0x20_0000;
const END: usize = 0x00ff_ffff_ffff_ffff;

// see privileged spec v1.10 p44 ff
// https://riscv.org/wp-content/uploads/2017/05/riscv-privileged-v1.10.pdf
fn init_pmp() {
    reg::pmpaddr0::write(0x0);
    reg::pmpaddr1::write(DRAM_BASE >> 2);
    reg::pmpaddr2::write(PAYLOAD_BASE >> 2);
    reg::pmpaddr3::write(END >> 2);
    // TODO
    // A: address matching; 0x01 means TOR (Top of range)
    // [ L  x  x  A1   A0  X  W  R ]
    // let cfg = 0x0000_0000_0f08_0f0f;
    let cfg = 0x0000_0000_0000_0000_0000_0000_0f0f_0f0f;
    // pmpaddr0-1 is read-only
    let cfg = 0x0000_0000_0000_0000_0000_0000_0f0f_090f;
    // reg::pmpcfg0::set_pmp(0, range, permission, locked);
    reg::pmpcfg0::write(cfg);
    reg::pmpcfg2::write(0); // nothing active here
}

struct Ipi;
impl rustsbi::Ipi for Ipi {
    fn send_ipi(&self, hart_mask: HartMask) -> SbiRet {
        // TODO
        SbiRet::success(0)
    }
}

struct Rfence;
impl rustsbi::Fence for Rfence {
    fn remote_fence_i(&self, hart_mask: HartMask) -> SbiRet {
        // TODO
        SbiRet::success(0)
    }

    fn remote_sfence_vma_asid(
        &self,
        hart_mask: HartMask,
        start_addr: usize,
        size: usize,
        asid: usize,
    ) -> SbiRet {
        // TODO
        SbiRet::success(0)
    }

    fn remote_sfence_vma(&self, hart_mask: HartMask, start_addr: usize, size: usize) -> SbiRet {
        // TODO
        SbiRet::success(0)
    }
}

const DEBUG: bool = false;

struct Timer;
impl rustsbi::Timer for Timer {
    fn set_timer(&self, value: u64) {
        if DEBUG {
            let t = riscv::register::time::read();
            println!("[SBI] current time: {t:016x} {t:020}");
            println!("[SBI] set timer to: {value:016x} {value:020}");
        }
        // Clear any pending timer
        unsafe { mip::clear_stimer() };

        // Set new value for this hart
        let hartid = mhartid::read();
        let mtime_cmp = xuantie::get_mtime_compare_reg() + 4 * hartid;
        if DEBUG {
            let mtime_val = read64x(mtime_cmp);
            println!("hart: {hartid}");
            println!("compare register: {mtime_cmp:016x}");
            println!("current value:    {mtime_val:016x}");
        }
        write64x(mtime_cmp, value);
        if DEBUG {
            let mtime_val = read64x(mtime_cmp);
            println!("new value:        {mtime_val:016x}");
        }

        // Reenable the interrupt
        unsafe { mie::set_mtimer() }
    }
}

// magic value to exit execution loop
const RESET_MAGIC: usize = 0x114514 << 32;

struct Reset;
impl rustsbi::Reset for Reset {
    fn system_reset(&self, reset_type: u32, reset_reason: u32) -> SbiRet {
        SbiRet {
            error: reset_type as usize | RESET_MAGIC,
            value: reset_reason as usize,
        }
    }
}
