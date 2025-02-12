use super::super::runtime::SupervisorContext;
use log::println;
use riscv::register::{cycle, time};

use util::mmio::read64le;

const RDINS_MASK: usize = 0xFFFF_F07F;
const RDTIME_INST: usize = 0xC010_2073;
const RDCYCLE_INST: usize = 0xC000_2073;

#[inline]
pub fn emulate_rdtime(ctx: &mut SupervisorContext, ins: usize, mtime_reg: Option<usize>) -> bool {
    match ins & RDINS_MASK {
        RDCYCLE_INST => {
            let rd = ((ins >> 7) & 0b1_1111) as u8;
            let cycle_usize = cycle::read64() as usize;
            set_register_xi(ctx, rd, cycle_usize);
            // advance S-mode to next instruction
            ctx.mepc = ctx.mepc.wrapping_add(4);
            true
        }
        RDTIME_INST => {
            // rdtime is actually a csrrw instruction
            let rd = ((ins >> 7) & 0b1_1111) as u8;
            let time_usize = match mtime_reg {
                Some(r) => read64le(r),
                None => time::read64(),
            } as usize;
            set_register_xi(ctx, reg, mtime);
            // advance S-mode to next instruction
            ctx.mepc = ctx.mepc.wrapping_add(4);
            true
        }
        _ => false, // is not an rdXXX instruction
    }
}

#[inline]
fn set_register_xi(ctx: &mut SupervisorContext, i: u8, data: usize) {
    let registers = unsafe { &mut *(ctx as *mut _ as *mut [usize; 31]) };
    assert!(i <= 31, "i should be valid register target");
    if i == 0 {
        // x0, don't modify
        return;
    }
    registers[(i - 1) as usize] = data;
}
