use super::super::runtime::SupervisorContext;
use log::println;
use riscv::register::{cycle, time};

const DEBUG: bool = true;
const DEBUG_RDCYCLE: bool = false;
const DEBUG_RDTIME: bool = false;

const RD_INST_MASK: usize = 0xFFFF_F07F;
const RDCYCLE_INST: usize = 0xC000_2073;
const RDTIME_INST: usize = 0xC010_2073;

// NOTE: Machine Time is a 64-bit value via high and low 32-bit registers.
const MTIME_OFFSET: usize = 0xbff8;

fn read32(reg: usize) -> u32 {
    unsafe { core::ptr::read_volatile(reg as *mut u32) }
}

fn read64(reg: usize) -> u64 {
    let l = read32(reg) as u64;
    let h = read32(reg + 4) as u64;
    (h << 32) | l
}

#[inline]
pub fn emulate_rdtime(ctx: &mut SupervisorContext, ins: usize, clint_base: usize) -> bool {
    // TODO: make a param / feature
    let has_rv_time = true;
    let mtime: usize = clint_base + MTIME_OFFSET;

    match ins & RD_INST_MASK {
        RDCYCLE_INST => {
            // examples:
            //  c0002573     rdcycle a0
            let reg = ((ins >> 7) & 0b1_1111) as u8;
            if DEBUG && DEBUG_RDCYCLE {
                println!("[rustsbi] rdcycle {ins:08x} ({reg})");
            }
            let cycle_usize = cycle::read64();
            set_register_xi(ctx, reg, cycle_usize);
            if DEBUG && DEBUG_RDCYCLE {
                println!("[rustsbi] rdcycle {cycle_usize:x}");
            }
            // skip current instruction, 4 bytes
            ctx.mepc = ctx.mepc.wrapping_add(4);
            true
        }
        RDTIME_INST => {
            // examples:
            //  c0102573     rdtime  a0    (reg = 10)
            //  c01027f3     rdtime  a5    (reg = 15)
            // rdtime is actually a csrrw instruction
            let reg = ((ins >> 7) & 0b1_1111) as u8;
            if DEBUG && DEBUG_RDTIME {
                println!("[rustsbi] rdtime {ins:08x} ({reg})");
            }
            // Some platforms support the time instruction natively.
            // For others, we need to read from the CLINT.
            let mtime = if has_rv_time {
                time::read64()
            } else {
                read64(mtime)
            };
            set_register_xi(ctx, reg, mtime);
            if DEBUG && DEBUG_RDTIME {
                println!("[rustsbi] rdtime: {mtime}");
            }
            // skip current instruction, 4 bytes
            ctx.mepc = ctx.mepc.wrapping_add(4);
            true
        }
        _ => false, // is not an rdXXX instruction
    }
}

#[inline]
fn set_register_xi(ctx: &mut SupervisorContext, i: u8, data: u64) {
    let registers = unsafe { &mut *(ctx as *mut _ as *mut [u64; 31]) };
    assert!(i <= 31, "i should be valid register target");
    if i == 0 {
        // x0, don't modify
        return;
    }
    registers[(i - 1) as usize] = data;
}
