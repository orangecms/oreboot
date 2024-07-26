use super::feature;
use super::runtime::{MachineTrap, Runtime, SupervisorContext};
use core::{
    arch::asm,
    ops::{Coroutine, CoroutineState},
    pin::Pin,
};
use log::{print, println};
use riscv::register::{
    mie, mip,
    scause::{Exception, Trap},
};
use rustsbi::spec::binary::SbiRet;
use sbi_spec::legacy::LEGACY_CONSOLE_PUTCHAR;

const ECALL_OREBOOT: usize = 0x0A02_3B00;
const ECALL_DCSR: usize = usize::from_be_bytes(*b"\0\0\0\0DCSR");
const ECALL_DHEX: usize = usize::from_be_bytes(*b"\0\0\0\0DHEX");
const ECALL_DUMP: usize = usize::from_be_bytes(*b"\0\0\0\0DUMP");
const ECALL_STAT: usize = usize::from_be_bytes(*b"\0\0\0\0STAT");
const ECALL_TRAP: usize = usize::from_be_bytes(*b"\0\0\0\0TRAP");
const EBREAK: u16 = 0x9002;

const DEBUG: bool = true;
const DEBUG_ECALL: bool = false;
const DEBUG_MTIMER: bool = false;
const DEBUG_EBREAK: bool = true;
const DEBUG_EMULATE: bool = false;
const DEBUG_ILLEGAL: bool = true;
const DEBUG_MISALIGNED: bool = true;

// Machine Time is a 64-bit register
const MTIME_OFFSET: usize = 0xbff8;

pub fn dump(addr: usize, size: usize) {
    let s = unsafe { core::slice::from_raw_parts(addr as *const u8, size) };
    for w in s.iter() {
        print!("{:02x}", w);
    }
    println!();
}

pub fn dump_block(addr: usize, size: usize, step_size: usize) {
    println!("[SBI] dump {size} bytes @{addr:x}");
    for b in (addr..addr + size).step_by(step_size) {
        dump(b, step_size);
    }
}

use riscv::register::medeleg;

fn ore_sbi(ctx: &SupervisorContext) -> SbiRet {
    let method = ctx.a6;
    match method {
        ECALL_STAT => {
            println!("[SBI] machine state: {ctx:#x?}");
            SbiRet { value: 0, error: 0 }
        }
        ECALL_TRAP => {
            println!("[SBI] undelegate traps; mepc: {:016x}", ctx.mepc);
            unsafe {
                medeleg::clear_instruction_misaligned();
                // medeleg::clear_instruction_fault();
                medeleg::clear_breakpoint();
                medeleg::clear_load_fault();
                medeleg::clear_load_misaligned();
                medeleg::clear_store_misaligned();
                medeleg::clear_store_fault();
                medeleg::clear_user_env_call();
                medeleg::clear_instruction_page_fault();
                medeleg::clear_load_page_fault();
                medeleg::clear_store_page_fault();
            }
            SbiRet { value: 0, error: 0 }
        }
        ECALL_DUMP => {
            let base = ctx.a0;
            let size = ctx.a1;
            dump_block(base, size, 0x20);
            println!("[SBI] mepc: {:016x}", ctx.mepc);
            SbiRet { value: 0, error: 0 }
        }
        ECALL_DHEX => {
            let val = ctx.a0;
            println!("[SBI] dump hex: {val:016x}");
            SbiRet { value: 0, error: 0 }
        }
        ECALL_DCSR => {
            let mut val = 0;
            let mut err = 0;
            let csr = ctx.a0;
            if DEBUG {
                println!("[SBI] read CSR {:x}", csr);
            }
            match csr {
                0x7c0 => unsafe {
                    asm!("csrr {0}, 0x7c0", out(reg) val);
                },
                0x7c1 => unsafe {
                    asm!("csrr {0}, 0x7c1", out(reg) val);
                },
                0x7c2 => unsafe {
                    asm!("csrr {0}, 0x7c2", out(reg) val);
                },
                0x7c5 => unsafe {
                    asm!("csrr {0}, 0x7c5", out(reg) val);
                },
                _ => {
                    err = 1;
                }
            }
            if DEBUG {
                println!("[SBI] CSR {csr:x} is {val:08x}, err {err:x}");
            }
            SbiRet {
                value: val,
                error: err,
            }
        }
        _ => SbiRet { value: 0, error: 1 },
    }
}

// TODO: Check newer specs on how this should work with method.
fn putchar(_method: usize, args: [usize; 6]) -> SbiRet {
    let char = args[0] as u8 as char;
    print!("{char}");
    SbiRet { value: 0, error: 0 }
}

fn print_ecall_context(ctx: &mut SupervisorContext) {
    if DEBUG && DEBUG_ECALL {
        println!(
            "[SBI] ecall a6: {:x}, a7: {:x}, a0-a5: {:x} {:x} {:x} {:x} {:x} {:x}",
            ctx.a6, ctx.a7, ctx.a0, ctx.a1, ctx.a2, ctx.a3, ctx.a4, ctx.a5,
        );
    }
}

pub fn execute_supervisor(
    supervisor_mepc: usize,
    hartid: usize,
    dtb_addr: usize,
    clint_base: usize,
) -> (usize, usize) {
    println!(
        "[SBI] Enter supervisor on hart {hartid} at {:x} with DTB from {:x}",
        supervisor_mepc, dtb_addr
    );
    let mut rt = Runtime::new_sbi_supervisor(supervisor_mepc, hartid, dtb_addr);
    let mtime: usize = clint_base + MTIME_OFFSET;
    println!("[SBI] Enter loop...");
    loop {
        // NOTE: `resume()` drops into S-mode by calling `mret` (asm) eventually
        match Pin::new(&mut rt).resume(()) {
            CoroutineState::Yielded(MachineTrap::SbiCall()) => {
                let ctx = rt.context_mut();
                // specific for 1.9.1; see document for details
                feature::preprocess_supervisor_external(ctx);
                let param = [ctx.a0, ctx.a1, ctx.a2, ctx.a3, ctx.a4, ctx.a5];
                let ans = match ctx.a7 {
                    ECALL_OREBOOT => ore_sbi(ctx),
                    LEGACY_CONSOLE_PUTCHAR => putchar(ctx.a6, param),
                    _ => {
                        print_ecall_context(ctx);
                        rustsbi::ecall(ctx.a7, ctx.a6, param)
                    }
                };
                if ans.error & (0xFFFFFFFF << 32) == 0x114514 << 32 {
                    // magic value to exit execution loop
                    break (ans.error & 0xFFFFFFFF, ans.value);
                }
                ctx.a0 = ans.error;
                ctx.a1 = ans.value;
                ctx.mepc = ctx.mepc.wrapping_add(4);
            }
            CoroutineState::Yielded(MachineTrap::InstructionFault()) => {
                let ctx = rt.context_mut();
                unsafe {
                    if feature::should_transfer_trap(ctx) {
                        let t = Trap::Exception(Exception::InstructionFault);
                        feature::do_transfer_trap(ctx, t)
                    } else {
                        // TODO: print address
                        panic!("[SBI] Instruction fault {ctx:#04X?}")
                    }
                }
            }
            CoroutineState::Yielded(MachineTrap::LoadFault()) => {
                let ctx = rt.context_mut();
                let addr = unsafe { get_vaddr_u32(ctx.mepc) } as usize;
                // TODO: print address
                panic!("[SBI] Load fault {addr:016x} {ctx:#04X?}")
            }
            CoroutineState::Yielded(MachineTrap::StoreFault()) => {
                let ctx = rt.context_mut();
                let addr = unsafe { get_vaddr_u32(ctx.mepc) } as usize;
                // TODO: print address
                panic!("[SBI] Store fault {addr:016x} {ctx:#04X?}")
            }
            CoroutineState::Yielded(MachineTrap::IllegalInstruction()) => {
                let ctx = rt.context_mut();
                let ins = unsafe { get_vaddr_u32(ctx.mepc) } as usize;
                // NOTE: Not all instructions are 32 bit
                if ins as u16 == EBREAK {
                    // dump context on breakpoints for debugging
                    // TODO: how would we allow for "real" debugging?
                    if DEBUG_EBREAK {
                        panic!("[SBI] Take an EBREAK! {ctx:#04X?}");
                    }
                    // skip instruction; this will likely cause the OS to crash
                    // use DEBUG to get actual information
                    ctx.mepc = ctx.mepc.wrapping_add(2);
                } else if !emulate_instruction(ctx, ins, mtime) {
                    if DEBUG_ILLEGAL {
                        println!("[SBI] Illegal instruction {ins:08x} not emulated {ctx:#04X?}");
                    }
                    unsafe {
                        if feature::should_transfer_trap(ctx) {
                            let t = Trap::Exception(Exception::IllegalInstruction);
                            feature::do_transfer_trap(ctx, t)
                        } else {
                            println!("[SBI] Na na na! {ctx:#04X?}");
                            fail_illegal_instruction(ctx, ins)
                        }
                    }
                }
            }
            CoroutineState::Yielded(MachineTrap::MachineTimer()) => {
                if DEBUG && DEBUG_MTIMER {
                    println!("[SBI] M-timer interrupt");
                }
                // NOTE: The ECALL handler enables the interrupt.
                unsafe { mie::clear_mtimer() }
                // Yeet timer interrupt pending to signal interrupt to S-mode
                // for this hart.
                unsafe { mip::set_stimer() }
            }
            CoroutineState::Complete(()) => unreachable!(),
        }
    }
}

#[inline]
unsafe fn get_vaddr_u32(vaddr: usize) -> u32 {
    get_vaddr_u16(vaddr) as u32 | ((get_vaddr_u16(vaddr.wrapping_add(2)) as u32) << 16)
}

#[inline]
#[allow(asm_sub_register)]
unsafe fn get_vaddr_u16(vaddr: usize) -> u16 {
    let mut ans: u16;
    asm!("
        li      {2}, (1 << 17)
        csrrs   {2}, mstatus, {2}
        lhu     {0}, 0({1})
        csrw    mstatus, {2}
    ", out(reg) ans, in(reg) vaddr, out(reg) _);
    ans
}

fn emulate_instruction(ctx: &mut SupervisorContext, ins: usize, mtimer: usize) -> bool {
    if DEBUG && DEBUG_EMULATE {
        println!("[SBI] Emulating instruction {ins:08x}, {ctx:#04X?}");
    }
    if feature::emulate_rdtime(ctx, ins, mtimer) {
        return true;
    }
    if feature::emulate_sfence_vma(ctx, ins) {
        return true;
    }
    false
}

// Real illegal instruction happening in M-mode
fn fail_illegal_instruction(ctx: &mut SupervisorContext, ins: usize) -> ! {
    let mepc = ctx.mepc;
    panic!("[SBI] invalid instruction from M-mode, mepc: {mepc:016x?}, instruction: {ins:016x?}, context: {ctx:016x?}");
}
