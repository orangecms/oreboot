use super::super::runtime::SupervisorContext;
use log::println;
use riscv::register::{
    mstatus::{self, MPP, SPP},
    mtval, scause, sepc, stval, stvec,
};

pub unsafe fn should_transfer_trap(ctx: &mut SupervisorContext) -> bool {
    ctx.mstatus.mpp() != MPP::Machine
}

pub unsafe fn do_transfer_trap<
    I: riscv::CoreInterruptNumber + core::fmt::Debug,
    E: riscv::ExceptionNumber + core::fmt::Debug,
>(
    ctx: &mut SupervisorContext,
    cause: scause::Trap<I, E>,
) {
    // The reason for setting S-layer exception is: illegal instruction
    scause::set(cause);
    // The instruction is stored in mtval.
    let ins = mtval::read();
    println!("[SBI] It's a trap! SCAUSE: {cause:04x?}");
    println!("[SBI] INSTRUCTION: 0x{ins:04x?}");
    // println!("[SBI] STATE\r  {ctx:#04X?}");
    stval::write(ins);
    // Fill in the address that S-mode needs to return to, the mepc here.
    // Will be overwritten by the subsequent code.
    sepc::write(ctx.mepc);
    mstatus::set_mpp(MPP::Supervisor);
    mstatus::set_spp(SPP::Supervisor);
    // Set the interrupt bit.
    if mstatus::read().sie() {
        mstatus::set_spie()
    }
    mstatus::clear_sie();
    // mstatus::set_sum();
    ctx.mstatus = mstatus::read();
    // Set the return address and return to S-mode.
    // Note that regardless of whether it is in Direct or Vectored mode, the
    // vector offset of all exceptions is 0, and there is no need to process
    // the interrupt vector, just jump to the entry address.
    ctx.mepc = stvec::read().address();
}
