#![no_std]
#![no_main]
// TODO: remove when done debugging crap
#![allow(unused)]

#[macro_use]
extern crate log;

use core::{
    arch::{asm, naked_asm},
    intrinsics::transmute,
    panic::PanicInfo,
    ptr::slice_from_raw_parts,
};
use embedded_hal_nb::serial::Write;
use riscv::register::{marchid, mhartid, mimpid, mvendorid};
use rustsbi::RustSBI;

mod uart;
use uart::JH71XXSerial;

pub type EntryPoint = unsafe extern "C" fn(r0: usize, dtb: usize);

const STACK_SIZE: usize = 4 * 1024; // 4KiB

#[link_section = ".bss.uninit"]
static mut STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

/// Set up stack and jump to executable code.
///
/// # Safety
///
/// Naked function.
#[unsafe(naked)]
#[export_name = "_start"]
#[link_section = ".text.entry"]
#[allow(named_asm_labels)]
pub unsafe extern "C" fn start() -> ! {
    naked_asm!(
        /*
        "0:",
        "li t4, 0x43",
        "li t5, 0x12440000",
        "sw t4, 0(t5)",
        "j 0b", // debug: CCCCCCCCCCC
        */
        "csrw   mtvec, t0",
        "csrw   mie, zero",
        "csrw   mstatus, zero",
        // suspend non-boot hart
        "li     a1, 0",
        "csrr   a0, mhartid",
        "bne    a0, a1, .nonboothart",
        // clear bss segment
        "la     t0, sbss",
        "la     t1, ebss",
        "1:",
        "bgeu   t0, t1, 1f",
        "sd     x0, 0(t0)",
        "addi   t0, t0, 4",
        "j      1b",
        "1:",
        // prepare stack
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        "j .boothart",
        // wait for multihart to get back into the game
        ".nonboothart:",
        "csrw   mie, 8", // 1 << 3
        "wfi",
        "call   {payload}",
        ".boothart:",
        "call   {main}",
        stack      =   sym STACK,
        stack_size = const STACK_SIZE,
        payload    =   sym exec_payload,
        main       =   sym main
    )
}

fn init_logger(s: JH71XXSerial) {
    unsafe {
        static mut SERIAL: Option<JH71XXSerial> = None;
        SERIAL.replace(s);
        log::init(SERIAL.as_mut().unwrap());
    }
}

const DTB_ADDR: usize = 0x0;
const LOAD_ADDR: usize = 0x0;

fn exec_payload() {
    // TODO: if SBI ... else ...
    let hart_id = mhartid::read();
    unsafe {
        // jump to payload
        let f = transmute::<usize, EntryPoint>(LOAD_ADDR);
        asm!("fence.i");
        f(hart_id, DTB_ADDR);
    }
}

fn main() {
    let serial = JH71XXSerial::new();
    init_logger(serial);
    println!("oreboot 🦀");
    exec_payload();
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
