#![no_std]
#![no_main]
#![feature(once_cell_get_mut)]

use core::arch::naked_asm;
use core::{arch::asm, panic::PanicInfo};

use embedded_hal_nb::serial::Write;

#[macro_use]
extern crate log;
mod mem_map;
mod uart;

const STACK_SIZE: usize = 1 * 1024;

#[link_section = ".bss.uninit"]
static mut STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

/// Save address and jump to reset.
///
/// # Safety
///
/// Naked function.
#[unsafe(naked)]
#[export_name = "start"]
#[link_section = ".text.entry"]
pub unsafe extern "C" fn start() -> ! {
    naked_asm!(
        "2:",          //
        "adr  r9, 2b", //
        "bl   {reset}",
        reset = sym reset
    )
}

/// Set up the stack and jump to main.
///
/// # Safety
///
/// Trust me, I'm an engineer.
#[no_mangle]
unsafe extern "C" fn reset() {
    // stack setup
    asm!("mov sp, {}", in(reg) &raw const STACK);
    asm!(
        "ldr  r1, ={stack_size}",
        "add  sp, r1",
        "bl   {main}",
        stack_size = const STACK_SIZE,
        main       = sym main
    );
}

fn sleep(t: usize) {
    for _ in 0..t {
        core::hint::spin_loop();
    }
}

fn init_logger(s: uart::XmSerial) {
    // This is the new method that also compiles in Rust 2024.
    if false {
        use core::{cell::OnceCell, ptr::addr_of_mut};
        static mut SERIAL: OnceCell<uart::XmSerial> = OnceCell::new();
        unsafe {
            log::init((*addr_of_mut!(SERIAL)).get_mut_or_init(|| s));
        }
    } else {
        static mut SERIAL: Option<uart::XmSerial> = None;
        unsafe {
            SERIAL.replace(s);
            log::init(SERIAL.as_mut().unwrap());
        }
    }
}

#[no_mangle]
pub extern "C" fn main() -> ! {
    let mut ini_pc: usize = 0;
    unsafe { asm!("mov {}, r9", out(reg) ini_pc) };
    let mut ini_sp: usize = 0;
    unsafe { asm!("mov {}, sp", out(reg) ini_sp) };

    let mut serial = uart::XmSerial::new();

    init_logger(serial);
    println!("oreboot 🦀");
    println!("  program counter (PC): {ini_pc:016x}");
    println!("    stack pointer (SP): {ini_sp:016x}");

    loop {
        sleep(100_000_000);
        print!(".");
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
