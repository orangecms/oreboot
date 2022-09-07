#![feature(naked_functions, asm_sym, asm_const)]
#![no_std]
#![no_main]

mod init;
use core::arch::global_asm;
use core::{arch::asm, panic::PanicInfo};
use init::{clock_init, iopad_init, rstgen_init, syscon_init, uart_init, uart_write};

const STACK_SIZE: usize = 1 * 1024; // 1KiB

const UART3_BASE: u32 = 0x1244_0000;

use core::ptr::{read_volatile, write_volatile};

#[link_section = ".bss.uninit"]
static mut BT0_STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

global_asm!(include_str!("../start.S"));

/// Set up stack and jump to executable code.
///
/// # Safety
///
/// Naked function.
#[naked]
#[export_name = "start"]
#[link_section = ".text.entry"]
pub unsafe extern "C" fn start() -> ! {
    asm!(
    "li t4, 0x47",
    "li t5, 0x12440000",
    "sw t4, 0(t5)",

    "csrwi 0x7c1, 0",

        "csrw   mie, zero",
        "csrw   mstatus, zero",
        // 2. initialize programming language runtime
        // clear bss segment
        "la     t0, sbss",
        "la     t1, ebss",
        "1:",
        "bgeu   t0, t1, 1f",
        "sd     x0, 0(t0)",
        "addi   t0, t0, 4",
        "j      1b",
        "1:",
        // 3. prepare stack
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        // "j _debug",
        "call   {main}",
        stack      =   sym BT0_STACK,
        stack_size = const STACK_SIZE,
        main       =   sym main,
        options(noreturn)
    )
}

fn main() {
    clock_init();
    // for illegal instruction exception
    crate::init::_SET_SYSCON_REG_register50_SCFG_funcshare_pad_ctrl_18(0x00c000c0);
    rstgen_init();
    iopad_init();
    uart_init();
    syscon_init();
    loop {
        uart_write('o');
        uart_write('r');
        uart_write('e');
        uart_write('b');
        uart_write('o');
        uart_write('o');
        uart_write('t');
        uart_write(' ');
        uart_write('🦀');
        uart_write('\r');
        uart_write('\n');
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    // TODO: implement println!
    /*
    if let Some(location) = info.location() {
        println!("panic in '{}' line {}", location.file(), location.line(),);
    } else {
        println!("panic at unknown location");
    };
    */
    loop {
        core::hint::spin_loop();
    }
}
