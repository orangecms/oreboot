#![feature(naked_functions, asm_sym, asm_const)]
#![no_std]
#![no_main]

mod init;
use core::{
    arch::{asm, global_asm},
    panic::PanicInfo,
};
use init::{clock_init, gmac_init, iopad_init, rstgen_init, uart_init, uart_write};
use riscv;

global_asm!(include_str!("../start.S"));

const STACK_SIZE: usize = 4 * 1024; // 4KiB

#[link_section = ".bss.uninit"]
static mut BT0_STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

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
        // "0:",
        // "li t4, 0x43",
        // "li t5, 0x12440000",
        // "sw t4, 0(t5)",
        // "j 0b", // debug: CCCCCCCCCCC
        // Clear feature disable CSR
        "csrwi  0x7c1, 0",

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

fn nop() {
    unsafe { riscv::asm::nop() }
}

fn hello() {
    uart_write('o');
    uart_write('r');
    uart_write('e');
    uart_write('b');
    uart_write('o');
    uart_write('o');
    uart_write('t');
    uart_write(' ');
    // uart_write('🦀');
    uart_write(0xf0 as char);
    uart_write(0x9f as char);
    uart_write(0xa6 as char);
    uart_write(0x80 as char);
    uart_write('\r');
    uart_write('\n');
}

fn main() {
    clock_init();
    // for illegal instruction exception
    crate::init::syscon_func_18(0x00c000c0);
    rstgen_init();

    // enable core (?)
    crate::init::syscon_core1_en(1);

    // move UART to other header
    crate::init::syscon_io_padshare_sel(6);
    iopad_init();
    // NOTE: In mask ROM mode, the UART is already set up for 9600 baud
    // We reconfigure it to 115200, but put it on the other header so that you
    // can use both headers with the respective different baud rates.
    uart_init();

    gmac_init();

    hello();

    // TODO: continue with DRAM init

    // how secondBoot does it
    /*
    unsafe {
        write_volatile(0x1800_0000 as *mut u32, 0x1801_fffc);
        /* restore hart1 from bootrom */
        write_volatile(0x0000_0001 as *mut u32, 0x0200_0004);
    }
    */

    unsafe { riscv::asm::wfi() }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    uart_write('P');
    uart_write('A');
    uart_write('N');
    uart_write('I');
    uart_write('C');
    uart_write('!');
    uart_write('\r');
    uart_write('\n');
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
