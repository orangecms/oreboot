#![feature(naked_functions, asm_sym, asm_const)]
#![no_std]
#![no_main]

use core::{
    arch::{asm, global_asm},
    panic::PanicInfo,
    ptr::slice_from_raw_parts,
};
use embedded_hal::serial::nb::Write;
use init::{clock_init, gmac_init, iopad_init, rstgen_init, uart_write};
use riscv;

mod init;
#[macro_use]
mod log;

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

// type `Serial` is declared outside this crate, avoid orphan rule
struct Wrap<T>(core::cell::UnsafeCell<T>);

type S = Wrap<init::Serial>;

impl core::fmt::Write for S {
    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        for byte in s.as_bytes() {
            while let Err(nb::Error::WouldBlock) = unsafe { (*self.0.get()).write(*byte) } {}
        }
        Ok(())
    }
}

const QSPI_CSR: usize = 0x1186_0000;
const QSPI_READ_CMD: usize = QSPI_CSR + 0x0004;
const SPI_FLASH_READ_CMD: u32 = 0x0003;

use core::ptr::{read_volatile, write_volatile};
fn spi_flash_init() {
    unsafe { write_volatile(QSPI_READ_CMD as *mut u32, SPI_FLASH_READ_CMD) };
}

fn dump(addr: usize, length: usize) {
    let slice = slice_from_raw_parts(addr as *const u8, length);
    for i in 0..length {
        print!("{:x}", unsafe { &*slice }[i]);
    }
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
    let serial = init::Serial::new();
    gmac_init();

    for _ in 0..0xffffff {
        unsafe { riscv::asm::nop() }
    }

    log::set_logger(serial);
    println!("oreboot 🦀");

    // TODO: continue with DRAM init

    // how secondBoot does it
    /*
    unsafe {
        write_volatile(0x1800_0000 as *mut u32, 0x1801_fffc);
        /* restore hart1 from bootrom */
        write_volatile(0x0000_0001 as *mut u32, 0x0200_0004);
    }
    */
    // Now, dump a bunch of memory ranges to check on
    // NOTE: When run via mask ROM from SRAM, we do not see the SPI flash
    // which would be mapped to memory on regular boot, only get ffffffffff.

    // NOTE: First SRAM: We are here!
    println!("\n\nRead from 0x1800_0000:");
    dump(0x1800_0000, 32);

    spi_flash_init();

    // NOTE: Offset 64K in stock firmware is DRAM init
    println!("\n\nRead from 0x2001_0000:");
    dump(0x2001_0000, 32);

    // Copy the actual DRAM init blob (starting at byte 4) to second SRAM
    let dram_blob_size = peek32(0x2001_0000) as u32;
    for addr in (0usize..dram_blob_size as usize).step_by(4) {
        // uart.pwrite(b".", 0).unwrap();
        let d = peek32(0x2001_0004 + addr as u32);
        poke32(0x1808_0000 + addr as u32, d);
    }

    use core::intrinsics::transmute;
    // call ddr
    pub type EntryPoint = unsafe extern "C" fn(r0: usize, dtb: usize);

    // this is SRAM space
    unsafe {
        let f = transmute::<usize, EntryPoint>(0x1808_0000);
        // NOTE: first argument would be the hart ID, so why 1 and not 0?
        f(1, 0x1804_0000);
    }
    unsafe { riscv::asm::wfi() }
}

fn peek32(a: u32) -> u32 {
    let y = a as *const u32;
    unsafe { read_volatile(y) }
}

fn poke32(a: u32, v: u32) {
    let y = a as *mut u32;
    unsafe {
        write_volatile(y, v);
    }
}

fn bye() {
    uart_write('B');
    uart_write('y');
    uart_write('e');
    uart_write('!');
    uart_write('\r');
    uart_write('\n');
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    bye();
    if let Some(location) = info.location() {
        println!("panic in '{}' line {}", location.file(), location.line(),);
    } else {
        println!("panic at unknown location");
    };

    loop {
        core::hint::spin_loop();
    }
}
