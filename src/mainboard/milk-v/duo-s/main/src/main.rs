#![feature(naked_functions)]
#![no_std]
#![no_main]
// TODO: remove when done debugging crap
#![allow(unused)]

#[macro_use]
extern crate log;

use core::{
    arch::{asm, naked_asm},
    mem::transmute,
    panic::PanicInfo,
    ptr::{self, addr_of, addr_of_mut},
    slice::from_raw_parts as slice_from,
};
use riscv::register::{marchid, mhartid, mimpid, mtvec, mvendorid};

use layoutflash::areas::{find_fdt, FdtIterator};
use util::write32;

mod sbi_platform;
mod uart;

pub type EntryPoint = unsafe extern "C" fn() -> !;

static PLATFORM: &str = "SG200x";
static VERSION: &str = env!("CARGO_PKG_VERSION");

const USE_SBI: bool = true;
const DRAM_BASE: usize = 0x8000_0000;
const LOAD_ADDR: usize = DRAM_BASE + 0x0020_0000;
const DTB_ADDR: usize = LOAD_ADDR + 16 * 1024 * 1024;

const DEBUG: bool = false;

const STACK_SIZE: usize = 2 * 1024;

#[link_section = ".bss.uninit"]
static mut STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

/// Set up stack and jump to executable code.
///
/// # Safety
///
/// Naked function.
#[naked]
#[export_name = "start"]
#[link_section = ".text.entry"]
#[allow(named_asm_labels)]
pub unsafe extern "C" fn start() -> ! {
    // starts with a 32 bytes header
    naked_asm!(
        // save program counter for early printing
        "auipc  s4, 0",
        // 1. clear processor states
        "csrw   mie, zero",
        "csrw   mip, zero",
        "csrw   mstatus, zero",
        // When a trap is hit early, jump back to start
        "ld     t0, {start}",
        "csrw   mtvec, t0",
        // 2. prepare stack
        // FIXME: each hart needs its own stack
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        // 4. jump to reset/payload
        "j      .boothart",
        // wait for multihart to get back into the game
        ".nonboothart:",
        "j      .boothart",
        "csrw   mie, 8", // 1 << 3
        "wfi",
        "call   {payload}",
        ".boothart:",
        "call   {reset}",
        stack      = sym STACK,
        stack_size = const STACK_SIZE,
        payload    = sym exec_payload,
        reset      = sym reset,
        start      = sym start
    )
}

/// Initialize RAM: Clear BSS and set up data.
/// See https://docs.rust-embedded.org/embedonomicon/main.html
///
/// # Safety
/// :shrug:
#[no_mangle]
pub unsafe extern "C" fn reset() -> ! {
    extern "C" {
        static mut _sbss: u8;
        static mut _ebss: u8;

        static mut _sdata: u8;
        static mut _edata: u8;
        static _sidata: u8;
    }
    // PROBLEMO
    if false {
        let bss_size = addr_of!(_ebss) as usize - addr_of!(_sbss) as usize;
        ptr::write_bytes(addr_of_mut!(_sbss), 0, bss_size);
    }
    write32(0x04140000, 0x30);
    if false {
        let data_size = addr_of!(_edata) as usize - addr_of!(_sdata) as usize;
        ptr::copy_nonoverlapping(addr_of!(_sidata), addr_of_mut!(_sdata), data_size);
    }
    write32(0x04140000, 0x31);
    // Call user entry point
    extern "Rust" {
        fn main() -> !;
    }
    main()
}

static mut SERIAL: Option<uart::SGSerial> = None;

fn init_logger(s: uart::SGSerial) {
    unsafe {
        SERIAL.replace(s);
        if let Some(m) = SERIAL.as_mut() {
            log::init(m);
        }
    }
}

#[no_mangle]
fn main() -> ! {
    write32(0x04140000, 0x32);

    let mut ini_pc: usize = 0;
    unsafe { asm!("mv {}, s4", out(reg) ini_pc) };

    let s = uart::SGSerial::new();
    init_logger(s);
    // WE GET HERE
    println!();
    println!("oreboot 🦀 main");
    println!("initial program counter (PC) {ini_pc:016x}");

    oreboot_arch::riscv64::ids::print_ids();
    oreboot_arch::riscv64::xuantie::print_cpuid();

    exec_payload()
}

fn exec_payload() -> ! {
    oreboot_arch::riscv64::xuantie::init_csrs();

    let payload_addr = LOAD_ADDR;
    let dtb_addr = DTB_ADDR;

    // TODO: make feature, see Nezha/D1
    if USE_SBI {
        use oreboot_arch::riscv64::sbi as ore_sbi;
        let sbi = sbi_platform::init();
        ore_sbi::runtime::init();
        ore_sbi::info::print_info(PLATFORM, VERSION);

        let hartid = mhartid::read();
        println!("[main] Launch SBI...");

        let (reset_type, reset_reason) =
            ore_sbi::execute::execute_supervisor(sbi, payload_addr, hartid, dtb_addr, None);
        println!("[main] oreboot: reset, type = {reset_type}, reason = {reset_reason}");
        unsafe { reset() }
    } else {
        unsafe {
            let f: EntryPoint = transmute(payload_addr);
            asm!("fence.i");
            f()
        }
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    if let Some(location) = info.location() {
        println!(
            "[main] panic in '{}' line {}",
            location.file(),
            location.line(),
        );
    } else {
        println!("[main] panic at unknown location");
    };
    let msg = info.message();
    println!("[main]   {msg}");
    loop {
        core::hint::spin_loop();
    }
}
