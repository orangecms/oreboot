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
    slice::from_raw_parts as slice_from,
};

use layoutflash::areas::{find_fdt, FdtIterator};
use util::mem::{dump, dump_block};
use util::mmio::read32;

mod axi_mon;
mod cv18xx;
mod ddr_bist;
mod ddr_ctrl;
mod ddr_phy;
mod ddr_pll;
mod dram;
mod efuse;
mod mem_map;
mod rom;
mod rtc;
mod uart;

pub type ExternFn0 = unsafe extern "C" fn() -> !;

const PRINT_LOG: bool = false;
const DUMP_MASK_ROM: bool = false;
const DRAM_TEST: bool = true;
const BOOT_MAIN: bool = true;
const USB_LOAD_PAYLOAD: bool = true;

// TODO: get from DTFS
const ORE_MAIN_SIZE: usize = 0x2_0000;
const ORE_MAIN_ADDR: usize = mem_map::DRAM_BASE;
const PAYLOAD_ADDR: usize = ORE_MAIN_ADDR + 0x0020_0000;
const PAYLOAD_SIZE: usize = 32 * 1024 * 1024;
const DTB_ADDR: usize = PAYLOAD_ADDR + PAYLOAD_SIZE;
const DTB_SIZE: usize = 256 * 1024;

const STACK_SIZE: usize = 512;

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
        "j      .forrealsiez", // 2 bytes with compact instruction
        ".byte 0",
        ".byte 0",
        ".word 0", // resvered
        ".word 0", // BL2 MSID
        ".word 0", // BL2 version
        ".word 0", // rest not documented
        ".word 0",
        ".word 0",
        ".word 0",
        ".forrealsiez:",
        // save program counter for early printing
        "auipc  s4, 0",
        // 1. clear processor states
        "csrw   mie, zero",
        "csrw   mip, zero",
        "csrw   mstatus, zero",
        // When a trap is hit early, jump back to start
        "ld     t0, {start}",
        "csrw   mtvec, t0",
        // 2. suspend non-boot hart
        "li     a1, 1",
        "csrr   a0, mhartid",
        "bne    a0, a1, .nonboothart",
        // 3. prepare stack
        // FIXME: each hart needs its own stack
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        // 4. jump to reset/next stage
        "j      .boothart",
        // wait for multihart to get back into the game
        ".nonboothart:",
        "j      .boothart",
        // enable interrupt
        "csrw   mie, 1 << 3",
        "wfi",
        "call   {next}",
        ".boothart:",
        "call   {reset}",
        stack      = sym STACK,
        stack_size = const STACK_SIZE,
        next    = sym next_stage,
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
pub unsafe extern "C" fn reset() {
    extern "C" {
        static mut _sbss: u8;
        static mut _ebss: u8;

        static mut _sdata: u8;
        static mut _edata: u8;
        static _sidata: u8;
    }

    if false {
        use core::ptr::{self, addr_of, addr_of_mut};
        // zero out BSS
        let sbss = addr_of_mut!(_sbss);
        let bss_size = addr_of!(_ebss) as usize - addr_of!(_sbss) as usize;
        ptr::write_bytes(sbss, 0, bss_size);
        // copy over data
        let sidata = addr_of!(_sidata);
        let sdata = addr_of_mut!(_sdata);
        let data_size = addr_of!(_edata) as usize - addr_of!(_sdata) as usize;
        ptr::copy_nonoverlapping(sidata, sdata, data_size);
    }

    // Call user entry point
    main();
}

use core::cell::OnceCell;
static mut SERIAL: OnceCell<uart::SGSerial> = OnceCell::new();

fn init_logger(s: uart::SGSerial) {
    unsafe {
        SERIAL.get_or_init(|| s);
        if let Some(m) = SERIAL.get_mut() {
            log::init(m);
        }
    }
}

// more funz
// Arm mask ROM runs in this other SRAM area
// _clear_or_smth(&DAT_0453_e580,0xf30);
// _fill_smth(0x0453_c000,&DAT_0441_3000,0x520);
/*
  _DAT_0453e540 = 0x00000605;
  _DAT_0453e548 = 0x20000605;
  _DAT_0453e550 = 0x40000701;
  _DAT_0453e558 = 0x60000701;
  _DAT_0453e560 = 0x80000701;
  _DAT_0453e568 = 0xa0000701;
  _DAT_0453e570 = 0xc0000701;
  _DAT_0453e578 = 0xe0000701;
*/

#[no_mangle]
fn main() {
    let mut ini_pc: usize = 0;
    unsafe { asm!("mv {}, s4", out(reg) ini_pc) };

    let s = uart::SGSerial::new();
    init_logger(s);
    // Some empty lines after mask ROM output, which has no line break before running our code
    println!();
    println!();
    println!("oreboot 🦀 bt0");
    println!("initial program counter (PC) {ini_pc:016x}");
    println!();
    oreboot_arch::riscv64::ids::print_ids();
    println!();
    let boot_src = rom::get_boot_src();
    let retry_count = rom::get_retry_count();
    println!("boot src: {boot_src}");
    println!("retries:  {retry_count}");
    println!();
    cv18xx::print_platform_state();
    cv18xx::print_boot_info();

    if PRINT_LOG {
        println!();
        cv18xx::print_boot_log();
        println!();
    }
    if DUMP_MASK_ROM {
        cv18xx::dump_mask_rom();
    }

    rtc::init();
    rtc::en();

    let start = riscv::register::time::read64();
    let (dram_vendor, ddr_rate) = cv18xx::get_dram_type();
    dram::init(ddr_rate, &dram::DramType::from(dram_vendor as u8));
    let time = riscv::register::time::read64() - start;
    println!("DRAM init done in {time}");

    // FIXME: DRAM on SG2002 is not stable and loses data :(
    if DRAM_TEST {
        util::mem::test(mem_map::DRAM_BASE, 0x20_0000);
    }

    // Load extra code

    let v = read32(cv18xx::AXI_SRAM_RTOS_BASE);
    // 0x0c85e985
    // CVI_RTOS_MAGIC_CODE 0x0ABC0DEF
    println!("RTOS base: 0x{v:08x}");

    // `make run` in main
    println!(
        ">> load main stage (max size: {} KB) over USB",
        ORE_MAIN_SIZE / 1024
    );
    rom::load_image(ORE_MAIN_ADDR, 0x0, ORE_MAIN_SIZE, 0);
    dump_block(ORE_MAIN_ADDR, 0x60, 0x20);

    if USB_LOAD_PAYLOAD {
        println!(
            ">> load payload (max size: {} MB) over USB",
            PAYLOAD_SIZE / 1024 / 1024
        );
        rom::load_image(PAYLOAD_ADDR, 0x0, PAYLOAD_SIZE, 0);
        dump_block(PAYLOAD_ADDR, 0x60, 0x20);

        println!(">> load DTB (max size: {} KB) over USB", DTB_SIZE / 1024);
        rom::load_image(DTB_ADDR, 0x0, DTB_SIZE, 0);
        dump_block(DTB_ADDR, 0x60, 0x20);
    }

    println!("[bt0] Jump to main stage @{ORE_MAIN_ADDR:08x}");

    if BOOT_MAIN {
        // RV64ACDFIMSUVX
        next_stage(ORE_MAIN_ADDR);
    } else {
        // RV64ACDFIMSUX
        cv18xx::exec_hartl(ORE_MAIN_ADDR);
    }
}

// jump to main stage
fn next_stage(addr: usize) {
    unsafe {
        let f: ExternFn0 = transmute(addr);
        asm!("fence.i");
        f();
    }

    println!("[bt0] Exit from main stage, resetting...");
    unsafe {
        reset();
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    if let Some(location) = info.location() {
        println!(
            "[bt0] panic in '{}' line {}",
            location.file(),
            location.line(),
        );
    } else {
        println!("[bt0] panic at unknown location");
    };
    let msg = info.message();
    println!("[bt0]   {msg}");
    loop {
        core::hint::spin_loop();
    }
}
