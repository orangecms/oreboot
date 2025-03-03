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
};

use embedded_hal_nb::serial::Write;
use riscv::register::{marchid, mhartid, mimpid, mvendorid};

use util::mem::{dump, dump_block};
use util::mmio::{read32, write32};

mod mem_map;
mod sbi_platform;
mod uart;

const DEBUG: bool = false;

pub type EntryPoint = unsafe extern "C" fn();

const BOOT_HART_ID: usize = 0;

const STACK_SIZE: usize = 8 * 1024;

static PLATFORM: &str = "Canaan Kendryte K230D";
static VERSION: &str = env!("CARGO_PKG_VERSION");

#[link_section = ".bss.uninit"]
static mut BT0_STACK: [u8; STACK_SIZE] = [0; STACK_SIZE];

/// Set up stack and jump to executable code.
///
/// # Safety
///
/// Naked function.
#[unsafe(naked)]
#[export_name = "start"]
#[link_section = ".text.entry"]
#[allow(named_asm_labels)]
pub unsafe extern "C" fn start() -> ! {
    naked_asm!(
        "auipc  s4, 0",

        "csrw   mstatus, zero",
        "csrw   mie, zero",
        "ld     t0, {start}",
        "csrw   mtvec, t0",
        // 1. suspend non-boot hart
        "li     t1, {boothart}",
        "csrr   t0, mhartid",
        "bne    t0, t1, .nonboothart",
        // 2. prepare stack
        // NOTE: non-boot harts need no stack here, they skip this
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        "j      .boothart",
        // wait for multihart to get back into the game
        ".nonboothart:",
        // "csrw   mie, (1 << 3)",
        "wfi",
        "call   {payload}",

        ".boothart:",
        "call   {main}",
        boothart   = const BOOT_HART_ID,
        stack      = sym BT0_STACK,
        stack_size = const STACK_SIZE,
        payload    = sym exec_payload,
        main       = sym main,
        start      = sym start,
    )
}

static mut SERIAL: Option<uart::K230Serial> = None;

fn init_logger(s: uart::K230Serial) {
    unsafe {
        SERIAL.replace(s);
        if let Some(m) = SERIAL.as_mut() {
            log::init(m);
        }
    }
}

// TODO: move to SoC lib crate
const STC2_CFG: usize = mem_map::STC_BASE + 0x0020;
const STC3_CFG: usize = mem_map::STC_BASE + 0x0030;
// This enables the mtimer clock and is specific to the K230 SoC.
// It is apparently backed by the STC, which probably means System Time Clock.
// The manual says:
// > K230 provides 9 timers (six general timer and three stc timer).
// > Six general-purpose timers can be used as system clock of operating system
// > and used to count external sinal input. Stc0 timer can be used for
// > video-input/video-output and audio synchronization.
// > STC2 timer is used for cpu0 and STC3 timer is used for cpu1.
// see k230_linux_sdk
// buildroot-overlay/boot/uboot/u-boot-2022.10-overlay/arch/riscv/cpu/k230/cpu.c
// harts_early_init
fn enable_system_time_clock() {
    write32(STC2_CFG, 1);
    write32(STC3_CFG, 1);
}

fn init_csrs() {
    if DEBUG {
        oreboot_arch::riscv64::xuantie::dump_csrs();
    }
    oreboot_arch::riscv64::xuantie::init_csrs();
    if DEBUG {
        oreboot_arch::riscv64::xuantie::dump_csrs();
    }
}

#[no_mangle]
fn main() {
    let mut ini_pc: usize = 0;
    unsafe { asm!("mv {}, s4", out(reg) ini_pc) };

    let s = uart::K230Serial::new();
    init_logger(s);
    println!("oreboot 🦀 main");
    println!("initial program counter (PC) {ini_pc:016x}");
    oreboot_arch::riscv64::ids::print_ids();
    oreboot_arch::riscv64::xuantie::print_cpuid();

    exec_payload();
}

const PAYLOAD_ADDR: usize = mem_map::DRAM_BASE_ADDR + 0x0020_0000;
const PAYLOAD_SIZE: usize = 32 * 1024 * 1024;
const DTB_ADDR: usize = PAYLOAD_ADDR + PAYLOAD_SIZE;

const CONFIG_MSECCFG_MENVCFG: bool = false;

fn exec_payload() {
    let payload_addr = PAYLOAD_ADDR;
    let dtb_addr = DTB_ADDR;
    // NOTE: The system time clock _must_ be enabled for mtime to work.
    // We already enable it in bt0, but keep it here as well to ensure
    // that it won't get lost. Otherwise, we never get mtime interrupts.
    // TODO: maybe move the timer init somewhere else
    enable_system_time_clock();
    init_csrs();

    // TODO
    if CONFIG_MSECCFG_MENVCFG {
        unsafe {
            // mseccfg
            const MSECCFG_USEED: u64 = 1 << 9;
            const MSECCFG_SSEED: u64 = 1 << 8;
            let v = MSECCFG_SSEED | MSECCFG_USEED;
            asm!("csrw 0x747, {}", in(reg) v);
            // S-mode time compare enable
            const MENVCFG_STCE: u64 = 1 << 63;
            // counter delegation enable
            const MENVCFG_CDE: u64 = 1 << 60;
            let v = MENVCFG_STCE | MENVCFG_CDE;
            asm!("csrw 0x30a, {}", in(reg) v);
        }
    }

    if DEBUG {
        println!("Payload @ {payload_addr:08x}");
        dump_block(payload_addr, 0x50, 0x10);
        println!("DTB @ {dtb_addr:08x}");
        dump_block(dtb_addr, 0x50, 0x10);
    }

    let use_sbi = true;
    if use_sbi {
        use oreboot_arch::riscv64::sbi as ore_sbi;
        let sbi = sbi_platform::init();

        ore_sbi::runtime::init();
        ore_sbi::info::print_info(PLATFORM, VERSION);

        let hart_id = mhartid::read();
        riscv::asm::fence_i();
        riscv::asm::fence();
        let (reset_type, reset_reason) =
            ore_sbi::execute::execute_supervisor(sbi, payload_addr, hart_id, dtb_addr, None);
        println!("[oreboot] reset reason: {reset_reason}");
    } else {
        unsafe {
            let f: EntryPoint = transmute(payload_addr);
            riscv::asm::fence_i();
            riscv::asm::fence();
            f();
        }
    }
    unsafe { riscv::asm::wfi() }
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
