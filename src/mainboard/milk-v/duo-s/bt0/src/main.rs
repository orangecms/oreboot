#![feature(naked_functions, asm_const)]
#![feature(panic_info_message)]
#![no_std]
#![no_main]
// TODO: remove when done debugging crap
#![allow(unused)]

use embedded_hal_nb::serial::Write;

#[macro_use]
extern crate log;

use core::{
    arch::asm,
    mem::transmute,
    panic::PanicInfo,
    ptr::{self, addr_of, addr_of_mut},
    slice::from_raw_parts as slice_from,
};
use riscv::register::mhartid;
use riscv::register::{marchid, mimpid, mvendorid};

use layoutflash::areas::{find_fdt, FdtIterator};

mod ddr_phy;
mod dram;
mod efuse;
mod mem_map;
mod rom;
mod uart;
mod util;

use rom::MASK_ROM_BASE;
use util::{read32, write32};

pub type EntryPoint = unsafe extern "C" fn();

const DEBUG: bool = false;

const STACK_SIZE: usize = 512;

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
#[allow(named_asm_labels)]
pub unsafe extern "C" fn start() -> ! {
    // starts with a 32 bytes header
    asm!(
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
        "li     t0, 0x04140000",
        "li     t1, 0x42",
        "sw     t1, 0(t0)",
        // Clear feature disable CSR to '0' to turn on all features
        // TODO: do in Rust
        // "csrwi  0x7c1, 0",
        "csrw   mie, zero",
        "csrw   mstatus, zero",
        "ld     t0, {start}",
        "csrw   mtvec, t0",
        // 1. suspend non-boot hart
        // hart 0 is the S7 monitor core; 1-4 are U7 cores
        "li     a1, 1",
        "csrr   a0, mhartid",
        "bne    a0, a1, .nonboothart",
        // 2. prepare stack
        // FIXME: each hart needs its own stack
        "la     sp, {stack}",
        "li     t0, {stack_size}",
        "add    sp, sp, t0",
        "j      .boothart",
        // wait for multihart to get back into the game
        ".nonboothart:",
        "j      .boothart",
        "csrw   mie, 8", // 1 << 3
        "wfi",
        "csrw   mip, 0",
        "call   {payload}",
        ".boothart:",
        "call   {reset}",
        stack      = sym BT0_STACK,
        stack_size = const STACK_SIZE,
        payload    = sym exec_payload,
        reset      = sym reset,
        start      = sym start,
        options(noreturn)
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

    let bss_size = addr_of!(_ebss) as usize - addr_of!(_sbss) as usize;
    ptr::write_bytes(addr_of_mut!(_sbss), 0, bss_size);

    let data_size = addr_of!(_edata) as usize - addr_of!(_sdata) as usize;
    ptr::copy_nonoverlapping(addr_of!(_sidata), addr_of_mut!(_sdata), data_size);
    // Call user entry point
    main();
}

fn vendorid_to_name<'a>(vendorid: usize) -> &'a str {
    match vendorid {
        0x0489 => "SiFive",
        0x05b7 => "T-Head",
        _ => "unknown",
    }
}

// FIXME: This really depends on the vendor first!
fn impid_to_name<'a>(impid: usize) -> &'a str {
    match impid {
        0x0421_0427 => "21G1.02.00 / llama.02.00-general",
        _ => "unknown",
    }
}

/// Print RISC-V core information:
/// - vendor
/// - arch
/// - implementation
/// - hart ID
fn print_ids() {
    let vid = mvendorid::read().map(|r| r.bits()).unwrap_or(0);
    let aid = marchid::read().map(|r| r.bits()).unwrap_or(0);
    let iid = mimpid::read().map(|r| r.bits()).unwrap_or(0);
    // TODO: This prints 8000000000000007, but should be 80000007.
    // See U74-MC core complex manual 21G3.
    println!("RISC-V arch {aid:08x}");
    let vendor_name = vendorid_to_name(vid);
    println!("RISC-V core vendor: {vendor_name} (0x{vid:04x})");
    let imp_name = impid_to_name(iid);
    println!("RISC-V implementation: {imp_name} (0x{iid:08x})");
    let hart_id = mhartid::read();
    println!("RISC-V hart ID {hart_id}");
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

//// only for debugging
///define ATF_DBG_REG (BOOT_LOG_LEN_ADDR + BOOT_LOG_LEN_SIZE)
///define ATF_ERR_REG (ATF_DBG_REG + 0x04)
///define ATF_ERR_INFO0 (ATF_DBG_REG + 0x08)
///define CP_STATE_REG (ATF_DBG_REG + 0x0C)
///
///define ATF_ERR (((unsigned int __volatile__ *)ATF_ERR_REG)[0])

const CONF: usize = mem_map::TOP_BASE + 0x0004;
// used by mask ROM: _DAT_03000080 == 0x6526228c
const GP_REG0: usize = mem_map::TOP_BASE + 0x0080;
const GP_REG1: usize = mem_map::TOP_BASE + 0x0084;
const ATF_STATE: usize = GP_REG1;

const ATF_STATE_INIT: u32 = 0xb100_0000;

const ATF_STATE_SXX0: u32 = 0xb100_1020;
const ATF_STATE_SXX1: u32 = 0xb100_1022;

// mask ROM writes this during bootup
const ATF_STATE_MASK_ROM: u32 = 0xb100_f000;

const ATF_STATE_TXX1: u32 = 0xb100_f001;
const ATF_STATE_TXX2: u32 = 0xb100_f002;

const ATF_STATE_FLASH_INIT_START: u32 = 0xb100_f005;
const ATF_STATE_FLASH_INIT_END: u32 = 0xb100_f006;

// CV1800B mask ROM
const ATF_STATE_XX1: u32 = 0xb100_f00f;
const ATF_STATE_XX2: u32 = 0xb100_f801;

// NOTE: Vendor calls bt0 "bl2" (boot loader 2? after mask ROM...)
// NOTE: ATF is probably meant to resemble Arm Trusted Firmware.
const ATF_STATE_BL2_MAIN: u32 = 0xB200_F000;

const ATF_STATE_RESET_WAIT: u32 = 0xBE00_3001;
const ATF_STATE_BEFORE_ERROR_WAIT: u32 = 0xbe00_3002;
const ATF_STATE_WD_XX: u32 = 0xc000_4004;

// set in set_boot_src
// NAND
const ATF_STATE_BOOT_SRC_X1: u32 = 0xb300_0001;
// NOR
const ATF_STATE_BOOT_SRC_X2: u32 = 0xb300_0002;
// EMMC
const ATF_STATE_BOOT_SRC_X3: u32 = 0xb300_0003;
// SD
const ATF_STATE_BOOT_SRC_X4: u32 = 0xb300_0004;
// USB
const ATF_STATE_BOOT_SRC_X5: u32 = 0xb300_0005;

// used in mask ROM
const XXXXXX: usize = mem_map::TOP_BASE + 0x0294;
const RST_GEN: usize = mem_map::TOP_BASE + 0x3000;
const SOFT_CPU_RSTN: usize = RST_GEN + 0x0024;

use mem_map::AXI_SRAM_BASE;
// highest byte is compared vs 0x40 in mask ROM
// lowest bit against 0, may be a status bit?
const UNK1: usize = AXI_SRAM_BASE;
const BOOT_SOURCE_FLAG: usize = AXI_SRAM_BASE + 0x0004;
const BOOT_LOG_SIZE: usize = AXI_SRAM_BASE + 0x0008;
// used in mask ROM
const UNK2: usize = AXI_SRAM_BASE + 0x0010;
// coprocessor state?
const CP_STATE: usize = AXI_SRAM_BASE + 0x0018;
const AXI_SRAM_RTOS_BASE: usize = AXI_SRAM_BASE + 0x007C;
// mask ROM polls this for 0x6526_228c
const AXI_STATUS_SMTH1: usize = AXI_SRAM_BASE + 0x0080;

const TPU_SRAM_BASE: usize = 0x0c00_0000;
// Our code runs from TPU SRAM, +4k for the header.
const HEADER_SIZE: usize = 0x1000;
const CODE_SIZE_MAX: usize = 0x0003_6000;
// To avoid colliding with the boot log, our maximum size is 0x3_6000;
// plat/cv181x/include/mmap.h
//     #define BOOT_LOG_BUF_BASE (BL2_BASE + BL2_SIZE)
const BOOT_LOG_BASE: usize = TPU_SRAM_BASE + HEADER_SIZE + CODE_SIZE_MAX;
const MAX_LOG_SIZE: usize = 0x2000; // 8k

// the mask ROM makes use of SRAM for special globals
// 0x0c03_9000
const AFTER_LOG: usize = BOOT_LOG_BASE + MAX_LOG_SIZE;
const XX_SMTH1: usize = AFTER_LOG + 0x0010;
const XX_SMTH2: usize = AFTER_LOG + 0x0080;
const XX_SMTH3: usize = AFTER_LOG + 0x00bc;
const XX_SMTH4: usize = AFTER_LOG + 0x00e8;

// 0x0c09_e000
const SPECIAL_BASE: usize = AFTER_LOG + 0x5000;
const SG200X_BOOT_SRC: usize = SPECIAL_BASE + 0x0540;

// TODO: Also dump log from Arm, see if we get anything
// The mask ROM stores its own boot log in SRAM.
fn print_boot_log() {
    let boot_log_len = read32(BOOT_LOG_SIZE) as usize;
    println!("boot_log_len: {boot_log_len}");
    println!();
    println!(">>> BEGIN OF BOOT LOG");

    for i in (0..boot_log_len).step_by(4) {
        let e = read32(BOOT_LOG_BASE + i);
        let b = e.to_le_bytes();
        if i + 4 < boot_log_len {
            for c in b {
                print!("{}", c as char);
            }
        } else {
            for cc in 0..boot_log_len % 4 {
                print!("{}", b[cc] as char);
            }
        }
    }
    println!();
    println!("<<< END OF BOOT LOG");
    println!();
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

const RTC_SYS_BASE: usize = 0x0500_0000;

const RTC_SMTH_BASE: usize = RTC_SYS_BASE + 0x0002_0000;
const RTC_SMTH_XX: usize = RTC_SMTH_BASE + 0x1050;

const RTC_CTRL_BASE: usize = RTC_SYS_BASE + 0x0002_5000;
const RTC_CTRL0_UNLOCKKEY: usize = RTC_CTRL_BASE + 0x0004;
const RTC_CTRL0: usize = RTC_CTRL_BASE + 0x0008;
const RTC_CTRL0_STATUS0: usize = RTC_CTRL_BASE + 0x000c;
const RTC_POR_RST_CTRL: usize = RTC_CTRL_BASE + 0x00ac;

const RTC_BASE: usize = RTC_SYS_BASE + 0x0002_6000;
const RTC_ST_ON_REASON: usize = RTC_BASE + 0x00f8;
const RTC_ST_OFF_REASON: usize = RTC_BASE + 0x00fc;

const RTC_EN_SHUTDOWN_REQUEST: usize = RTC_BASE + 0x00c0;
const RTC_EN_POWER_CYCLE_REQUEST: usize = RTC_BASE + 0x00c8;
const RTC_EN_WARM_RESET_REQUEST: usize = RTC_BASE + 0x00cc;
const RTC_EN_PWR_VBAT_DET: usize = RTC_BASE + 0x00d0;
const RTC_EN_WATCHDOG_TIMER_RESET_REQUEST: usize = RTC_BASE + 0x00e0;

const RTC_MACRO_BASE: usize = RTC_SYS_BASE + 0x0002_6400;

fn rtc_setup() {
    const CV181X_SUPPORT_SUSPEND_RESUME: bool = false;
    if CV181X_SUPPORT_SUSPEND_RESUME {
        /*
        if get_warmboot_entry() == BL31_WARMBOOT_ENTRY {
            return;
        }
        */
    }

    // reg_rtc_mode = rtc_ctrl0[10]
    if read32(RTC_CTRL0) & (1 << 10) != 0 {
        println!("Bypass RTC mode switch");
        return;
    }

    write32(RTC_CTRL0_UNLOCKKEY, 0xAB18);

    // reg_clk32k_cg_en = rtc_ctrl0[11] -> 0
    let v = read32(RTC_CTRL0);
    let v = 0x08000000 | (v & 0xfffff7ff);
    write32(RTC_CTRL0, v);

    // cg_en_out_clk_32k = rtc_ctrl_status0[25]
    while read32(RTC_CTRL0_STATUS0) & (1 << 25) != 0x00 {}

    //r eg_rtc_mode = rtc_ctrl0[10];
    let v = read32(RTC_CTRL0);
    let v = 0x04000000 | (v & 0xfffffbff) | (0x1 << 10);
    write32(RTC_CTRL0, v);

    // DA_SOC_READY = 1
    write32(RTC_MACRO_BASE + 0x8C, 0x1);
    // DA_SOC_READY = 0
    write32(RTC_MACRO_BASE + 0x8C, 0x0);

    // udelay(200); // delay ~200us
    for i in 0..200 {
        // hack
        read32(RTC_CTRL0);
    }

    // reg_clk32k_cg_en = rtc_ctrl0[11] -> 1
    let v = read32(RTC_CTRL0);
    let v = 0x0C000000 | (v & 0xffffffff) | (0x1 << 11);
    write32(RTC_CTRL0, v);
}

fn rtc_en() {
    let v = read32(RTC_ST_ON_REASON);
    println!("st_on_reason  {v:08x}");
    let v = read32(RTC_ST_OFF_REASON);
    println!("st_off_reason {v:08x}");

    write32(RTC_EN_SHUTDOWN_REQUEST, 0x01);
    while read32(RTC_EN_SHUTDOWN_REQUEST) != 0x01 {}
    write32(RTC_EN_WARM_RESET_REQUEST, 0x01);
    while read32(RTC_EN_WARM_RESET_REQUEST) != 0x01 {}
    write32(RTC_EN_POWER_CYCLE_REQUEST, 0x01);
    while read32(RTC_EN_POWER_CYCLE_REQUEST) != 0x01 {}
    write32(RTC_EN_WATCHDOG_TIMER_RESET_REQUEST, 0x01);
    while read32(RTC_EN_WATCHDOG_TIMER_RESET_REQUEST) != 0x01 {}

    // Set rtcsys_rst_ctrl[24] = 1; bit 24 is reg_rtcsys_reset_en
    let v = read32(RTC_POR_RST_CTRL);
    write32(RTC_POR_RST_CTRL, 1 << 1);

    write32(RTC_CTRL0_UNLOCKKEY, 0xAB18);

    // Enable hw_wdg_rst_en
    let v = read32(RTC_CTRL0);
    let v = v | 0xffff0000 | (0x1 << 11) | (0x01 << 6);
    write32(RTC_CTRL0, v);

    // Avoid power up again after poweroff
    let v = read32(RTC_EN_PWR_VBAT_DET);
    write32(RTC_EN_PWR_VBAT_DET, v & !(1 << 2));
}

fn rdtime() -> usize {
    let mut time: usize = 0;
    unsafe { asm!("rdtime {time}", time = inout(reg) time) };
    time
}

#[no_mangle]
fn main() {
    let s = uart::SGSerial::new();
    init_logger(s);
    println!();
    println!();
    println!();
    println!("oreboot 🦀 bt0");
    print_ids();

    if false {
        println!(">>> mask ROM dump");
        util::dump_block(rom::MASK_ROM_FN_BASE, 96 * 1024, 32);
        println!("<<< mask ROM dump");
        panic!("DO NOT PANIC! EVERYTHING IS OKAY!");
    }

    let boot_src = rom::get_boot_src();
    println!("boot src: {boot_src}");
    let retry_count = rom::get_retry_count();
    println!("retries:  {retry_count}");

    println!();

    let atf_state = read32(ATF_STATE);
    println!("ATF state:     {atf_state:08x}");
    write32(ATF_STATE, ATF_STATE_BL2_MAIN);

    let cp_state = read32(CP_STATE);
    println!("CP_STATE:      {cp_state:08x}");
    let conf = read32(CONF);
    println!("CONF:          {conf:08x}");

    let chip_type_v = (conf >> 28) & 0b111;
    let chip_type = match chip_type_v {
        1 => "SG20000 / 512MB DDR3 RAM @1866",
        3 => "CV1800B / 64MB DDR2 RAM @1333",
        _ => "unknown",
    };
    println!("TYPE:          {chip_type} ({chip_type_v})");
    println!();

    let efuse_leakage = efuse::setup();
    println!();

    if false {
        print_boot_log();
    }

    // fsbl plat/cv181x/ddr/ddr_pkg_info.c

    // 1: NY 4Gbit DDR3
    // 4: ESMT 512Mbit DDR2
    let dram_vendor = (efuse_leakage >> 21) & 0b11111;
    // 1: 512Mbit
    // 4: 4Gbit
    let dram_capacity = (efuse_leakage >> 26) & 0b111;

    println!("dram_vendor {dram_vendor}, dram_capacity {dram_capacity}");
    println!();

    println!();

    let ddr_rate = match chip_type_v {
        1 => 1866,
        3 => 1333,
        _ => panic!("DDR rate not supported"),
    };

    /*
     CV1800B / Duo
       W_LOCK0:       00000000
       EFUSE_STATUS:  00000070
       CONF:          3500032a
       EFUSE_LEAKAGE: 64800024
       FTSN3:         e1a5e4ca
       FTSN4:         15274190
       TYPE:          CV1800B / 64MB DDR2 RAM 1333
       CP_STATE:      00000000
    */

    /*
     SG2000 / Duo S
       W_LOCK0:       00000018
       EFUSE_STATUS:  00000070
       CONF:          170003ab
       EFUSE_LEAKAGE: 5020002d
       FTSN3:         d1c21ea5
       FTSN4:         1526b59a
       TYPE:          unknown
       CP_STATE:      00000000
       [bt0] Jump to main stage @80200000
    */

    {
        let src = rom::get_boot_src();
        println!("boot from {src}");

        let flag = read32(BOOT_SOURCE_FLAG);
        println!("boot flag {flag:08x}");

        const BOOT_SRC_USB: [u8; 4] = *b"MGN1";
        let v = u32::from_be_bytes(BOOT_SRC_USB);
        write32(BOOT_SOURCE_FLAG, v);

        let flag = read32(BOOT_SOURCE_FLAG);
        println!("boot flag {flag:08x}");
    }

    rtc_setup();
    rtc_en();

    // print_boot_log();

    let start = rdtime();
    dram::init(ddr_rate, dram_vendor);
    println!("DRAM init done");
    let time = rdtime() - start;
    println!("time: {time}");

    let v = read32(AXI_SRAM_RTOS_BASE);
    // 0x0c85e985
    // CVI_RTOS_MAGIC_CODE 0xABC0DEF
    println!("RTOS base: 0x{v:08x}");

    // `make run` in main
    println!(">> load main stage over USB");
    println!();

    let load_addr = mem_map::DRAM_BASE;
    rom::load_image(load_addr, 0x0, 0x6000, 0);

    // https://github.com/orangecms/sbitest
    println!(">> load SBI test over USB");
    println!();

    let test_addr = mem_map::DRAM_BASE + 0x0020_0000;
    rom::load_image(test_addr, 0x0, 0x1000, 0);

    println!("[bt0] Jump to main stage @{load_addr:08x}");

    const BOOT_MAIN: bool = false;
    if BOOT_MAIN {
        // RV64ACDFIMSUVX
        exec_payload(load_addr);
    } else {
        // RV64ACDFIMSUX
        exec_hartl(load_addr);
    }

    if false {
        println!("[bt0] Exit from main stage, resetting...");
        unsafe {
            reset();
        }
    }

    unsafe { riscv::asm::wfi() };
}

const SEC_SUBSYS_BASE: usize = 0x0200_0000;

const SEC_XXY_BASE: usize = SEC_SUBSYS_BASE + 0x0009_0000;
// mask ROM may set this to 0x0080_0800
const SEC_SYS_SMTH: usize = SEC_XXY_BASE + 0x005c;

const SEC_SYS_BASE: usize = SEC_SUBSYS_BASE + 0x000B_0000;

const SEC_SYS_CTRL: usize = SEC_SYS_BASE + 0x0004;

const SEC_SYS_A_ADDR_L: usize = SEC_SYS_BASE + 0x0010;
const SEC_SYS_A_ADDR_H: usize = SEC_SYS_BASE + 0x0014;

const SEC_SYS_B_ADDR_L: usize = SEC_SYS_BASE + 0x0018;
const SEC_SYS_B_ADDR_H: usize = SEC_SYS_BASE + 0x001c;

const SEC_SYS_L_ADDR_L: usize = SEC_SYS_BASE + 0x0020;
const SEC_SYS_L_ADDR_H: usize = SEC_SYS_BASE + 0x0024;

// Bits Name
// 0    reg_soft_reset_x_cpucore0
// 1    reg_soft_reset_x_cpucore1
// 2    reg_soft_reset_x_cpucore2
// 3    reg_soft_reset_x_cpucore3
// 4    reg_soft_reset_x_cpusys0
// 5    reg_soft_reset_x_cpusys1
// 6    reg_soft_reset_x_cpusys2
// 31:7 Reserved

fn exec_hartl(addr: usize) {
    // should be no-op
    let v = read32(SOFT_CPU_RSTN);
    write32(SOFT_CPU_RSTN, v & !(1 << 6));

    let v = read32(SEC_SYS_CTRL);
    write32(SEC_SYS_CTRL, v | (1 << 13));

    write32(SEC_SYS_L_ADDR_L, addr as u32);
    write32(SEC_SYS_L_ADDR_H, (addr >> 32) as u32);

    // reset
    let v = read32(SOFT_CPU_RSTN);
    write32(SOFT_CPU_RSTN, v | (1 << 6));
}

fn exec_payload(addr: usize) {
    unsafe {
        // jump to main
        let f: EntryPoint = transmute(addr);
        // asm!("fence.i");
        f();
    }
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &PanicInfo) -> ! {
    if let Some(location) = info.location() {
        if DEBUG {
            println!(
                "[bt0] panic in '{}' line {}",
                location.file(),
                location.line(),
            );
        }
    } else {
        if DEBUG {
            println!("[bt0] panic at unknown location");
        }
    };
    if let Some(msg) = info.message() {
        println!("[bt0]   {msg}");
    }
    loop {
        core::hint::spin_loop();
    }
}
