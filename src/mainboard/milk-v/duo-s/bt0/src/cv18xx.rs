use util::{read32, write32};

use crate::efuse;
use crate::mem_map::{AXI_SRAM_BASE, MASK_ROM_FN_BASE, SEC_SUBSYS_BASE, TOP_BASE};

// Our code runs from TPU SRAM, +4k for the header.
pub const HEADER_SIZE: usize = 0x1000;
// To avoid colliding with the boot log, our maximum size is 0x3_6000;
pub const CODE_SIZE_MAX: usize = 0x0003_6000;

const BOOT_SOURCE_FLAG: usize = AXI_SRAM_BASE + 0x0004;
const BOOT_LOG_SIZE: usize = AXI_SRAM_BASE + 0x0008;

// plat/cv181x/include/mmap.h
//     #define BOOT_LOG_BUF_BASE (BL2_BASE + BL2_SIZE)
const BOOT_LOG_BASE: usize = crate::mem_map::TPU_SRAM_BASE + HEADER_SIZE + CODE_SIZE_MAX;
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

const CONF: usize = TOP_BASE + 0x0004;
// used by mask ROM: _DAT_03000080 == 0x6526228c
const GP_REG0: usize = TOP_BASE + 0x0080;
// aka GP_REG1 in vendor code
const ATF_STATE: usize = TOP_BASE + 0x0084;
// used in mask ROM
const XXXXXX: usize = TOP_BASE + 0x0294;
const RST_GEN: usize = TOP_BASE + 0x3000;
pub const SOFT_CPU_RSTN: usize = RST_GEN + 0x0024;

// highest byte is compared vs 0x40 in mask ROM
// lowest bit against 0, may be a status bit?
const UNK1: usize = AXI_SRAM_BASE;
// used in mask ROM
const UNK2: usize = AXI_SRAM_BASE + 0x0010;
// coprocessor state?
const CP_STATE: usize = AXI_SRAM_BASE + 0x0018;
pub const AXI_SRAM_RTOS_BASE: usize = AXI_SRAM_BASE + 0x007C;
// mask ROM polls this for 0x6526_228c
const AXI_STATUS_SMTH1: usize = AXI_SRAM_BASE + 0x0080;
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
*/

/*
 SG2002 / Duo 256
   ATF state:     b100fe00
   CP_STATE:      00000000
   CONF:          570003ab
   TYPE:          SG2002 / 256 DDR3 RAM @1866 (5)

   SW INFO:       00000000
   EFUSE_STATUS:  00000020
   FTSN0:         00000000
   efuse: FTSN0 is NOT locked
   FTSN1:         00000000
   efuse: FTSN1 is NOT locked
   EFUSE_LEAKAGE: 2c40002a
   efuse: FTSN2 is locked
   FTSN3:         d1c05443
   efuse: FTSN3 is locked
   FTSN4:         1526b59a
   efuse: FTSN4 is locked

   DRAM: NY 2Gbit DDR3, (vendor: 2, capacity: 3)
   Package: QFN88
*/

pub fn print_platform_state() {
    let atf_state = read32(ATF_STATE);
    // TODO: map possible values to strings
    println!("ATF state:     {atf_state:08x}");
    write32(ATF_STATE, ATF_STATE_BL2_MAIN);

    let cp_state = read32(CP_STATE);
    println!("CP_STATE:      {cp_state:08x}");
}

pub fn get_chip_type() -> u32 {
    let conf = read32(CONF);
    println!("CONF:          {conf:08x}");

    let chip_type = (conf >> 28) & 0b111;
    let info = match chip_type {
        1 => "SG2000 / 512MB DDR3 RAM @1866",
        3 => "CV1800B / 64MB DDR2 RAM @1333",
        5 => "SG2002 / 256 DDR3 RAM @1866",
        _ => "unknown",
    };
    println!("Platform: {info} ({chip_type})");
    chip_type
}

pub fn get_dram_type() -> (u32, usize) {
    let efuse_leakage = efuse::setup();

    // fsbl plat/cv181x/ddr/ddr_pkg_info.c
    let dram_vendor = (efuse_leakage >> 21) & 0b11111;
    let dram_capacity = (efuse_leakage >> 26) & 0b111;
    let package_type = (efuse_leakage >> 29) & 0b111;

    let dram_type = match (dram_vendor, dram_capacity) {
        (1, 5) => "NY 4Gbit DDR3",
        (2, 3) => "NY 2Gbit DDR3",
        (4, 1) => "ESMT 512Mbit DDR2",
        (_, _) => "unknown",
    };

    let package = match package_type {
        1 => "QFN88",
        2 => "BGA",
        3 => "QFN68",
        _ => "unknown",
    };

    println!("DRAM: {dram_type}, (vendor: {dram_vendor}, capacity: {dram_capacity})");
    println!("Package: {package}");

    let chip_type = get_chip_type();
    let ddr_rate = match chip_type {
        1 => 1866,
        3 => 1333,
        5 => 1866,
        _ => panic!("DDR rate for chip type {chip_type} not supported"),
    };

    (dram_vendor, ddr_rate)
}

// TODO: Also dump log from Arm, see if we get anything
// The mask ROM stores its own boot log in SRAM.
pub fn print_boot_log() {
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

pub fn print_boot_info() {
    let src = crate::rom::get_boot_src();
    println!("boot from {src}");

    let flag = read32(BOOT_SOURCE_FLAG);
    println!("boot flag {flag:08x}");

    let v = u32::from_be_bytes(BOOT_SRC_USB);
    write32(BOOT_SOURCE_FLAG, v);

    let flag = read32(BOOT_SOURCE_FLAG);
}

const BOOT_SRC_USB: [u8; 4] = *b"MGN1";
pub fn dump_mask_rom() {
    println!(">>> mask ROM dump");
    util::dump_block(MASK_ROM_FN_BASE, 96 * 1024, 32);
    println!("<<< mask ROM dump");
    panic!("DO NOT PANIC! EVERYTHING IS OKAY!");
}

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

pub fn exec_hartl(addr: usize) {
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

// only for debugging
// we call this AFTER_LOG
// define ATF_DBG_REG (BOOT_LOG_LEN_ADDR + BOOT_LOG_LEN_SIZE)
// define ATF_ERR_REG (ATF_DBG_REG + 0x04)
// define ATF_ERR_INFO0 (ATF_DBG_REG + 0x08)
// define CP_STATE_REG (ATF_DBG_REG + 0x0C)
// define ATF_ERR (((unsigned int __volatile__ *)ATF_ERR_REG)[0])

// NOTE: Those values are used to tell the current step/state in the mask ROM flow.
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
