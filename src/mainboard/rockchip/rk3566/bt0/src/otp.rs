use util::mmio::{read32, write32};

use crate::arm::udelay;
use crate::mem_map::{CRU_NS_BASE, OTP_NS_BASE, OTP_PHY_BASE, OTP_S_BASE, SYS_SGRF_BASE};

const SYS_SGRF_0008: usize = SYS_SGRF_BASE + 0x0008;

// OTP_NS = non-secure, OTP_S = secure
// TODO: Why non-secure vs secure OTP?

/* --- "Non-secure" --- */
// Register names as per Linux driver; the vendor manual calls it OTP_NS.
// https://github.com/torvalds/linux/blob/master/drivers/nvmem/rockchip-otp.c
const OTP_NS_SBPI_CTRL: usize = OTP_NS_BASE + 0x0020;
const OTP_NS_SBPI_CMD_VALID_PRE: usize = OTP_NS_BASE + 0x0024;
const OTP_NS_SBPI_CS_VALID_PRE: usize = OTP_NS_BASE + 0x0028;
const OTP_NS_SBPI_STATUS: usize = OTP_NS_BASE + 0x002C;
const OTP_NS_USER_CTRL: usize = OTP_NS_BASE + 0x0100;
const OTP_NS_USER_ADDR: usize = OTP_NS_BASE + 0x0104;
const OTP_NS_USER_ENABLE: usize = OTP_NS_BASE + 0x0108;
const OTP_NS_0120: usize = OTP_NS_BASE + 0x0120;
const OTP_NS_USER_Q: usize = OTP_NS_BASE + 0x0124;
const OTP_NS_INT_STATUS: usize = OTP_NS_BASE + 0x0304;
const OTP_NS_SBPI_CMD0_OFFSET: usize = OTP_NS_BASE + 0x1000;
const OTP_NS_SBPI_CMD1_OFFSET: usize = OTP_NS_BASE + 0x1004;

/* ----- "Secure" ----- */
const OTP_S_SBPI_CTRL: usize = OTP_S_BASE + 0x0020;
const OTP_S_SBPI_CMD_VALID_PRE: usize = OTP_S_BASE + 0x0024;

const OTP_S_USER_CTRL: usize = OTP_S_BASE + 0x0100;
const OTP_S_USER_ADDR: usize = OTP_S_BASE + 0x0104;
const OTP_S_USER_ENABLE: usize = OTP_S_BASE + 0x0108;
const OTP_S_0120: usize = OTP_S_BASE + 0x0120;
const OTP_S_USER_Q: usize = OTP_S_BASE + 0x0124;
const OTP_S_INT_STATUS: usize = OTP_S_BASE + 0x0304;
const OTP_S_SBPI_CMD0_OFFSET: usize = OTP_S_BASE + 0x1000;
const OTP_S_SBPI_CMD1_OFFSET: usize = OTP_S_BASE + 0x1004;

/* -------- PHY ------- */
const OTP_PHY_0000: usize = OTP_PHY_BASE + 0x0000;
const OTP_PHY_0004: usize = OTP_PHY_BASE + 0x0004;

const OTP_PHY_0010: usize = OTP_PHY_BASE + 0x0010;
const OTP_PHY_0014: usize = OTP_PHY_BASE + 0x0014;
const OTP_PHY_0018: usize = OTP_PHY_BASE + 0x0018;
const OTP_PHY_001C: usize = OTP_PHY_BASE + 0x001C;
const OTP_PHY_0020: usize = OTP_PHY_BASE + 0x0020;

const OTP_PHY_0050: usize = OTP_PHY_BASE + 0x0050;

const OTP_PHY_0210: usize = OTP_PHY_BASE + 0x0210;
const OTP_PHY_0218: usize = OTP_PHY_BASE + 0x0218;

pub fn otp_read() {
    //
}

fn otpc_status(bit: u32) -> Result<(), ()> {
    let b = 1 << bit;
    for _ in 0..10000 {
        if read32(OTP_NS_INT_STATUS) & b != 0 {
            write32(OTP_NS_INT_STATUS, 0xffff_0000 | b);
            return Ok(());
        }
        udelay(1);
    }
    Err(())
}

fn otp_s_status(bit: u32) -> Result<(), ()> {
    let b = 1 << bit;
    for _ in 0..10000 {
        if read32(OTP_S_INT_STATUS) & b != 0 {
            write32(OTP_S_INT_STATUS, 0xffff_0000 | b);
            return Ok(());
        }
        udelay(1);
    }
    Err(())
}

fn s_init(x: bool) {
    write32(OTP_S_SBPI_CTRL, 0xff00_0200);
    write32(OTP_S_SBPI_CMD_VALID_PRE, 0xffff_0001);
    write32(OTP_S_SBPI_CMD0_OFFSET, 0x0000_00fa);
    // NOTE: semantics unknown, and always true in vendor code
    let v = if x { 0 } else { 9 };
    write32(OTP_S_SBPI_CMD1_OFFSET, v);
    write32(OTP_S_SBPI_CTRL, 0x0001_0001);
    if otp_s_status(1).is_err() {
        panic!("OTP S error");
    }
}

// NOTE: factored out, duplicated in vendor code
pub fn pre() {
    write32(CRU_NS_BASE + 0x0470, 0x8000_8000);
    udelay(2);
    write32(CRU_NS_BASE + 0x0470, 0x8000_0000);
    udelay(1);
    // NOTE: This is conditional in the vendor code, but always run.
    s_init(true);
}

const DEBUG: bool = false;

fn xx_status(xx: u32) -> u32 {
    // ubfx, sbfx
    if (xx >> 6) & 3 != 3 {
        if DEBUG {
            println!("  we got this");
        }
        return (xx >> 5) & 1;
    }
    return 0xffffffff;
}

const STEP_SIZE: usize = 4;
const READS_PER_STEP: usize = 2;
const BITS_PER_READ: usize = 0x10;

// NOTE: The vendor code has another, last param that is always set to true and
// condition for doing s_init() in pre().
// NOTE: By convention, an entry is considered a 32-bit value (4 bytes) here.
fn read_s(start: u32, entries: u32) -> Result<(), ()> {
    write32(SYS_SGRF_0008, 0x0002_0002);
    pre();

    write32(OTP_S_USER_CTRL, 0x0001_0001);
    udelay(2);

    // We get 16 bits per read, convert to 4 bytes at a time, so the max offset
    // is the number of entries shifted by << 2, i.e., *4.
    let max_offset = (start + entries) << READS_PER_STEP;
    let mut entry_pos = start << READS_PER_STEP;

    for offset in (entry_pos..max_offset).step_by(STEP_SIZE) {
        let mut val = 0;
        for pos in (0..READS_PER_STEP * BITS_PER_READ).step_by(BITS_PER_READ) {
            write32(OTP_S_USER_ADDR, 0xffff_0000 | entry_pos);
            write32(OTP_S_USER_ENABLE, 0x0001_0001);
            let _ = otp_s_status(2);
            let v16 = read32(OTP_S_USER_Q) as u16;
            let xx = read32(OTP_S_0120);
            let s = xx_status(xx);
            if s != 0 {
                write32(OTP_S_USER_CTRL, 0x0001_0000);
                write32(SYS_SGRF_0008, 0x0002_0000);
                println!("OTP S @{offset:02x}({pos}): status {xx:08x}/{s:08x}");
                return Err(());
            }
            entry_pos += 1;
            val = val | ((v16 as u32) << pos);
        }
        let val = val.swap_bytes();
        println!("OTP S @{offset:02x}: {val:08x}");
    }

    write32(OTP_S_USER_CTRL, 0x0001_0000);
    write32(SYS_SGRF_0008, 0x0002_0000);
    Ok(())
}

// NOTE: By convention, an entry is considered a 32-bit value (4 bytes) here.
// see Linux px30_otp_read
pub fn read_ns(start: u32, entries: u32) -> Result<(), ()> {
    write32(SYS_SGRF_0008, 0x0002_0000);
    pre();

    write32(OTP_NS_USER_CTRL, 0x0001_0001);
    udelay(2);

    // We get 16 bits per read, convert to 4 bytes at a time, so the max offset
    // is the number of entries shifted by << 2, i.e., *4.
    let max_offset = (start + entries) << READS_PER_STEP;
    let mut entry_pos = start << READS_PER_STEP;

    for offset in (entry_pos..max_offset).step_by(STEP_SIZE) {
        let mut val = 0;
        for pos in (0..READS_PER_STEP * BITS_PER_READ).step_by(BITS_PER_READ) {
            write32(OTP_NS_USER_ADDR, 0xffff_0000 | entry_pos);
            write32(OTP_NS_USER_ENABLE, 0x0001_0001);
            let _ = otpc_status(2);
            // 16 bits per read
            let v16 = read32(OTP_NS_USER_Q) as u16;
            let xx = read32(OTP_NS_0120);
            let s = xx_status(xx);
            if s != 0 {
                write32(OTP_NS_USER_CTRL, 0x0001_0000);
                println!("OTP NS @{offset:02x}({pos}): status {xx:08x}/{s:08x}");
                return Err(());
            }
            entry_pos += 1;
            val = val | ((v16 as u32) << pos);
        }
        let val = val.swap_bytes();
        println!("OTP NS @{offset:02x}: {val:08x}");
    }

    write32(OTP_NS_USER_CTRL, 0x0001_0000);
    Ok(())
}

pub fn otp_phy_init() {
    // "clear" the first 0x80 bytes - or 32 registers
    for reg in (OTP_PHY_0000..OTP_PHY_0000 + 0x80).step_by(4) {
        write32(reg, 0xffff_ffff);
    }
    if read_s(0x0a / 2, 1).is_ok() {
        write32(OTP_PHY_0004, 0xffff_f00f);
    }
    if read_s(0x20 / 2, 1).is_ok() {
        write32(OTP_PHY_0010, 0xffff_00fc);
        write32(OTP_PHY_0014, 0xffff_ff00);
    }
    write32(OTP_PHY_0018, 0xffff_00ff);
    write32(OTP_PHY_001C, 0xffff_ff00);
    if read_s(0x3c / 2, 1).is_ok() {
        write32(OTP_PHY_001C, 0xffff_0000);
        write32(OTP_PHY_0020, 0xffff_ff00);
    }
    if read_s(0x44 / 2, 1).is_ok() {
        write32(OTP_PHY_0020, 0xfffffcff);
    }
    for reg in (OTP_PHY_0050..OTP_PHY_0050 + 0x20).step_by(4) {
        write32(reg, 0xffff_0000);
    }
    write32(OTP_PHY_0210, 0x0001_0001);
    write32(OTP_PHY_0218, 0x0001_0001);

    // OUR CODE
    let _ = read_s(0x0, 0x40);
}
