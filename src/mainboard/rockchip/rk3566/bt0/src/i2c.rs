use util::mmio::{read32, write32};

use crate::arm::udelay;
use crate::mem_map::{self, I2C0_BASE};

const I2C0_CTRL: usize = I2C0_BASE + 0x0000;
const I2C0_CLK: usize = I2C0_BASE + 0x0004;
const I2C0_MRXADDR: usize = I2C0_BASE + 0x0008;
const I2C0_MRXRADDR: usize = I2C0_BASE + 0x000C;
const I2C0_MRXCNT: usize = I2C0_BASE + 0x0014;
const I2C0_IEN: usize = I2C0_BASE + 0x0018;
const I2C0_IPD: usize = I2C0_BASE + 0x001c;

const I2C0_RXDATA0: usize = I2C0_BASE + 0x0200;

pub fn i2c_init() {
    write32(mem_map::PMU_CRU_BASE + 0x010c, 0x007f_0000);
    write32(mem_map::PMU_CRU_BASE + 0x0080, 0x0003_0000);
    /* I2C0 GPIO config (GPIO B) */
    write32(mem_map::PMU_GRF_BASE + 0x0008, 0x0ff0_0110);
    write32(I2C0_CLK, 0x000e_000e);
}

const POLL_LIMIT: usize = 2000;

fn i2c_commit(addr: u8) {
    for _ in 0..POLL_LIMIT {
        if read32(I2C0_IPD) & addr as u32 != 0 {
            break;
        }
        udelay(10);
    }
    write32(I2C0_IPD, addr as u32);
}

pub fn i2c_read(addr: u8, val: u8) -> u8 {
    write32(I2C0_IEN, 0x0010);
    write32(I2C0_CTRL, 0x122b);

    i2c_commit(addr);

    let v = read32(I2C0_CTRL);
    write32(I2C0_CTRL, v & 0xfffffff7);
    write32(I2C0_IEN, 8);
    write32(I2C0_MRXCNT, 1);

    write32(I2C0_MRXADDR, ((addr as u32 & 0x7f) << 1) | 0x1000000);
    write32(I2C0_MRXRADDR, val as u32 | 0x1000000);

    i2c_commit(addr);

    let r = read32(I2C0_RXDATA0);

    write32(I2C0_IEN, 0x20);
    let v = read32(I2C0_CTRL);
    write32(I2C0_CTRL, v & (0xfffffff7 | 0x10));

    i2c_commit(addr);

    write32(I2C0_CTRL, 0);

    r as u8
}
