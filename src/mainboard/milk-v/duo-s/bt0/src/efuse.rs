use crate::mem_map::EFUSE;
use crate::util::{read32, write32};

// https://github.com/sophgo/cvi_alios_open
//   components/chip_cv181x/src/drivers/efuse/wj/cvi/cvi_efuse.c
// plat/cv181x/include/security/efuse.h
const EFUSE_MODE: usize = EFUSE;
const EFUSE_ADDR: usize = EFUSE + 0x0004;
const EFUSE_STATUS: usize = EFUSE + 0x0010;

const EFUSE_SHADOW: usize = EFUSE + 0x0100;
// aliases...
const FTSN0: usize = EFUSE + 0x0100;

const EFUSE_CUSTOMER: usize = EFUSE_SHADOW + 0x0004;
const FTSN1: usize = EFUSE + 0x0104;

// A bit of an odd name? Taken from vendor code, alias of FTSN2.
// It is locked and contains the DRAM vendor and capacity.
const EFUSE_LEAKAGE: usize = EFUSE + 0x0108;
const FTSN2: usize = EFUSE + 0x0108;

const FTSN3: usize = EFUSE + 0x010C;
const FTSN4: usize = EFUSE + 0x0110;
// maks ROM reads this in the beginning
const XY: usize = EFUSE + 0x0128;
const SW_INFO: usize = EFUSE + 0x012c;
const XZ: usize = EFUSE + 0x0130;

// range used to hold some state, printed every now and then in SRAM log
const YZ: usize = EFUSE + 0x0134;
const ZZ: usize = EFUSE + 0x0180;

const EFUSE_W_LOCK0: usize = EFUSE + 0x0198;

// if XX is 0, mask ROM sets `0x0209_005c = 0x0080_0800;` (security subsystem)
// it is only ever read, never written to
// in SRAM boot log: `SCS/0/0. I:ep_swinfo.` - XX value is the first 0 after SCS
const XX: usize = EFUSE + 0x01a0;

const EFUSE_MODE_ON: u32 = 0x10;
const EFUSE_MODE_SET_BIT: u32 = 0x14;
const EFUSE_MODE_OFF: u32 = 0x18;
const EFUSE_MODE_REFRESH_SHADOW: u32 = 0x30;

const BIT_FTSN0_LOCK: u32 = 0;
const BIT_FTSN1_LOCK: u32 = 1;
const BIT_FTSN2_LOCK: u32 = 2;
const BIT_FTSN3_LOCK: u32 = 3;
const BIT_FTSN4_LOCK: u32 = 4;

fn poll_efuse() {
    while read32(EFUSE_STATUS) & 0x1 != 0 {}
}

fn efuse_program_bit(addr: u32, bit: u32) {
    let v = 0xfff & ((bit << 7) | ((addr & 0x3f) << 1));
    poll_efuse();
    write32(EFUSE_ADDR, v);
    write32(EFUSE_MODE, EFUSE_MODE_SET_BIT);
    poll_efuse();
    write32(EFUSE_ADDR, v | 1);
    write32(EFUSE_MODE, EFUSE_MODE_SET_BIT);
}

// after lock_efuse_chipsn() in plat/cv181x/bl2/bl2_opt.c
pub fn setup() -> u32 {
    // let efuse_mode = read32(EFUSE_MODE);
    // println!("efuse mode: {efuse_mode:08x}");
    poll_efuse();
    write32(EFUSE_MODE, EFUSE_MODE_ON);

    let info = read32(SW_INFO);
    println!("SW INFO:       {info:08x}");
    let efuse_status = read32(EFUSE_STATUS);
    println!("EFUSE_STATUS:  {efuse_status:08x}");

    let w_lock0 = read32(EFUSE_W_LOCK0);
    let v = read32(FTSN0);
    println!("FTSN0:         {v:08x}");
    if w_lock0 & (1 << BIT_FTSN0_LOCK) == 0 {
        println!("efuse: FTSN0 is NOT locked");
    } else {
        println!("efuse: FTSN0 is locked");
    }
    let v = read32(FTSN1);
    println!("FTSN1:         {v:08x}");
    if w_lock0 & (1 << BIT_FTSN1_LOCK) == 0 {
        println!("efuse: FTSN1 is NOT locked");
    } else {
        println!("efuse: FTSN1 is locked");
    }
    let efuse_leakage = read32(EFUSE_LEAKAGE);
    println!("EFUSE_LEAKAGE: {efuse_leakage:08x}");
    if w_lock0 & (1 << BIT_FTSN2_LOCK) != 0 {
        println!("efuse: FTSN2 is NOT locked");
    } else {
        println!("efuse: FTSN2 is locked");
    }
    let v = read32(FTSN3);
    println!("FTSN3:         {v:08x}");
    if w_lock0 & (1 << BIT_FTSN3_LOCK) == 0 {
        println!("efuse: FTSN3 is NOT locked");
        // efuse_program_bit(0x26, BIT_FTSN3_LOCK);
    } else {
        println!("efuse: FTSN3 is locked");
    }
    let v = read32(FTSN4);
    println!("FTSN4:         {v:08x}");
    if w_lock0 & (1 << BIT_FTSN4_LOCK) == 0 {
        println!("efuse: FTSN4 is NOT locked");
        // efuse_program_bit(0x26, BIT_FTSN4_LOCK);
    } else {
        println!("efuse: FTSN4 is locked");
    }

    poll_efuse();
    write32(EFUSE_MODE, EFUSE_MODE_REFRESH_SHADOW);

    poll_efuse();
    write32(EFUSE_MODE, EFUSE_MODE_OFF);

    efuse_leakage
}
