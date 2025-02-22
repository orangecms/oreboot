use util::mmio::{read32, write32};

use crate::mem_map::{AXI_MON_BASE, DDR_TOP_BASE};

const MONITOR_CLOCK_CONTROL: usize = DDR_TOP_BASE + 0x0014;
const MONITOR_CLOCK_GATING_ENABLE: u32 = 1 << 8;

const REMAPPING_BASE: usize = 0;
const AXIMON_M1_WRITE: usize = REMAPPING_BASE + 0x000;
const AXIMON_M1_READ: usize = REMAPPING_BASE + 0x080;
const AXIMON_M2_WRITE: usize = REMAPPING_BASE + 0x100;
const AXIMON_M2_READ: usize = REMAPPING_BASE + 0x180;
const AXIMON_M3_WRITE: usize = REMAPPING_BASE + 0x200;
const AXIMON_M3_READ: usize = REMAPPING_BASE + 0x280;
const AXIMON_M4_WRITE: usize = REMAPPING_BASE + 0x300;
const AXIMON_M4_READ: usize = REMAPPING_BASE + 0x380;
const AXIMON_M5_WRITE: usize = REMAPPING_BASE + 0x400;
const AXIMON_M5_READ: usize = REMAPPING_BASE + 0x480;
const AXIMON_M6_WRITE: usize = REMAPPING_BASE + 0x500;
const AXIMON_M6_READ: usize = REMAPPING_BASE + 0x580;

const AXIMON_OFFSET_LAT_BIN_SIZE_SEL: usize = 0x50;
pub fn axi_mon_latency_setting(lat_bin_size_sel: u32) {
    // for ddr3 1866: bin_size_sel=0d'5
    write32(
        (AXI_MON_BASE + AXIMON_M1_WRITE + AXIMON_OFFSET_LAT_BIN_SIZE_SEL),
        lat_bin_size_sel,
    );
    write32(
        (AXI_MON_BASE + AXIMON_M1_READ + AXIMON_OFFSET_LAT_BIN_SIZE_SEL),
        lat_bin_size_sel,
    );

    // input clk sel
    write32(AXI_MON_BASE + AXIMON_M1_WRITE + 0x00, 0x01000100);
    // hit sel setting
    let rdata = read32(AXI_MON_BASE + AXIMON_M1_WRITE + 0x04);
    write32(AXI_MON_BASE + AXIMON_M1_WRITE + 0x04, rdata & 0xfffffc00);

    write32(AXI_MON_BASE + AXIMON_M1_READ + 0x00, 0x01000100);
    let rdata = read32(AXI_MON_BASE + AXIMON_M1_READ + 0x04);
    write32(AXI_MON_BASE + AXIMON_M1_READ + 0x04, rdata & 0xfffffc00);

    write32(
        AXI_MON_BASE + AXIMON_M5_WRITE + AXIMON_OFFSET_LAT_BIN_SIZE_SEL,
        lat_bin_size_sel,
    );
    write32(
        AXI_MON_BASE + AXIMON_M5_READ + AXIMON_OFFSET_LAT_BIN_SIZE_SEL,
        lat_bin_size_sel,
    );

    write32(AXI_MON_BASE + AXIMON_M5_WRITE + 0x00, 0x01000100);
    let rdata = read32(AXI_MON_BASE + AXIMON_M5_WRITE + 0x04);
    write32(AXI_MON_BASE + AXIMON_M5_WRITE + 0x04, rdata & 0xfffffc00);

    write32(AXI_MON_BASE + AXIMON_M5_READ + 0x00, 0x01000100);
    let rdata = read32(AXI_MON_BASE + AXIMON_M5_READ + 0x04);
    write32(AXI_MON_BASE + AXIMON_M5_READ + 0x04, rdata & 0xfffffc00);

    let rdata = read32(MONITOR_CLOCK_CONTROL);
    write32(MONITOR_CLOCK_CONTROL, rdata | MONITOR_CLOCK_GATING_ENABLE);
}

const AXIMON_START_REGVALUE: u32 = 0x30001;
fn axi_mon_start(r: usize) {
    write32(AXI_MON_BASE + r, AXIMON_START_REGVALUE);
}

pub fn axi_mon_start_all() {
    axi_mon_start(AXIMON_M1_WRITE);
    axi_mon_start(AXIMON_M1_READ);
    axi_mon_start(AXIMON_M2_WRITE);
    axi_mon_start(AXIMON_M2_READ);
    axi_mon_start(AXIMON_M3_WRITE);
    axi_mon_start(AXIMON_M3_READ);
    axi_mon_start(AXIMON_M4_WRITE);
    axi_mon_start(AXIMON_M4_READ);
    axi_mon_start(AXIMON_M5_WRITE);
    axi_mon_start(AXIMON_M5_READ);
    axi_mon_start(AXIMON_M6_WRITE);
    axi_mon_start(AXIMON_M6_READ);
}
