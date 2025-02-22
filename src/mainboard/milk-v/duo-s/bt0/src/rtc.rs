use oreboot_arch::riscv64::util::delay;
use util::mmio::{read32, write32};

use crate::mem_map::RTC_SYS_BASE;

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

pub fn init() {
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

    // delay ~200us
    delay(500);

    // reg_clk32k_cg_en = rtc_ctrl0[11] -> 1
    let v = read32(RTC_CTRL0);
    let v = 0x0C000000 | (v & 0xffffffff) | (0x1 << 11);
    write32(RTC_CTRL0, v);
}

pub fn en() {
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
