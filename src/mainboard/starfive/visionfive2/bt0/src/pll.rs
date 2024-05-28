use soc::starfive::jh7110::pac;

use crate::init;
use starfive_visionfive2_lib::{read32, udelay, write32};

// see `boot/arch/riscv/cpu/jh7110/pll.c` `pll_set_rate`
// NOTE: The order may be irrelevant, which would allow for simplification.

// ---------- see JH7110 SoC manual p61
pub struct PllFreq {
    prediv: u8,
    fbdiv: u16, // feedback divider
}

pub const PLL0_500000000: PllFreq = PllFreq {
    prediv: 6,
    fbdiv: 125,
};

pub const PLL0_1000000000: PllFreq = PllFreq {
    prediv: 3,
    fbdiv: 125,
};

pub const PLL0_1500000000: PllFreq = PllFreq {
    prediv: 2,
    fbdiv: 125,
};

pub const PLL1_DDR2133_1066000000: PllFreq = PllFreq {
    prediv: 12,
    fbdiv: 533,
};

pub const PLL1_DDR_LOW_SPEED: PllFreq = PllFreq {
    prediv: 1,
    fbdiv: 533,
};

pub const PLL2_1188000000: PllFreq = PllFreq {
    prediv: 2,
    fbdiv: 99,
};

// NOTE: all register name offset values use zero-indexed, array-based numbering
// This is in contrast to the address-offset numbering used in the TRM
// Basically, divide the TRM numbering by four to get the PAC numbering
pub fn pll0_set_freq(f: PllFreq) {
    let s = pac::sys_syscon_reg();

    let v1 = s.sys_syscfg_6().read().bits();
    let v2 = s.sys_syscfg_9().read().bits();
    let v3 = s.sys_syscfg_7().read().bits();
    println!("PLL0: {v1:08x} {v2:08x} {v3:08x}");

    // Turn off PD by setting the bit
    s.sys_syscfg_8().modify(|_, w| w.pll0_pd().set_bit());

    s.sys_syscfg_6().modify(|_, w| w.pll0_dacpd().set_bit());
    s.sys_syscfg_6().modify(|_, w| w.pll0_dsmpd().set_bit());

    s.sys_syscfg_9()
        .modify(|_, w| w.pll0_prediv().variant(f.prediv));

    s.sys_syscfg_7()
        .modify(|_, w| w.pll0_fbdiv().variant(f.fbdiv));

    // NOTE: The original code defines all postdiv1 values for
    // all PLLs and config to be 1 and then shifts them to 0.
    s.sys_syscfg_8().modify(|_, w| w.pll0_postdiv1().variant(0));
    // Turn on PD by clearing the bit
    s.sys_syscfg_8().modify(|_, w| w.pll0_pd().clear_bit());

    let v1 = s.sys_syscfg_9().read().bits();
    let v2 = s.sys_syscfg_8().read().bits();
    let v3 = s.sys_syscfg_11().read().bits();
    println!("PLL0: {v1:08x} {v2:08x} {v3:08x}");
}

// 2133 / 1066 yields:
// PLL1: 00b02603 55e00000 00c7a601
// PLL1: 042ba603 41e00000 00c7a60c
// vs low speed:
// PLL1: 00b02603 55e00000 00c7a601
// PLL1: 042a2603 41e00000 00c7a601
pub fn pll1_set_freq(f: PllFreq) {
    let s = pac::sys_syscon_reg();

    let v1 = s.sys_syscfg_9().read().bits();
    let v2 = s.sys_syscfg_10().read().bits();
    let v3 = s.sys_syscfg_11().read().bits();
    println!("PLL1: {v1:08x} {v2:08x} {v3:08x}");

    // Turn off PD by setting the bit
    s.sys_syscfg_10().modify(|_, w| w.pll1_pd().set_bit());
    s.sys_syscfg_9().modify(|_, w| w.pll1_dacpd().set_bit());
    s.sys_syscfg_9().modify(|_, w| w.pll1_dsmpd().set_bit());

    let frac = 0xe00000;
    s.sys_syscfg_10().modify(|_, w| w.pll1_frac().variant(frac));

    s.sys_syscfg_11()
        .modify(|_, w| w.pll1_prediv().variant(f.prediv));

    s.sys_syscfg_9()
        .modify(|_, w| w.pll1_fbdiv().variant(f.fbdiv));

    // NOTE: The original code defines all postdiv1 values for
    // all PLLs and config to be 1 and then shifts them to 0.
    s.sys_syscfg_10()
        .modify(|_, w| w.pll1_postdiv1().variant(0));

    // Turn on PD by clearing the bit
    s.sys_syscfg_10().modify(|_, w| w.pll1_pd().clear_bit());

    let v1 = s.sys_syscfg_9().read().bits();
    let v2 = s.sys_syscfg_10().read().bits();
    let v3 = s.sys_syscfg_11().read().bits();
    println!("PLL1: {v1:08x} {v2:08x} {v3:08x}");
}

pub fn pll2_set_freq(f: PllFreq) {
    let s = pac::sys_syscon_reg();

    let v1 = s.sys_syscfg_11().read().bits();
    let v2 = s.sys_syscfg_12().read().bits();
    let v3 = s.sys_syscfg_13().read().bits();
    println!("PLL0: {v1:08x} {v2:08x} {v3:08x}");

    // Turn PD off by setting the bit
    s.sys_syscfg_12().modify(|_, w| w.pll2_pd().set_bit());
    s.sys_syscfg_11().modify(|_, w| w.pll2_dacpd().set_bit());
    s.sys_syscfg_11().modify(|_, w| w.pll2_dsmpd().set_bit());

    s.sys_syscfg_13()
        .modify(|_, w| w.pll2_prediv().variant(f.prediv));

    s.sys_syscfg_11()
        .modify(|_, w| w.pll2_fbdiv().variant(f.fbdiv));

    // NOTE: The original code defines all postdiv1 values for
    // all PLLs and config to be 1 and then shifts them to 0.
    s.sys_syscfg_12()
        .modify(|_, w| w.pll2_postdiv1().variant(0));

    // Turn PD on by clearing the bit
    s.sys_syscfg_12().modify(|_, w| w.pll2_pd().clear_bit());

    let v1 = s.sys_syscfg_11().read().bits();
    let v2 = s.sys_syscfg_12().read().bits();
    let v3 = s.sys_syscfg_13().read().bits();
    println!("PLL0: {v1:08x} {v2:08x} {v3:08x}");
}
