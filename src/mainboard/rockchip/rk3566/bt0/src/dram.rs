// For related/similar SoCs see also U-Boot
// - arch/arm/include/asm/arch-rockchip/sdram_pctl_px30.h
// - arch/arm/include/asm/arch-rockchip/sdram_phy_px30.h
// - arch/arm/include/asm/arch-rockchip/sdram_px30.h
// - arch/arm/include/asm/arch-rockchip/sdram_rk3328.h
// - arch/arm/include/asm/arch-rockchip/sdram_common.h
// - drivers/ram/rockchip/sdram_pctl_px30.c
// - drivers/ram/rockchip/sdram_rv1126.c
// - drivers/ram/rockchip/sdram-rv1126-lpddr4-detect-528.inc

// PCTL = p... controller
// UPCTL = ultra low power ... ?

use util::mmio::{read32, write32};

use crate::arm::udelay;
use crate::i2c::{i2c_init, i2c_read};
use crate::mem_map::{
    CRU_NS_BASE, CRU_S_BASE, DDR_GRF_BASE, DDR_PHY_BASE, PMU_GRF_BASE, SRAM_BASE, SYS_SGRF_BASE,
    UPCTL2_BASE,
};

// SGRF: security subsystem (?)
// https://www.kernel.org/doc/Documentation/devicetree/bindings/soc/rockchip/grf.txt
// https://www.rockchip.fr/Rockchip%20RK3288%20TRM%20V1.0%20Part%201-System%20and%20System%20Control.pdf
const SYS_SGRF_0014: usize = SYS_SGRF_BASE + 0x0014;
const SYS_SGRF_0200: usize = SYS_SGRF_BASE + 0x0200;
const SYS_SGRF_0204: usize = SYS_SGRF_BASE + 0x0204;

const CRU_S_0208: usize = CRU_S_BASE + 0x0208;

const CRU_NS_046C: usize = CRU_NS_BASE + 0x046c;

/*
DDR Version V1337 20200218_resume
ln
start i2c rd
suspend_info:0x0, flag:0x20
LP4 MR12:0x4d,MR14:0x4d
LP4 MR12:0x4d,MR14:0x4d
LPDDR4, 324MHz
BW=32 Col=10 Bk=8 CS0 Row=16 CS=1 Die BW=16 Size=2048MB
change to: 324MHz
PWRCTL:0x40,stat:0x303
minca:0x80,ck:0x80,ab:0x80,0x80, min_ck:0x0
PWRCTL:0x0,stat:0x1
get tdqqs2dq:482 ps
change to: 528MHz
PWRCTL:0x40,stat:0x303
vref_ca:00000072
minca:0x76,ck:0x80,ab:0x80,0x80, min_ck:0xa
PWRCTL:0x0,stat:0x1
change to: 528MHz
PWRCTL:0x40,stat:0x303
vref_ca:00000072
minca:0x76,ck:0x80,ab:0x80,0x80, min_ck:0xa
PWRCTL:0x0,stat:0x1
change to: 528MHz(final freq)
PWRCTL:0x40,stat:0x303
vref_ca:00000072
minca:0x76,ck:0x80,ab:0x80,0x80, min_ck:0xa
PWRCTL:0x0,stat:0x1
osreg:0x1000e2c1,0x20000001
out
*/

// see also U-Boot arch/arm/include/asm/arch-rockchip/sdram_pctl_px30.h
// https://github.com/u-boot/u-boot/blob/master/arch/arm/include/asm/arch-rockchip/sdram_pctl_px30.h
const UPCTL2_MSTR: usize = UPCTL2_BASE + 0x0000;
const UPCTL2_STAT: usize = UPCTL2_BASE + 0x0004;
const UPCTL2_MR_CTRL0: usize = UPCTL2_BASE + 0x0010;
const UPCTL2_MR_CTRL1: usize = UPCTL2_BASE + 0x0014;
const UPCTL2_MR_STAT: usize = UPCTL2_BASE + 0x0018;
const UPCTL2_MSTR2: usize = UPCTL2_BASE + 0x0028;
const UPCTL2_PWRTMG: usize = UPCTL2_BASE + 0x0034;
const UPCTL2_HWLPCTL: usize = UPCTL2_BASE + 0x0038;
const UPCTL2_ZQCTL0: usize = UPCTL2_BASE + 0x0180;
const UPCTL2_DFI_MISC: usize = UPCTL2_BASE + 0x01b0;
const UPCTL2_SW_CTRL: usize = UPCTL2_BASE + 0x0320;
const UPCTL2_SW_STAT: usize = UPCTL2_BASE + 0x0324;
const UPCTL2_0404: usize = UPCTL2_BASE + 0x0404;

const UPCTL2_MR_WR_BUSY: u32 = 1;

const DDR_PHY_0000: usize = DDR_PHY_BASE + 0x0000;
const DDR_PHY_0038: usize = DDR_PHY_BASE + 0x0038;
const DDR_PHY_0044: usize = DDR_PHY_BASE + 0x0044;
const DDR_PHY_008C: usize = DDR_PHY_BASE + 0x008c;
const DDR_PHY_0094: usize = DDR_PHY_BASE + 0x0094;
const DDR_PHY_00AC: usize = DDR_PHY_BASE + 0x00ac;
const DDR_PHY_00C0: usize = DDR_PHY_BASE + 0x00c0;
// PLL
const DDR_PHY_00D0: usize = DDR_PHY_BASE + 0x00d0;
const DDR_PHY_00F0: usize = DDR_PHY_BASE + 0x00f0;
const DDR_PHY_00F4: usize = DDR_PHY_BASE + 0x00f4;
const DDR_PHY_00F8: usize = DDR_PHY_BASE + 0x00f8;
const DDR_PHY_01F4: usize = DDR_PHY_BASE + 0x01f4;
const DDR_PHY_0304: usize = DDR_PHY_BASE + 0x0304;

// https://www.synopsys.com/dw/ipdir.php?ds=dwc_ddr_universal_upctl2
fn upctl2_pre_init() {
    let v0248 = read32(UPCTL2_BASE + 0x0248);
    println!("{v0248:08x}");
    let v024c = read32(UPCTL2_BASE + 0x024C);
    println!("{v024c:08x}");
    let v = (v0248 >> 13) & 0xf;
    println!("{v:x}");
}

const MEGA: u32 = 1_000_000;

const PMU_GRF_OS2: usize = PMU_GRF_BASE + 0x0208;

// similar to U-Boot drivers/ram/rockchip/sdram_rv1126.c rkclk_set_dpll
fn clk_set_dpll(freq: u32) {
    let f_mhz = freq / MEGA;

    let postdiv1 = match f_mhz {
        ..=100 => 6,
        ..=200 => 4,
        ..800 => 2,
        _ => 1,
    };
    let postdiv2 = match f_mhz {
        ..=150 => 6,
        ..800 => 4,
        _ => 2,
    };
    let m3 = match freq {
        ..528000001 => 0x30000,
        _ => 0x30001,
    };

    const CRU_NS_MODE_CONF0: usize = CRU_NS_BASE + 0x00C0;
    write32(CRU_NS_MODE_CONF0, 0x000c_0000);
    write32(CRU_NS_BASE + 0x0128, 0x2000_2000);
    let fbdiv = (f_mhz * postdiv1 * postdiv2 / 24);
    write32(CRU_NS_BASE + 0x0020, 0x7fff_0000 | (postdiv2 << 12) | fbdiv);

    write32(CRU_NS_BASE + 0x0024, 0x11ff_1001 | (postdiv1 << 6));
    write32(CRU_NS_BASE + 0x0024, 0x2000_0000);

    for _ in 0..1000 {
        udelay(1);
        if read32(CRU_NS_BASE + 0x0024) & (1 << 10) != 0 {
            break;
        }
    }

    write32(CRU_NS_BASE + 0x00c0, 0x000c_0004);
}

struct RegVal {
    offset: u32,
    value: u32,
}

const XX_PARAMS_0_PHY: [RegVal; 4] = [
    RegVal {
        offset: 0x00,
        value: 0x0000_1FA7,
    }, //
    RegVal {
        offset: 0x08,
        value: 0x0000_0000,
    }, //
    RegVal {
        offset: 0x0c,
        value: 0x0500_0000,
    }, //
    RegVal {
        offset: 0x10,
        value: 0x0500_0000,
    }, //
];

// similar to U-Boot drivers/ram/rockchip/sdram_rv1126.c phy_pll_set
fn phy_pll_set(freq: u32, p2: u32) {
    // maybe divider value & enable-bit
    let (v1, v2) = match freq / MEGA {
        ..51 => (5, 1),
        ..101 => (4, 1),
        ..201 => (3, 1),
        ..401 => (2, 1),
        ..801 => (1, 1),
        _ => (0, 0),
    };
    let s1 = (p2 * 8 + 4) & 0x1f;
    let s2 = (p2 * 8 + 3) & 0x1f;

    let v = read32(DDR_PHY_00D0);
    let mask = (0b111 << s1) | (1 << s2);
    let v = v & !mask | (v1 << s1) | (v2 << s2);
    write32(DDR_PHY_00D0, v);
}

const DDR_GRF_CTRL0: usize = DDR_GRF_BASE + 0x0000;
const DDR_GRF_CTRL1: usize = DDR_GRF_BASE + 0x0004;
const DDR_GRF_CTRL2: usize = DDR_GRF_BASE + 0x0008;
const DDR_GRF_CTRL3: usize = DDR_GRF_BASE + 0x000c;
const DDR_GRF_CTRL4: usize = DDR_GRF_BASE + 0x0010;

const DDR_GRF_SPLIT_CON: usize = DDR_GRF_BASE + 0x0014;
const DDR_GRF_LP_CON: usize = DDR_GRF_BASE + 0x0020;

const DDR_GRF_STATUS00: usize = DDR_GRF_BASE + 0x0100;
const DDR_GRF_STATUS01: usize = DDR_GRF_BASE + 0x0104;
const DDR_GRF_STATUS02: usize = DDR_GRF_BASE + 0x0108;
const DDR_GRF_STATUS03: usize = DDR_GRF_BASE + 0x010c;
const DDR_GRF_STATUS04: usize = DDR_GRF_BASE + 0x0100;
const DDR_GRF_STATUS05: usize = DDR_GRF_BASE + 0x0114;
const DDR_GRF_STATUS06: usize = DDR_GRF_BASE + 0x0118;
const DDR_GRF_STATUS07: usize = DDR_GRF_BASE + 0x011c;
const DDR_GRF_STATUS08: usize = DDR_GRF_BASE + 0x0120;
const DDR_GRF_STATUS09: usize = DDR_GRF_BASE + 0x0124;
// NOTE: taken from manual, apparently some offsets are skipped here
const DDR_GRF_STATUS10: usize = DDR_GRF_BASE + 0x0130;
const DDR_GRF_STATUS11: usize = DDR_GRF_BASE + 0x0134;
const DDR_GRF_STATUS12: usize = DDR_GRF_BASE + 0x0138;

fn get_funny_bits() -> (u32, u32) {
    let vtt = read32(DDR_GRF_CTRL3);
    // Extract bits 8..15. The & 0xff is technically not necessary since we
    // do another extraction hereafter, taking a pair of bits at idx * 2.
    let vpx = (vtt >> 8) & 0xff;
    println!("DDR_GRF_000C bits 15..8: {vpx:08b} (full reg val: {vtt:08x})");

    let mut vl = 0;
    let mut vxx = 0;
    for idx in 0..4 {
        // check on bits 8..9, 10..11, 12..13, 14..15 in respective round
        match (vpx >> (2 * idx)) & 0b11 {
            0 => {
                vl = idx;
            }
            1 => {
                vxx = idx;
            }
            _ => {}
        }

        println!("round {idx}: {vl},{vxx}");
    }
    (vl, vxx)
}

// FIXME: vendor code also refers to +0x24, +0x3c, +0x30... but that overlaps
// with other instances of this struct in the vendor code. What's up with that?
struct Cfg {
    p0: u32, // + 0x00
    p1: u32, // + 0x04
    p2: u32, // + 0x08
    p3: u32, // + 0x0c
    p4: u32, // + 0x10
    p5: u32, // + 0x14
    p6: u32, // + 0x18
    p7: u32, // + 0x1c
    p8: u32, // + 0x20
}

const CFG_0X11: Cfg = Cfg {
    p0: 0x0014_4210,
    p1: 0x0021_0210,
    p2: 0x0000_0000,
    p3: 0x2221_2121,
    p4: 0x2221_2121,
    p5: 0x000C_A778,
    p6: 0x0014_D14D, // + 0x18; (>> 12) => 14d < 144 ? NO
    p7: 0x0000_030F,
    p8: 0x0000_030F,
};

const CFG_0X20: Cfg = Cfg {
    p0: 0x0014_4210, // + 0x24
    p1: 0x0021_0210, // + 0x28
    p2: 0x0000_0000, // + 0x2c
    p3: 0x2225_2525, // + 0x30
    p4: 0x2225_2525,
    p5: 0x000C_8B78,
    p6: 0x0027_1271,
    p7: 0x0001_010E,
    p8: 0x0001_010E,
};

const CFG_0X29: Cfg = Cfg {
    p0: 0x0014_4210,
    p1: 0x0021_0210,
    p2: 0x0000_0000,
    p3: 0x2227_2525,
    p4: 0x2227_2525,
    p5: 0x000C_9478,
    p6: 0x0014_D14D,
    p7: 0x000F_010F,
    p8: 0x000F_010F,
};

const CFG_0X41: Cfg = Cfg {
    p0: 0x0014_4210,
    p1: 0x0021_0210,
    p2: 0x0000_0000,
    p3: 0x2824_241D,
    p4: 0x2824_241D,
    p5: 0x01E0_3C50,
    p6: 0x000C_8320,
    p7: 0x0000_0000,
    p8: 0x0000_0000,
};

// NOTE: Those structs do not align all too well in the vendor code.
// There are potential overlaps, probably related to pointer casts in source,
// with optimizations in the build process dropping parts of then-free memory.
fn cfg_for_dram_type(dram_type: u32) -> Cfg {
    match dram_type {
        // 0 => CFG_0X14, // does not really exist, vendor code is buggy (?)
        3 => CFG_0X11,
        6 => CFG_0X29,
        7 => CFG_0X20,
        8 => CFG_0X41,
        // FIXME: This should happen much earlier. No need to carry it around.
        // It depends on parameters currently evaluated at runtime; we can just
        // do this at build time.
        _ => panic!("DRAM type {dram_type} not supported!"),
    }
}

type DramMx = [u32; 24];

// NOTE: This is all LE, so the upper half is what matters in upctl2_phy_smth.
const DRAM_T3_MX: DramMx = [
    0x01F4_0001,
    0x00FA_0002,
    0x00A7_0003,
    0x007D_0004,
    0x0064_0005,
    0x0053_0006,
    0x0047_0007,
    0x003F_0008,
    0x0038_0009,
    0x0032_000A,
    0x002D_000B,
    0x0029_000C,
    0x0026_000D,
    0x0024_000E,
    // THIS: no longer < 0x21
    0x0021_000F,
    0x001F_0018,
    0x001D_0019,
    0x001C_001A,
    0x001A_001B,
    0x0019_001C,
    0x0018_001D,
    0x0017_001E,
    0x0016_001F,
    0x0000_0000, // last value unused
];

// TODO: enum for dram_type
fn upctl2_phy_smth(dram_freq: u32, dram_type: u32, smth: bool) {
    let cfg = cfg_for_dram_type(dram_type);

    // Those really depend on dram_freq and cfg; shortcut taken here.
    let p7_8 = cfg.p8;
    let p5 = (cfg.p5 >> 29) & 1;
    let p3_4 = cfg.p4;

    let mut params: [u16; 12] = [
        0, 0, 0, 0, // first 4 values are prefilled
        0, 0, 0, 0, // remaining 8 values are
        0, 0, 0, 0, // being determind later
    ];

    // all three are 0x21
    params[0] = (p3_4 >> 16) as u8 as u16;
    params[1] = (p3_4 >> 8) as u8 as u16;
    params[2] = p3_4 as u8 as u16;

    // XXX
    // let dram_type_x = dram_type - 7; // fffffffc

    // NOTE: conditions skipped
    let p4_byte3 = cfg.p4 >> 24;
    let params_3 = 0 as u8;
    let p5_bit27 = (cfg.p5 >> 27) & 1;

    // TODO: tweak this
    // 4 iterations
    for o in (0x0300..0x0a80).step_by(0x180) {
        let r = DDR_PHY_BASE + o + 8;
        let v = read32(r);
        write32(r, v & 0xffff_fdff);
    }

    // conditions omitted
    let m0 = 0;
    let m1 = 0;
    let m2 = 0;

    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v | (1 << 1));
    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v & !(1 << 3));
    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v | (1 << 3));
    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v & !(1 << 1));

    let mx = match dram_type {
        3 => DRAM_T3_MX,
        7 => todo!(), // DRAM_T7_MX,
        8 => todo!(), // DRAM_T8_MX,
        _ => todo!(), // DRAM_TX_MX,
    };

    // TODO: calculate other params! precalc..?

    // FIXME: + 0x30 / + 0x2c
    // let xx1 = if params_3 == 0 { cfg.p12 } else { cfg.p11 };
    let xx1 = if params_3 == 0 {
        CFG_0X20.p3
    } else {
        CFG_0X20.p2
    };
    let xx2 = ((xx1 & 0x3ff) << 9) / 1000;
    let xx3 = 0x100;

    let vxm = ((params[8] as u32) << 8) | (params[4] as u32);

    let v = (((params[9] as u32) << 24) | ((params[5] as u32) << 16)) | vxm;
    write32(DDR_PHY_00F4, v);

    let v = read32(DDR_PHY_00F8) & 0xffff_e0e0 | vxm;
    write32(DDR_PHY_00F8, v);

    let v = read32(DDR_PHY_00F0) & 0xffff_e0e0;
    let v = v | (p7_8 & 0xff00) | ((p7_8 >> 16) & 0xff);
    write32(DDR_PHY_00F0, v);

    if m1 == 0 {
        params[7] = 0;
    }
    if m0 == 0 {
        params[11] = 0;
    }

    // TODO: tweak this
    // 4 iterations
    for o in (0x0300..0x0a80).step_by(0x180) {
        let r = DDR_PHY_BASE + o + 4;
        let v = ((params[10] as u32) << 24)
            | ((params[6] as u32) << 16)
            | ((params[11] as u32) << 8)
            | params[7] as u32;
        write32(r, v);
        let r = DDR_PHY_BASE + o;
        let v = read32(r) & 0x007f_e07f;
        let v = v | ((p5 << 7) ^ 0x80) | ((p7_8 & 0xff) << 8) | (xx2 << 23);
        write32(r, v as u32);
    }

    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v | 0x80);
    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v & !0x80);

    let v = read32(DDR_PHY_00F8);
    write32(DDR_PHY_00F8, v & 0xfe00_ffff | (xx3 << 16));

    // p0 & 0xfff  0x210
    let v1 = if CFG_0X20.p0 & 0xfff < dram_freq {
        CFG_0X20.p2 // 0x0
    } else {
        CFG_0X20.p3 // 0x2225_2525
    };
    // 0x94 (148)
    let v1 = (v1 >> 14) & 0x3ff;

    // p6 & 0xfff = 0x14d
    let v2 = if cfg.p6 & 0xfff < dram_freq {
        CFG_0X20.p2
    } else {
        CFG_0X20.p3
    };
    // 0x149
    let v2 = (v2 >> 10) & 0x3ff;

    // 0x25b (603)
    let v2 = v2 * 11 / 6;

    let v2 = match v2 {
        ..150 => 0,
        ..450 => (v2 - 150) / 6,
        ..630 => ((v2 - 329) / 6) | 0x40, // we should be here
        _ => 114,
    };

    let v1 = match v1 {
        ..150 => 0, // we should be here
        ..450 => (v1 - 150) / 6,
        ..630 => ((v1 - 329) / 6) | 0x40,
        _ => 114,
    };

    write32(UPCTL2_SW_CTRL, 0);

    let p4_sby_0x1000 = if smth { 0x1000 } else { 0 };
    let o_base = p4_sby_0x1000 * 2;
    let o1 = o_base + 0x00e8;
    let o2 = o_base + 0x00ec;

    let v = read32(UPCTL2_BASE + o1);
    write32(UPCTL2_BASE + o1, (v & 0xffff_0000) | v2);

    let v = read32(UPCTL2_BASE + o2);
    write32(UPCTL2_BASE + o2, (v & 0xffff_0000) | v1);

    upctl2_sw_set_ack();

    let o3 = o_base + 0x00dc;
    let v = read32(UPCTL2_BASE + o3);
    let v = v & 0xfd99;

    let p4_byte3 = cfg.p4 >> 24;

    let v3 = if p4_byte3 == 0x22 { v | (1 << 1) } else { v };
    // TODO: if !smth ...

    write32(UPCTL2_SW_CTRL, 0);

    let v = read32(UPCTL2_BASE + o3);
    write32(UPCTL2_BASE + o3, (v & 0xffff_0000) | v3);

    upctl2_sw_set_ack();
}

// drivers/ram/rockchip/sdram_rv1126.c sw_set_ack
fn upctl2_sw_set_ack() {
    write32(UPCTL2_SW_CTRL, 1);
    while read32(UPCTL2_SW_STAT) & 1 == 0 {}
}

// set_ds_odt ?
fn ddr_xxx(enable_ecc: bool) {
    // TODO: These values come from structs at the offsets encoded in the
    // variable names. Should we make those structs or simple parameters?
    // U-Boot arm/include/asm/arch-rockchip/sdram_common.h
    let s_0000 = 0x1; // rank
    let s_0004 = 0xc; // col
    let s_0008 = 0x3; // bank number, power of 2, i.e., 2^3=8
    let s_000c = 0x1; // channel bus width, 1 means 16bit
    let s_0010 = 0x0; // die bus width, 0 means 8bit
    let s_0014 = 0x0; // row 3_4, 0 means normal die, power of 2
    let s_0018 = 0x10; // CS0 row
    let s_001c = 0x10; // CS1 row

    // DRAM frequency
    let dram_freq = 0x144;
    // apparently s_0068 encodes the DRAM type
    let dram_type = 0x3;
    let s_007c = XX_PARAMS_0_UPCTL2[0].value;

    println!("ddr_xxx");
    write32(DDR_GRF_CTRL0, 0x20000);

    clk_set_dpll((dram_freq * MEGA) / 2);

    write32(SYS_SGRF_0014, 0x0b00_0b00);
    write32(CRU_S_0208, 0x0002_0002);
    write32(CRU_NS_046C, 0x0180_0180);
    udelay(10);
    write32(SYS_SGRF_0014, 0x0b00_0b00);
    write32(CRU_S_0208, 0x0002_0002);
    write32(CRU_NS_046C, 0x0180_0100);

    // TODO: What is the possible value range?
    // This check may be unnecessary.
    if dram_type <= 8 {
        let m1 = if dram_type == 8 { 7 } else { dram_type };

        const XX: u32 = 0xe400_00e4;
        let x = (XX >> ((m1 & 0b11) << 3)) & 0xff;
        let v = if x == 0xe4 {
            0xff80_e400
        } else {
            0xff80_0080 | (x << 8)
        };

        println!("DDR_GRF_000C: write {v:08x}");
        write32(DDR_GRF_CTRL3, v);
    }

    phy_pll_set(dram_freq * MEGA, 0);

    // TODO: other rounds have different params / sizes thereof
    // NOTE: this looks similar to PHY cfg functions for other PHYs
    for p in XX_PARAMS_0_PHY.iter() {
        let r = DDR_PHY_BASE + p.offset as usize;
        let v = if p.value * 4 < 9 {
            (read32(r) & 0xc0ffffff) | p.value
        } else {
            p.value
        };
        write32(r, v)
    }

    // Extracted here to keep the flow simpler
    let (vl, vxx) = get_funny_bits();

    let v = read32(DDR_PHY_0000) & 0xffffe0ff;
    let vx = match s_000c {
        1 => v | ((1 << vl) | (1 << vxx)) << 8,
        2 => v | 0x0f00,
        _ => v | 0x100 << vl,
    };
    let mut vxo = if enable_ecc { vx } else { vx | 0x1000 };

    if s_0000 == 4 {
        vxo |= 0x0010_0000;
        let v = read32(DDR_PHY_0038);
        write32(DDR_PHY_0038, v | 1 << 1);
    }
    write32(DDR_PHY_BASE, vxo);

    // TODO: tweak this
    // Each loop has 4 iterations
    match dram_type {
        0 | 3 | 6 => {
            for o in (0x0300..0x0a80).step_by(0x180) {
                let r = DDR_PHY_BASE + o + 8;
                let v = read32(r);
                write32(r, v & 0xffff_fdff);
            }
        }
        7 => {
            let v = read32(DDR_PHY_0038);
            write32(DDR_PHY_0038, (v & 0xffff_07ff) | 0x0000_5800);
            for o in (0x0300..0x0a80).step_by(0x180) {
                let r = DDR_PHY_BASE + o;
                let v = read32(r);
                write32(r, (v & 0xffff_ff9f) | 0x40);
            }
        }
        8 => {
            for o in (0x0300..0x0a80).step_by(0x180) {
                let r = DDR_PHY_BASE + o + 8;
                let v = read32(r);
                write32(r, v | 0x100);
                let r = DDR_PHY_BASE + o;
                let v = read32(r);
                write32(r, (v & 0xffff_ff9f) | 0x40);
            }
        }
        _ => {}
    }

    let v = read32(DDR_PHY_00C0);
    write32(DDR_PHY_00C0, v | 1);
    if s_007c & (1 << 10) != 0 {
        let v = read32(DDR_PHY_00C0);
        write32(DDR_PHY_00C0, v | 0x0006_0000);
    }
    let v = read32(DDR_PHY_00AC);
    write32(DDR_PHY_00AC, v | 0x10);
    let v = read32(DDR_PHY_0044);
    write32(DDR_PHY_0044, v & 0x3fffffff);

    write32(CRU_NS_046C, 0x0180_0000);
    write32(SYS_SGRF_0014, 0x0b00_0300);
    write32(CRU_S_0208, 0x0002_0000);
    write32(CRU_NS_046C, 0x0180_0000);

    // NOTE: params list needs to be a param here as well
    upctl2_config(&XX_PARAMS_0_UPCTL2, 0x005d, 0x000d);

    let v = read32(UPCTL2_0404);
    write32(UPCTL2_0404, v | 0x0001_0000);

    // drivers/ram/rockchip/sdram_rv1126.c
    // static int sdram_init_(/*...*/)
    // set frequency_mode
    let v = read32(UPCTL2_MSTR);
    write32(UPCTL2_MSTR, v | (1 << 29));
    // set target_frequency to Frequency 0
    let v = read32(UPCTL2_MSTR2);
    write32(UPCTL2_MSTR2, v & !(0b11));

    upctl2_phy_smth(dram_freq, dram_type, false);

    // 0xd
    let s_000c_0004 = s_000c + s_0004;

    // 3 * 0x20 | 0 | (-3) = 0xffff_fffc
    let vt = ((s_0000 - 1) << 8) | ((s_0018 - 13) << 5) | (s_000c_0004 - 10);

    let vxx = if s_0008 == 3 { vt | 8 } else { vt };

    let idx = find_index(vxx);

    // NOTE: original code mutates global variable
    let s_0030 = idx.unwrap_or_else(|| {
        if s_0008 == 3 && s_000c_0004 == 10 {
            14
        } else if s_0000 != 1 || s_0008 != 3 || s_0018 > 17 || s_000c_0004 != 13 {
            // NOTE: This should never happen.
            panic!("calculcate DDR config error")
        } else {
            17
        }
    });

    // TODO: more logic
    // s_0018
    let o = 0x0218 + 4;
    let r = UPCTL2_BASE + o;
    let v = read32(r);
    write32(r, v | (0xf << 8));
    let v = read32(r);
    write32(r, v | (0xf << 0));

    if s_0000 == 1 {
        let r = UPCTL2_BASE + 0x200;
        let v = read32(r);
        write32(r, v | 0x1f);
    }

    // also in U-Boot drivers/ram/rockchip/sdram_rv1126.c sdram_init_
    let r = UPCTL2_DFI_MISC;
    let v = read32(r);
    write32(r, v | (1 << 5) | (1 << 4));

    write32(SYS_SGRF_0014, 0x0b00_0000);
    write32(CRU_S_0208, 0x0002_0000);
    write32(CRU_NS_046C, 0x0180_0000);

    // bits 0..2: OPERATING_MODE
    // - 0 = init
    // - 1 = normal
    // - 2 = PD (?)
    // - 3 = self-refresh
    // bits 4..5: SELFREF_TYPE
    // - 2 = "not auto"
    while read32(UPCTL2_STAT) & 0b111 == 0 {}

    // The following appears to be some kind of measurement, yielding different
    // values for different runs.

    // dram_freq = 0x144
    let v1 = 500_000 / dram_freq; // 1543
    let v2 = 10_000 / v1; // 6

    let v = read32(DDR_PHY_01F4) >> 24;
    println!("DDR_PHY_01F4: {v:08x}");

    #[allow(arithmetic_overflow)]
    let v3 = if v < 0x41 {
        (v2 + 1) * 0xffff_ffc0 + 0xc80 // 0xac0
    } else {
        (50 - (v2 + 1)) * v // 43 * v
    };
    let vf = (v3 / 100) & 0x7f; // possibly 27 (0x1b)

    // real values seen: 0x5c, 0xd6, 0xd7, 0xd8
    println!("vf: {vf:02x}");

    const BLOCK_SIZE: usize = 0x180;
    const BLOCK_COUNT: usize = 4;
    // NOTE: s_0000 apparently could be hardcoded at build time and reflects the
    // design/variant of the PHY.
    for i in 0..s_0000 {
        let o = match i {
            0 => 0x33c,
            1 => 0x35c,
            2 => 0x418,
            _ => 0x438,
        };
        let m = 0x80ff_80ff;
        // NOTE: inclusive
        for o in (o..=o + BLOCK_COUNT * BLOCK_SIZE).step_by(BLOCK_SIZE) {
            let r = DDR_PHY_BASE + o;
            let v = read32(r);
            // println!("{r:08x}: {v:08x}");
            write32(r, (v & m) | (vf << 24) | (vf << 8));
        }
    }

    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v | (1 << 2));
    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v & !(1 << 2));

    let mr12 = upctl2_read_mr(1, 12, 7);
    let mr14 = upctl2_read_mr(1, 14, 7);
    // we expect 0x4d for both
    println!("LP4  MR12: {mr12:02x}  MR14: {mr14:02x}");

    if DEBUG {
        // WHOOPSIES
        assert_eq!(mr12, 0x4d);
        assert_eq!(mr14, 0x4d);
    }

    // TODO: draw the rest of the owl 🦉🖌️

    println!("ddr_xxx done");
}

// U-Boot drivers/ram/rockchip/sdram_pctl_px30.c pctl_write_mr
fn upctl2_write_mr(rank: u32, mr: u32, val: u8, p4: u32) {
    println!("upctl2_write_mr rank {rank} mr {mr} val {val}");

    while read32(UPCTL2_MR_STAT) & UPCTL2_MR_WR_BUSY != 0 {}

    if (p4 == 0 || p4 == 3) {
        // DDR3 / DDR4
        write32(UPCTL2_MR_CTRL0, (mr << 12) | (rank << 4));
        write32(UPCTL2_MR_CTRL1, val as u32);
    } else {
        write32(UPCTL2_MR_CTRL0, rank << 4);
        write32(UPCTL2_MR_CTRL1, (mr << 8) | (val as u32));
    }

    // NOTE: same here as upctl2_prep_poll_mr
    // looks like a control + status register
    let v = read32(UPCTL2_MR_CTRL0);
    write32(UPCTL2_MR_CTRL0, v | (1 << 31));
    // wait for bit to be cleared
    while read32(UPCTL2_MR_CTRL0) & (1 << 31) != 0 {}

    while read32(UPCTL2_MR_STAT) & UPCTL2_MR_WR_BUSY != 0 {}
}

// FIXME: we only get 0 :(
// pctl_read_mr
fn upctl2_read_mr(rank: u32, mr: u32, mx: u32) -> u8 {
    // NOTE: We might move this out, since parameters are just forwarded.
    upctl2_prep_poll_mr(rank, mr);

    let v = read32(DDR_GRF_STATUS00);
    println!("upctl2_get_xxxxx DDR_GRF_0100: {v:08x}");
    let v = if mx < 9 {
        let v2 = read32(DDR_GRF_STATUS01);
        println!("upctl2_get_xxxxx DDR_GRF_0104: {v2:08x}");
        v2 >> 8
    } else {
        v
    };
    v as u8
}

const DEBUG: bool = true;

/*
 * drivers/ram/rockchip/sdram_pctl_px30.c pctl_read_mr()
 *
 * rank = 1: cs0
 * rank = 2: cs1
 */
fn upctl2_prep_poll_mr(rank: u32, mr: u32) {
    println!("upctl2_prep_poll_mr {rank} {mr}");
    write32(UPCTL2_MR_CTRL0, (rank << 4) | 1);
    write32(UPCTL2_MR_CTRL1, mr << 8);

    let v = read32(UPCTL2_MR_CTRL0);
    write32(UPCTL2_MR_CTRL0, v | (1 << 31));
    // wait for bit to be cleared
    while read32(UPCTL2_MR_CTRL0) & (1 << 31) != 0 {}

    while read32(UPCTL2_MR_STAT) & UPCTL2_MR_WR_BUSY != 0 {}
}

// 0..=8
fn find_index(vxx: u32) -> Option<usize> {
    for (i, c) in DATA.iter().enumerate() {
        // NOTE: ^ is XOR
        if (c ^ vxx) & 0x1f == 0 && // asd
            (vxx & 0xe0) <= (c & 0xe0) && // asd
            (vxx & 0x100) <= (c & 0x100)
        {
            return Some(i);
        }
    }
    None
}

const DATA: [u32; 9] = [
    0x00AA, 0x01A9, 0x018A, 0x016B, //
    0x014C, 0x005C, 0x0099, 0x009A, //
    0x007B,
];

fn upctl2_config(reg_vals: &[RegVal], p1: u32, p2: u32) {
    fill_regs(UPCTL2_BASE, reg_vals);

    let v = read32(UPCTL2_PWRTMG);
    let m = 0xff00_ffe0;
    write32(UPCTL2_PWRTMG, v & m | ((p1 & 0xff) << 16) | p2 & 0x1f);

    let v = read32(UPCTL2_HWLPCTL);
    let m = 0xf000_ffff;
    write32(UPCTL2_HWLPCTL, v & m | 0x50000);

    let v = read32(UPCTL2_ZQCTL0);
    write32(UPCTL2_ZQCTL0, v | 0x80000000);
}

fn fill_regs(base: usize, data: &[RegVal]) {
    for e in data {
        write32(base + e.offset as usize, e.value);
    }
}

// NOTE: The first value here has been changed to 0x4304_1401 ( | 0x400 ) in
// the first round of dram_init_main.
const XX_PARAMS_0_UPCTL2: [RegVal; 25] = [
    RegVal {
        offset: 0x00,
        // NOTE: value overridden in control flow in dram_init_main, condition
        // for setting or clearing bit 10 (0x400) seems to be a fixed constant.
        value: 0x43041001 | (1 << 10),
    },
    RegVal {
        offset: 0x64,
        value: 0x270039,
    },
    RegVal {
        offset: 0xD0,
        value: 0x20051,
    },
    RegVal {
        offset: 0xD4,
        value: 0x210000,
    },
    RegVal {
        offset: 0xD8,
        value: 0x100,
    },
    RegVal {
        offset: 0xDC,
        value: 0x3100000,
    },
    RegVal {
        offset: 0xE0,
        value: 0x0,
    },
    RegVal {
        offset: 0xE4,
        value: 0x90000,
    },
    RegVal {
        offset: 0xF4,
        value: 0xF022F,
    },
    RegVal {
        offset: 0x100,
        value: 0x7090B06,
    },
    RegVal {
        offset: 0x104,
        value: 0x50209,
    },
    RegVal {
        offset: 0x108,
        value: 0x3030307,
    },
    RegVal {
        offset: 0x10C,
        value: 0x202006,
    },
    RegVal {
        offset: 0x110,
        value: 0x3020203,
    },
    RegVal {
        offset: 0x114,
        value: 0x3030202,
    },
    RegVal {
        offset: 0x120,
        value: 0x903,
    },
    RegVal {
        offset: 0x180,
        value: 0x800020,
    },
    RegVal {
        offset: 0x184,
        value: 0x0,
    },
    RegVal {
        offset: 0x190,
        value: 0x7010001,
    },
    RegVal {
        offset: 0x198,
        value: 0xA000101,
    },
    RegVal {
        offset: 0x1A0,
        value: 0xC0400003,
    },
    RegVal {
        offset: 0x240,
        value: 0x6000600,
    },
    RegVal {
        offset: 0x244,
        value: 0x201,
    },
    RegVal {
        offset: 0x250,
        value: 0x1F00,
    },
    RegVal {
        offset: 0x490,
        value: 0x1,
    },
];

// https://www.rockchip.fr/RK809%20datasheet%20V1.01.pdf
const PMIC_ADDR: u8 = 0x20;

const DUMP_OTP_NS: bool = false;

pub fn init() {
    if DUMP_OTP_NS {
        _ = crate::otp::read_ns(0, 0x40);
    }
    // dram_init_start
    // read first 4 bytes encoding SoC ID (524b3566 = RK3566)
    if crate::otp::read_ns(0, 0x1).is_err() {
        panic!("OTP error");
    }

    write32(SYS_SGRF_0200, 0xffff_8280);
    write32(SYS_SGRF_0204, 0xffff_1240);

    // remapping ? register is PMU_SGRF_SOC_CON1

    // dram_init_main
    i2c_init();

    // FIXME: first read always gets 0xff regardless of the register we want
    _ = i2c_read(PMIC_ADDR, 0x0);

    // gas_gauge_DATA7: initial value 0x00
    let r = i2c_read(PMIC_ADDR, 0xa4);
    // PMIC_POWER_SLP_EN1: initial value OTP (we get 0xf6)
    let r = i2c_read(PMIC_ADDR, 0xb6);
    println!("flag: {:02x}", r & 0x20);

    // Why is this being done here?
    crate::otp::otp_phy_init();

    // XXX: ignore for now
    if false {
        let v0208 = read32(PMU_GRF_OS2);
        // we get 0 (reset value), would be non-zero for a second run...
        println!("PMU_GRF_OS2: {v0208:08x}");
        if v0208 == 0 {
            println!("PMU_GRF_OS2 is 0, whoops");
        } else {
            upctl2_pre_init();
        }
    }

    // TODO: only first round?
    let enable_ecc = true;
    ddr_xxx(enable_ecc);
}
