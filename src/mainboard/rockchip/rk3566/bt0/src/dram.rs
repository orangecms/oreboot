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

const CRU_S_CLK_SEL_CFG66: usize = CRU_S_BASE + 0x0208;

const CRU_NS_MODE_CFG0: usize = CRU_NS_BASE + 0x00c0;
const CRU_NS_VPLL_CFG0: usize = CRU_NS_BASE + 0x00a0;
const CRU_NS_SOFT_RESET_CFG27: usize = CRU_NS_BASE + 0x046c;

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

// https://www.synopsys.com/dw/ipdir.php?ds=dwc_ddr_universal_upctl2
// Synopsys Enhanced Universal DDR Protocol Controller (uPCTL2)
// see also U-Boot arch/arm/include/asm/arch-rockchip/sdram_pctl_px30.h
// https://github.com/u-boot/u-boot/blob/master/arch/arm/include/asm/arch-rockchip/sdram_pctl_px30.h
const UPCTL2_MSTR: usize = UPCTL2_BASE + 0x0000;
const UPCTL2_STAT: usize = UPCTL2_BASE + 0x0004;
const UPCTL2_MR_CTRL0: usize = UPCTL2_BASE + 0x0010;
const UPCTL2_MR_CTRL1: usize = UPCTL2_BASE + 0x0014;
const UPCTL2_MR_STAT: usize = UPCTL2_BASE + 0x0018;
const UPCTL2_MSTR2: usize = UPCTL2_BASE + 0x0028;
const UPCTL2_POWER_CTRL: usize = UPCTL2_BASE + 0x0030;
const UPCTL2_POWER_TIMING: usize = UPCTL2_BASE + 0x0034;
const UPCTL2_HWLP_CTRL: usize = UPCTL2_BASE + 0x0038;
const UPCTL2_REFRESH_CTRL0: usize = UPCTL2_BASE + 0x0050;
const UPCTL2_REFRESH_CTRL1: usize = UPCTL2_BASE + 0x0054;
const UPCTL2_REFRESH_CTRL2: usize = UPCTL2_BASE + 0x0058;
// NOTE: The following two are mixed up in U-Boot. Or are they?
const UPCTL2_REFRESH_CTRL3: usize = UPCTL2_BASE + 0x005c;
const UPCTL2_REFRESH_CTRL4: usize = UPCTL2_BASE + 0x0060;
const UPCTL2_REFRESH_TIMING: usize = UPCTL2_BASE + 0x0064;
const UPCTL2_INIT0: usize = UPCTL2_BASE + 0x00d0;
const UPCTL2_INIT1: usize = UPCTL2_BASE + 0x00d4;
const UPCTL2_INIT2: usize = UPCTL2_BASE + 0x00d8;
const UPCTL2_INIT3: usize = UPCTL2_BASE + 0x00dc;
const UPCTL2_INIT4: usize = UPCTL2_BASE + 0x00e0;
const UPCTL2_INIT5: usize = UPCTL2_BASE + 0x00e4;
const UPCTL2_INIT6: usize = UPCTL2_BASE + 0x00e8;
const UPCTL2_INIT7: usize = UPCTL2_BASE + 0x00ec;
const UPCTL2_ZQ_CTRL0: usize = UPCTL2_BASE + 0x0180;
const UPCTL2_DFI_MISC: usize = UPCTL2_BASE + 0x01b0;
// ADDRMAP0..11
const UPCTL2_ADDR_MAP_BASE: usize = UPCTL2_BASE + 0x200;
const UPCTL2_0248: usize = UPCTL2_BASE + 0x0248;
const UPCTL2_024C: usize = UPCTL2_BASE + 0x024c;
const UPCTL2_DBG_CMD: usize = UPCTL2_BASE + 0x030c;
const UPCTL2_DBG_STAT: usize = UPCTL2_BASE + 0x0310;
const UPCTL2_SW_CTRL: usize = UPCTL2_BASE + 0x0320;
const UPCTL2_SW_STAT: usize = UPCTL2_BASE + 0x0324;
const UPCTL2_PCFGR_N: usize = UPCTL2_BASE + 0x0404;

const UPCTL2_MR_WR_BUSY: u32 = 1;

const DDR_PHY_0000: usize = DDR_PHY_BASE + 0x0000;
const DDR_PHY_0004: usize = DDR_PHY_BASE + 0x0004;
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
const DDR_PHY_020C: usize = DDR_PHY_BASE + 0x020C;

const DDR_PHY_0300: usize = DDR_PHY_BASE + 0x0300;
const DDR_PHY_0304: usize = DDR_PHY_BASE + 0x0304;

const DDR_PHY_0448: usize = DDR_PHY_BASE + 0x0448;

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

fn upctl2_pre_init() {
    let v0248 = read32(UPCTL2_0248);
    println!("{v0248:08x}");
    let v024c = read32(UPCTL2_024C);
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

    write32(CRU_NS_MODE_CFG0, 0x000c_0000);
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

type PhyCfg = [RegVal; 4];

const PHY_CFG0: PhyCfg = [
    RegVal {
        offset: 0x0000,
        value: 0x0000_1fa7,
    }, //
    RegVal {
        offset: 0x0008,
        value: 0x0000_0000,
    }, //
    RegVal {
        offset: 0x000c,
        value: 0x0500_0000,
    }, //
    RegVal {
        offset: 0x0010,
        value: 0x0500_0000,
    }, //
];

const PHY_CFG3: PhyCfg = [
    RegVal {
        offset: 0x0000,
        value: 0x0000_1fd7,
    }, //
    RegVal {
        offset: 0x0008,
        value: 0x0000_0000,
    }, //
    RegVal {
        offset: 0x000c,
        value: 0x0e00_0000,
    }, //
    RegVal {
        offset: 0x0010,
        value: 0x0800_0000,
    }, //
];

struct ValKey16 {
    key: u16,
    val: u16,
}

type OdtOhm = [ValKey16; 23];

// drivers/ram/rockchip/sdram_rv1126.c  d3_phy_drv_2_ohm
const DRAM_T3_ODT_OHM: OdtOhm = [
    ValKey16 {
        val: 0x01F4,
        key: 0x0001,
    },
    ValKey16 {
        val: 0x00FA,
        key: 0x0002,
    },
    ValKey16 {
        val: 0x00A7,
        key: 0x0003,
    },
    ValKey16 {
        val: 0x007D,
        key: 0x0004,
    },
    ValKey16 {
        val: 0x0064,
        key: 0x0005,
    },
    ValKey16 {
        val: 0x0053,
        key: 0x0006,
    },
    ValKey16 {
        val: 0x0047,
        key: 0x0007,
    },
    ValKey16 {
        val: 0x003F,
        key: 0x0008,
    },
    ValKey16 {
        val: 0x0038,
        key: 0x0009,
    },
    ValKey16 {
        val: 0x0032,
        key: 0x000A,
    },
    ValKey16 {
        val: 0x002D,
        key: 0x000B,
    },
    ValKey16 {
        val: 0x0029,
        key: 0x000C,
    },
    ValKey16 {
        val: 0x0026,
        key: 0x000D,
    },
    ValKey16 {
        val: 0x0024,
        key: 0x000E,
    },
    // THIS: no longer < phy_clk_drv_ohm  (0x21)
    ValKey16 {
        val: 0x0021,
        key: 0x000F,
    },
    ValKey16 {
        val: 0x001F,
        key: 0x0018,
    },
    ValKey16 {
        val: 0x001D,
        key: 0x0019,
    },
    ValKey16 {
        val: 0x001C,
        key: 0x001A,
    },
    ValKey16 {
        val: 0x001A,
        key: 0x001B,
    },
    ValKey16 {
        val: 0x0019,
        key: 0x001C,
    },
    ValKey16 {
        val: 0x0018,
        key: 0x001D,
    },
    ValKey16 {
        val: 0x0017,
        key: 0x001E,
    },
    ValKey16 {
        val: 0x0016,
        key: 0x001F,
    },
];

// lp4_phy_odt_2_ohm
const DRAM_T7_ODT_OHM: OdtOhm = [
    ValKey16 {
        val: 0x1F4,
        key: 0x1,
    },
    ValKey16 {
        val: 0xFA,
        key: 0x2,
    },
    ValKey16 {
        val: 0xA7,
        key: 0x3,
    },
    ValKey16 {
        val: 0x7D,
        key: 0x4,
    },
    ValKey16 {
        val: 0x64,
        key: 0x5,
    },
    ValKey16 {
        val: 0x53,
        key: 0x6,
    },
    ValKey16 {
        val: 0x47,
        key: 0x7,
    },
    ValKey16 {
        val: 0x3F,
        key: 0x8,
    },
    ValKey16 {
        val: 0x38,
        key: 0x9,
    },
    ValKey16 {
        val: 0x32,
        key: 0xA,
    },
    ValKey16 {
        val: 0x2D,
        key: 0xB,
    },
    ValKey16 {
        val: 0x29,
        key: 0xC,
    },
    // THIS
    ValKey16 {
        val: 0x26, // >= phy_clk_drv_ohm (0x26)
        key: 0xD,
    },
    ValKey16 {
        val: 0x24,
        key: 0xE,
    },
    ValKey16 {
        val: 0x21,
        key: 0xF,
    },
    ValKey16 {
        val: 0x1F,
        key: 0x18,
    },
    // THIS
    ValKey16 {
        val: 0x1D, // x
        key: 0x19,
    },
    ValKey16 {
        val: 0x1C,
        key: 0x1A,
    },
    ValKey16 {
        val: 0x1A,
        key: 0x1B,
    },
    ValKey16 {
        val: 0x19,
        key: 0x1C,
    },
    ValKey16 {
        val: 0x18,
        key: 0x1D,
    },
    ValKey16 {
        val: 0x17,
        key: 0x1E,
    },
    ValKey16 {
        val: 0x16,
        key: 0x1F,
    },
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

// U-Boot defines two structs for this; for LPDDR4, there are more properties
// see arch/arm/include/asm/arch-rockchip/sdram_common.h
//    structs  ddr2_3_4_lp2_3_info + lp4_info
// data really comes from a global include file
// drivers/ram/rockchip/sdram-rv1126-loader_params.inc
// included with later type casts
// drivers/ram/rockchip/sdram_rv1126.c  u32 common_info[]
struct DdrCfg {
    ddr_freq_f0_f1: u32,
    ddr_freq_f2_f3: u32,
    ddr_freq_f4_f5: u32,
    odt_on_drv: u32,
    odt_off_drv: u32,
    odt_pu_cal_info: u32,
    odt_enable_freq: u32,
    odt_on_slew_rate: u32,
    odt_off_slew_rate: u32,
    // the below are LPDDR4 only
    ca_odt_enable_freq: u32,
    cs_drv_ca_odt_info: u32,
    odt_on_vref: u32,
    odt_off_vref: u32,
}

// NOTE: ODT is enabled if odt_enable_freq < dram_freq
const CFG_0X11: DdrCfg = DdrCfg {
    ddr_freq_f0_f1: 0x0014_4210,
    ddr_freq_f2_f3: 0x0021_0210,
    ddr_freq_f4_f5: 0x0000_0000,
    odt_on_drv: 0x2221_2121,
    odt_off_drv: 0x2221_2121,
    odt_pu_cal_info: 0x000C_A778,
    odt_enable_freq: 0x0014_D14D, // 0x14d < 0x144 ? NO
    odt_on_slew_rate: 0x0000_030F,
    odt_off_slew_rate: 0x0000_030F,
    ca_odt_enable_freq: 0,
    cs_drv_ca_odt_info: 0,
    odt_on_vref: 0,
    odt_off_vref: 0,
};

const CFG_0X20: DdrCfg = DdrCfg {
    ddr_freq_f0_f1: 0x0014_4210,
    ddr_freq_f2_f3: 0x0021_0210,
    ddr_freq_f4_f5: 0x0000_0000,
    odt_on_drv: 0x2225_2525,
    odt_off_drv: 0x2225_2525,
    odt_pu_cal_info: 0x000C_8B78,
    odt_enable_freq: 0x0027_1271, // 0x271 < 0x144 ? NO
    odt_on_slew_rate: 0x0001_010E,
    odt_off_slew_rate: 0x0001_010E,
    ca_odt_enable_freq: 0,
    cs_drv_ca_odt_info: 0,
    odt_on_vref: 0,
    odt_off_vref: 0,
};

const CFG_0X29: DdrCfg = DdrCfg {
    ddr_freq_f0_f1: 0x0014_4210,
    ddr_freq_f2_f3: 0x0021_0210,
    ddr_freq_f4_f5: 0x0000_0000,
    odt_on_drv: 0x2227_2525,
    odt_off_drv: 0x2227_2525,
    odt_pu_cal_info: 0x000C_9478,
    odt_enable_freq: 0x0014_D14D,
    odt_on_slew_rate: 0x000F_010F,
    odt_off_slew_rate: 0x000F_010F,
    ca_odt_enable_freq: 0,
    cs_drv_ca_odt_info: 0,
    odt_on_vref: 0,
    odt_off_vref: 0,
};

const CFG_0X41: DdrCfg = DdrCfg {
    ddr_freq_f0_f1: 0x0014_4210,
    ddr_freq_f2_f3: 0x0021_0210,
    ddr_freq_f4_f5: 0x0000_0000,
    odt_on_drv: 0x2824_241D,
    odt_off_drv: 0x2824_241D,
    odt_pu_cal_info: 0x01E0_3C50,
    odt_enable_freq: 0x000C_8320,
    odt_on_slew_rate: 0x0000_0000,
    odt_off_slew_rate: 0x0000_0000,
    ca_odt_enable_freq: 0,
    cs_drv_ca_odt_info: 0,
    odt_on_vref: 0,
    odt_off_vref: 0,
};

// NOTE: we use this
const CFG_LPDDR4: DdrCfg = DdrCfg {
    ddr_freq_f0_f1: 0x0014_4210,
    ddr_freq_f2_f3: 0x0021_0210,
    ddr_freq_f4_f5: 0x0000_0000,
    odt_on_drv: 0x2826_261e,
    odt_off_drv: 0x2826_261e,
    odt_pu_cal_info: 0x0de0_3c50,
    odt_enable_freq: 0x0014_d320,
    odt_on_slew_rate: 0x000f_0f00,
    odt_off_slew_rate: 0x000f_0f00,
    ca_odt_enable_freq: 0x0000_0320,
    cs_drv_ca_odt_info: 0x0003_0000,
    odt_on_vref: 0x17c4_b0a6,
    odt_off_vref: 0x1a46_91a4,
};

// FIXME: Recheck, some configs may be wrong here.
// U-Boot  drivers/ram/rockchip/sdram_rv1126.c  get_ddr_drv_odt_info
fn get_ddr_drv_odt_info(dram_type: u32) -> DdrCfg {
    match dram_type {
        // 0 => CFG_0X14, // does not really exist, vendor code is buggy (?)
        3 => CFG_0X11,
        6 => CFG_0X29, // RECHECK
        7 => CFG_LPDDR4,
        8 => CFG_0X41, // RECHECK
        // FIXME: This should happen much earlier. No need to carry it around.
        // It depends on parameters currently evaluated at runtime; we can just
        // do this at build time.
        _ => panic!("DRAM type {dram_type} not supported!"),
    }
}

// TODO: enum for dram_type
// drivers/ram/rockchip/sdram_rv1126.c  set_ds_odt
// set drive strength for on-die termination
fn set_ds_odt(dram_freq: u32, dram_type: u32, dst_fsp: u32) {
    println!("set_ds_odt START");
    let cfg = get_ddr_drv_odt_info(dram_type);

    // NOTE: The struct is very compact, but that is unnecessary.
    // TODO: These values could be split up in the struct already.

    let dram_odt_en_freq = cfg.odt_on_slew_rate & 0xfff;
    let dram_odt_en = dram_freq > dram_odt_en_freq;
    let (slew_rate, drive_strength, pulldown_en, dq_odt_ohm) = if dram_odt_en {
        let slew_rate = cfg.odt_on_slew_rate;
        let drive_strength = cfg.odt_on_drv;
        // PHY_LP4_DRV_PULLDOWN_EN_ODTON
        let pulldown_en = (cfg.odt_pu_cal_info >> 28) & 1;
        let dq_odt_ohm = cfg.odt_pu_cal_info & 0xff;
        (slew_rate, drive_strength, pulldown_en, dq_odt_ohm)
    } else {
        // NOTE: We are here.
        let slew_rate = cfg.odt_off_slew_rate;
        let drive_strength = cfg.odt_off_drv;
        // PHY_LP4_DRV_PULLDOWN_EN_ODTOFF
        let pulldown_en = (cfg.odt_pu_cal_info >> 29) & 1;
        let dq_odt_ohm = 0;
        (slew_rate, drive_strength, pulldown_en, dq_odt_ohm)
    };

    // reference values used in search
    let phy_clk_drv_ohm = (drive_strength >> 16) as u8; // 0x26
    let phy_ca_drv_ohm = (drive_strength >> 8) as u8; // 0x26
    let phy_dq_drv_ohm = drive_strength as u8; // 0x1e

    let phy_odt_en_freq = (cfg.odt_on_slew_rate >> 12) & 0xfff;
    let phy_odt_en = dram_freq > phy_odt_en_freq;
    let (dram_dq_drv_ohm, phy_odt_ohm, phy_odt_pullup_enable, phy_odt_pulldown_enable, drv_pu_cal) =
        if phy_odt_en {
            let dram_dq_drv_ohm = cfg.odt_on_drv >> 24;
            let phy_odt_ohm = (cfg.odt_pu_cal_info >> 8) & 0x3ff;
            let phy_odt_pullup_enable = (cfg.odt_pu_cal_info >> 18) & 1;
            let phy_odt_pulldown_enable = (cfg.odt_pu_cal_info >> 19) & 1;
            let drv_pu_cal = (cfg.odt_pu_cal_info >> 26) & 1;
            (
                dram_dq_drv_ohm,
                phy_odt_ohm,
                phy_odt_pullup_enable,
                phy_odt_pulldown_enable,
                drv_pu_cal,
            )
        } else {
            let dram_dq_drv_ohm = cfg.odt_off_drv >> 24;
            // LP4_DRV_PU_CAL_ODTOFF
            let drv_pu_cal = (cfg.odt_pu_cal_info >> 27) & 1;
            let phy_odt_ohm = 0;
            let phy_odt_pullup_enable = 0;
            let phy_odt_pulldown_enable = 0;
            (
                dram_dq_drv_ohm,
                phy_odt_ohm,
                phy_odt_pullup_enable,
                phy_odt_pulldown_enable,
                drv_pu_cal,
            )
        };

    // 5 iterations
    if dram_type < 9 {
        for o in (0x0300..0x0a80).step_by(0x180) {
            let r = DDR_PHY_BASE + o + 8;
            let v = read32(r);
            write32(r, v & 0xffff_fdff);
        }
    }

    // conditions omitted
    let m0 = 0;
    let m1 = 0;

    let ca_odt_en_freq = cfg.ca_odt_enable_freq & 0xfff;
    let ca_odt_en = dram_freq > ca_odt_en_freq;
    let ca_odt_ohm = if dram_type == 7 && ca_odt_en {
        (cfg.odt_pu_cal_info >> 18) & 0xff
    } else {
        0
    };

    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v | (1 << 1));
    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v & !(1 << 3));
    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v | (1 << 3));
    let v = read32(DDR_PHY_008C);
    write32(DDR_PHY_008C, v & !(1 << 1));

    let odt_ohm = match dram_type {
        3 => DRAM_T3_ODT_OHM,
        7 => DRAM_T7_ODT_OHM,
        8 => todo!(), // DRAM_T8_ODT_OHM,
        _ => todo!(), // DRAM_TX_ODT_OHM,
    };

    // TODO: Find key in odt_ohm via each target_val; precalc..?
    let target_val3 = 0;

    let phy_clk_drv = &odt_ohm[12 + 2];
    let phy_ca_drv = &odt_ohm[12 + 2];
    let phy_dq_drv = &odt_ohm[16 + 2];
    let val3 = 0;

    let vref = if dram_type < 9 {
        let v = if target_val3 == 0 {
            cfg.odt_off_vref
        } else {
            cfg.odt_on_vref
        };
        ((v & 0x3ff) << 9) / 1000
    } else {
        todo!("vref for other DRAM types");
    };

    let xx3 = 0x100;

    let vxm = ((phy_clk_drv.key as u32) << 8) | (phy_clk_drv.key as u32);

    let v = (((phy_ca_drv.key as u32) << 24) | ((phy_ca_drv.key as u32) << 16)) | vxm;
    write32(DDR_PHY_00F4, v);

    let v = read32(DDR_PHY_00F8) & 0xffff_e0e0 | vxm;
    write32(DDR_PHY_00F8, v);

    let v = read32(DDR_PHY_00F0) & 0xffff_e0e0;
    let v = v | (slew_rate & 0xff00) | ((slew_rate >> 16) & 0xff);
    write32(DDR_PHY_00F0, v);

    let mx0 = if m0 == 0 { 0 } else { val3 };
    let mx1 = if m1 == 0 { 0 } else { val3 };

    // NOTE: XOR here flips the bit
    let pulldown = (pulldown_en << 7) ^ (1 << 7);
    // TODO: tweak this
    // 5 iterations
    for o in (0x0300..0x0a80).step_by(0x180) {
        let r = DDR_PHY_BASE + o + 4;
        let v = ((phy_dq_drv.key as u32) << 24)
            | ((phy_dq_drv.key as u32) << 16)
            | ((mx0 as u32) << 8)
            | mx1 as u32;
        write32(r, v);
        let r = DDR_PHY_BASE + o;
        let v = read32(r) & 0x007f_e07f;
        let v = v | ((slew_rate & 0xff) << 8) | (vref << 23) | pulldown;
        write32(r, v as u32);
    }

    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v | 0x80);
    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v & !0x80);

    let v = read32(DDR_PHY_00F8);
    write32(DDR_PHY_00F8, v & 0xfe00_ffff | (xx3 << 16));

    let v1 = if cfg.ca_odt_enable_freq & 0xfff < dram_freq {
        cfg.odt_on_vref
    } else {
        cfg.odt_off_drv
    };
    let v1 = (v1 >> 20) & 0x3ff;

    // odt_enable_freq & 0xfff = 0x14d
    let v2 = if cfg.odt_enable_freq & 0xfff < dram_freq {
        cfg.odt_on_vref
    } else {
        cfg.odt_off_drv
    };
    let v2 = (v2 >> 10) & 0x3ff;

    let (v1, v2) = if dram_type == 7 {
        let v1 = match v1 {
            ..100 => 0,
            ..301 => (v1 - 100) / 4,
            ..421 => ((v1 - 220) / 4) | 0x40,
            _ => 114,
        };

        let v2 = match v2 {
            ..100 => 0,
            ..301 => (v2 - 100) / 4,
            ..421 => ((v2 - 220) / 4) | 0x40,
            _ => 114,
        };

        (v1, v2)
    } else {
        let v1 = v1 * 11 / 6;

        let v1 = match v1 {
            ..150 => 0,
            ..450 => (v1 - 150) / 6,
            ..630 => ((v1 - 329) / 6) | 0x40, // we should be here
            _ => 114,
        };

        let v2 = match v2 {
            ..150 => 0, // we should be here
            ..450 => (v2 - 150) / 6,
            ..630 => ((v2 - 329) / 6) | 0x40,
            _ => 114,
        };
        (v1, v2)
    };

    write32(UPCTL2_SW_CTRL, 0);

    let fsp_offset = get_fsp_offset(dst_fsp);
    let init3 = UPCTL2_INIT3 + fsp_offset;
    let init4 = UPCTL2_INIT4 + fsp_offset;
    let init6 = UPCTL2_INIT6 + fsp_offset;
    let init7 = UPCTL2_INIT7 + fsp_offset;

    let v = read32(init4);
    write32(init4, (v & 0xffff_0000) | v1);

    let v = read32(init6);
    write32(init6, (v & 0xffff_0000) | v2);

    let mr1_mr3 = match dram_type {
        0 | 3 => {
            let v = read32(init3);
            let v = v & 0xfd99;

            let nv = if dram_dq_drv_ohm == 0x22 {
                v | (1 << 1)
            } else {
                v
            };
            // TODO: if !smth ...
            0
        }
        6 => {
            // TODO
            0
        }
        // LPDDR4(X)
        _ => {
            // NOTE: This is not in upstream U-Boot as of 4d3b5c679bc9.
            if dq_odt_ohm != 0 {
                // TODO: skipped as we do not run into this for now
            }

            // NOTE: U-Boot reads from init4 this earlier.
            let mr1_mr3_pre = (read32(init4) >> 16) & 0xffff_ffc6 | drv_pu_cal;
            let drv_odt = odt_calc(dram_dq_drv_ohm);
            let drv_odt = if drv_odt == 0 { 8 } else { drv_odt };
            let mr1_mr3 = mr1_mr3_pre | (drv_odt << 3);

            const MR_SHIFT: usize = 16;
            const MR_MASK: u32 = 0x0000_ffff;

            /* MR11 for lp4 ca odt, dq odt set */
            let dq_odt = odt_calc(dq_odt_ohm);
            let ca_odt = odt_calc(ca_odt_ohm);

            let v = read32(init6);
            let v1 = (v >> MR_SHIFT) & !0b0111_0111;
            let mr11 = v1 | (ca_odt << 4) | dq_odt;

            upctl2_sw_set_req();
            write32(init6, v & MR_MASK | (mr11 << MR_SHIFT));
            upctl2_sw_set_ack();

            /* MR22 for soc odt/odt-ck/odt-cs/odt-ca */
            let phy_odt = odt_calc(phy_odt_ohm);
            // NOTE: looks like those are retained or explicitly set here.
            let ck_cs_ca = (cfg.cs_drv_ca_odt_info >> 16) & 0b111;

            let v = read32(init7);
            let v1 = (v >> MR_SHIFT) & !0b111;
            let mr14 = v1 | (ck_cs_ca << 3) | phy_odt;

            upctl2_sw_set_req();
            write32(init7, v & MR_MASK | (mr14 << MR_SHIFT));
            upctl2_sw_set_ack();

            mr1_mr3
        }
    };

    upctl2_sw_set_req();
    if dram_type != 0 && dram_type != 3 {
        let m = 0xffff_0000;
        let v = read32(init4);
        write32(init4, (v & m) | mr1_mr3);
    } else {
        // TODO
    }
    upctl2_sw_set_ack();
    println!("set_ds_odt DONE");
}

// NOTE: There are multiple blocks with INIT registers.
// The offset depends on dst_fsp; here just f.
// TODO: what is fsp short for?
fn get_fsp_offset(f: u32) -> usize {
    (if f == 0 { 0 } else { (f + 1) * 0x1000 }) as usize
}

fn odt_calc(odt_ohm: u32) -> u32 {
    match odt_ohm {
        0 => 0,
        ..40 => 6,
        ..48 => 5,
        ..60 => 4,
        ..80 => 3,
        ..120 => 2,
        _ => 1,
    }
}

// U-Boot  drivers/ram/rockchip/sdram_rv1126.c  sw_set_ack
fn upctl2_sw_set_ack() {
    write32(UPCTL2_SW_CTRL, 1);
    while read32(UPCTL2_SW_STAT) & 1 == 0 {}
}

// clear sw_done=0; U-Boot: sw_set_req
fn upctl2_sw_set_req() {
    write32(UPCTL2_SW_CTRL, 0);
}

fn get_funny_bits() -> (u32, u32) {
    let v = read32(DDR_GRF_CTRL3);
    // Extract bits 8..15. The & 0xff is technically not necessary since we
    // do another extraction hereafter, taking a pair of bits at i * 2.
    let dq_map = (v >> 8) & 0xff;
    println!("DDR_GRF_000C bits 15..8: {dq_map:08b} (full reg val: {v:08x})");

    let mut v0 = 0;
    let mut v1 = 0;
    for i in 0..4 {
        // check on bits 8..9, 10..11, 12..13, 14..15 in respective round
        match (dq_map >> (i * 2)) & 0x3 {
            0 => {
                v0 = i;
            }
            1 => {
                v1 = i;
            }
            _ => {}
        }

        println!("round {i}: {v0},{v1}");
    }
    (v0, v1)
}

// NOTE: this looks similar to PHY cfg functions for other PHYs
// U-Boot  drivers/ram/rockchip/sdram_rv1126.c  phy_cfg
fn phy_cfg(cfg: &PhyCfg, dram_freq: u32, rank: u32, chan_bus_width: u32, post_init: bool) {
    phy_pll_set(dram_freq * MEGA, 0);

    for p in cfg.iter() {
        let r = DDR_PHY_BASE + p.offset as usize;
        let v = if p.offset <= 16 {
            (read32(r) & 0xc0ff_ffff) | p.value
        } else {
            p.value
        };
        write32(r, v)
    }

    // Extracted here to keep the flow simpler
    let (v0, v1) = get_funny_bits();

    // TODO: Is this really PHY DQ width mask?
    const PHY_DQ_WIDTH_MASK: u32 = 0xffff_e0ff;
    let v = read32(DDR_PHY_0000) & PHY_DQ_WIDTH_MASK;

    let vx = match chan_bus_width {
        1 => v | ((1 << v0) | (1 << v1)) << 8,
        2 => v | 0x0f00,
        _ => v | 0x100 << v0,
    };
    let mut vxo = if post_init { vx } else { vx | 0x1000 };

    if rank == 4 {
        vxo |= 0x0010_0000;
        let v = read32(DDR_PHY_0038);
        write32(DDR_PHY_0038, v | 1 << 1);
    }

    write32(DDR_PHY_BASE, vxo);
}

// This appears to be some kind of measurement, yielding different values for
// different runs.
fn phy_measure_xx(dram_freq: u32) -> u32 {
    let vx = if dram_freq == 0 {
        0
    } else {
        // dram_freq = 0x144 -> 10000 / 1543 + 1 = 6 + 1 = 7
        10_000 / (500_000 / dram_freq)
    } + 1;

    let v = read32(DDR_PHY_01F4);
    println!("DDR_PHY_01F4: {v:08x}");
    let v = v >> 24;

    let v3 = if v < 0x41 {
        3200 - 64 * v
    } else {
        (50 - vx) * v // 43 * v
    };

    // real values seen:
    //    27 (0x1b)
    //    92 (0x5c)
    //   214 (0xd6)
    //   215 (0xd7)
    //   216 (0xd8)
    (v3 / 100) & 0x7f
}

type AddrmapData = [u32; 9];

const ADDR_MAP_DATA_17: AddrmapData = [
    0x0000_1f1f,
    0x0009_0909,
    0x0000_0000,
    0x0000_0000,
    0x0000_1f00,
    0x0808_0808,
    0x0808_0808,
    0x0000_0f08,
    0x0000_0000,
];

fn upctl2_addrmap_prefill(d: &AddrmapData) {
    for (i, v) in d.iter().enumerate() {
        write32(UPCTL2_ADDR_MAP_BASE + i * 4, *v);
    }
}

#[repr(u32)]
enum DdrType {
    DDR4 = 0,
    DDR3 = 3,
    LPDDR2 = 5,
    LPDDR3 = 6,
    LPDDR4 = 7,
    LPDDR4X = 8,
    LPDDR5 = 9,
    DDR5 = 10,
    UNUSED = 0xFF,
}

// sdram_init_ / sdram_init_detect ?
fn sdram_init(post_init: bool) {
    // TODO: These values come from structs at the offsets encoded in the
    // variable names. Should we make those structs or simple parameters?
    // U-Boot arch/arm/include/asm/arch-rockchip/sdram_common.h sdram_cap_info
    //        arch/arm/include/asm/arch-rockchip/sdram_rv1126.h
    let rank = 1;
    let column = 11;
    let bank_num = 3; // power of 2, i.e., 2^3=8
    let chan_bus_width = 1; // 1 means 16bit
    let die_bus_width = 1; // 0 means 8bit
    let row_3_4 = 0; // 0 means normal die, power of 2
    let cs0_row = 17;
    let cs1_row = 17;

    let dram_freq = 0x144;
    let dram_type = DdrType::LPDDR4 as u32;

    println!("sdram_init");
    write32(DDR_GRF_CTRL0, 0x20000);

    clk_set_dpll((dram_freq * MEGA) / 2);

    // maybe reset
    write32(SYS_SGRF_0014, 0x0b00_0b00);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0002);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0180);
    udelay(10);
    write32(SYS_SGRF_0014, 0x0b00_0b00);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0002);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0100);

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

    // extracted
    // TODO: other rounds may have different params / sizes thereof
    phy_cfg(&PHY_CFG3, dram_freq, rank, chan_bus_width, post_init);

    // TODO: tweak this
    // Each loop has 5 iterations
    match dram_type {
        0 | 3 | 6 => {
            for o in (0x0300..0x0a80).step_by(0x180) {
                let r = DDR_PHY_BASE + o + 8;
                let v = read32(r);
                let nv = v & 0xffff_fdff;
                println!("  {r:08x}: {v:08x} -> {nv:08x}");
                write32(r, nv);
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

    let ctl_cfg_first_val = UPCTL2_CFG3[0].value;
    if ctl_cfg_first_val & (1 << 10) != 0 {
        let v = read32(DDR_PHY_00C0);
        write32(DDR_PHY_00C0, v | 0x0006_0000);
    }

    let v = read32(DDR_PHY_00AC);
    write32(DDR_PHY_00AC, v | 0x10);

    let v = read32(DDR_PHY_0044);
    write32(DDR_PHY_0044, v & 0x3fff_ffff);

    // reset?
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0000);
    write32(SYS_SGRF_0014, 0x0b00_0300);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0000);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0000);

    // U-Boot sdram_init_

    // NOTE: params list needs to be a param here as well
    // these stem from the global config:
    const SELF_REFRESH_IDLE: u32 = 0x005d;
    const PD_IDLE: u32 = 0x000d;
    upctl2_config(&UPCTL2_CFG3, SELF_REFRESH_IDLE, PD_IDLE);

    let v = read32(UPCTL2_PCFGR_N);
    write32(UPCTL2_PCFGR_N, v | (1 << 16));

    // drivers/ram/rockchip/sdram_rv1126.c
    // set frequency_mode
    let v = read32(UPCTL2_MSTR);
    write32(UPCTL2_MSTR, v | (1 << 29));
    // set target_frequency to Frequency 0
    let v = read32(UPCTL2_MSTR2);
    write32(UPCTL2_MSTR2, v & !(0b11));

    set_ds_odt(dram_freq, dram_type, 0);

    // similar to arch/arm/mach-rockchip/rk3036/sdram_rk3036.c  sdram_all_config
    // 0xd (13)
    let bw_plus_col = chan_bus_width + column;

    // 0 | (3 << 5) | 3 = 0b0110_0011 = 0x63
    // TODO: is this osreg?
    let vt = ((rank - 1) << 8) | ((cs0_row - 13) << 5) | (bw_plus_col - 10);
    println!("vt {vt:08x}");

    // bank_num is 3 -> 0x6b
    let vxx = if bank_num == 3 { vt | 8 } else { vt };

    // see calculate_ddrconfig

    // This searches for an index to address map data.
    // We currently hardcode that list.
    // TODO: actually use this resulting value.
    // Possible resulting values: 0..=8, 14, 17
    let idx = find_addrmap_index(vxx).unwrap_or_else(|| {
        if bank_num == 3 && bw_plus_col == 10 {
            14
        } else if rank != 1 || bank_num != 3 || cs0_row > 17 || bw_plus_col != 13 {
            // NOTE: This should never happen.
            // Do the check at build time, and do it earlier instead of here.
            panic!("DDR config error")
        } else {
            // We should be here.
            17
        }
    });
    // NOTE: unused; original code mutates config field12 (offset 0x30).
    let s_0030 = idx;
    println!("DDR addrmap data index: {idx}");

    // essentially memcpy
    // NOTE: data really depends on previously found index.
    upctl2_addrmap_prefill(&ADDR_MAP_DATA_17);

    // TODO: loop logic
    // This is only one iteration, can be more depending on cs0_row.
    let o = 0x0218 + 4;
    let r = UPCTL2_BASE + o;
    let v = read32(r);
    // put 0xf at respectively byte position 0, 1, 2 or 3
    let byte_pos = (0x11 & 3) << 3; // 1 << 3 = 8
    write32(r, v | (0xf << byte_pos));

    // TODO: some code skipped here that is for non-LPDDR4

    if rank == 1 {
        let r = UPCTL2_ADDR_MAP_BASE;
        let v = read32(r);
        write32(r, v | 0x1f);
    }

    // also in U-Boot drivers/ram/rockchip/sdram_rv1126.c sdram_init_
    let r = UPCTL2_DFI_MISC;
    let v = read32(r);
    write32(r, v | (1 << 5) | (1 << 4));

    // reset ?
    write32(SYS_SGRF_0014, 0x0b00_0000);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0000);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0000);

    // bits 0..2: OPERATING_MODE
    // - 0 = init
    // - 1 = normal
    // - 2 = PD (?)
    // - 3 = self-refresh
    // bits 4..5: SELFREF_TYPE
    // - 2 = "not auto"
    while read32(UPCTL2_STAT) & 0b111 == 0 {}

    let vf = phy_measure_xx(dram_freq);
    println!("vf: {vf} (0x{vf:02x})");
    // This is used to adjust registers in multiple blocks.
    let a = (vf << 24) | (vf << 8);

    const BLOCK_SIZE: usize = 0x180;
    const BLOCK_COUNT: usize = 5;
    let mask = !((0x7f << 24) | (0x7f << 8));
    // NOTE: rank apparently could be hardcoded at build time.
    for i in 0..rank {
        let o = match i {
            0 => 0x33c,
            1 => 0x35c,
            2 => 0x418,
            _ => 0x438,
        };
        // NOTE: inclusive
        for block in 0..BLOCK_COUNT {
            let reg = DDR_PHY_BASE + block * BLOCK_SIZE + o;
            let val = read32(reg);
            let v = val & mask | a;
            if DEBUG {
                println!("  {reg:08x}: {val:08x} -> {v:08x}");
            }
            write32(reg, v);
        }
    }

    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v | (1 << 2));
    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v & !(1 << 2));

    let mr12 = upctl2_read_mr(1, 12, 7);
    let mr14 = upctl2_read_mr(1, 14, 7);

    if DEBUG {
        println!("LPDDR4 mode registers");
        println!("  MR12: {mr12:02x}");
        println!("  MR14: {mr14:02x}");
        assert_eq!(mr12, 0x4d);
        assert_eq!(mr14, 0x4d);
    }

    let init6 = read32(UPCTL2_INIT6);
    let init7 = read32(UPCTL2_INIT7);
    if DEBUG {
        println!("INIT6: {init6:08x}");
        println!("INIT7: {init7:08x}");
    }

    upctl2_write_mr(15, 11, (init6 >> 16) as u16, 7);
    upctl2_write_mr(15, 12, init6 as u16, 7);
    upctl2_write_mr(15, 22, (init7 >> 16) as u16, 7);

    if DEBUG {
        let mr11 = upctl2_read_mr(1, 11, 7);
        let mr12 = upctl2_read_mr(1, 12, 7);
        let mr22 = upctl2_read_mr(1, 22, 7);
        println!("  MR11: {mr11:02x}");
        println!("  MR12: {mr12:02x}");
        println!("  MR22: {mr22:02x}");
    }

    while read32(UPCTL2_DBG_STAT) & (1 << 4) != 0 {}
    write32(UPCTL2_DBG_CMD, 0x0000_0010);
    while read32(UPCTL2_DBG_STAT) & (1 << 4) != 0 {}

    train(
        0, // cs
        dram_type,
        FlagSet::<TrainingFlag>::from(TrainingFlag::ReadGate),
    );

    todo!("draw the rest of the owl 🦉🖌️");

    println!("sdram_init done");
}

// TODO: Is this correct?
// NOTE: We start at +4K to avoid accessing address 0, on which Rust errors.
const RAM_BASE: usize = 0x1000;
fn dram_test() {
    let pattern = 0xffaa_5500;
    for o in (0..64).step_by(4) {
        write32(RAM_BASE + o, pattern);
        let p = read32(RAM_BASE + o);
        println!("{p:08x}");
    }
    for o in (0..64).step_by(4) {
        let p = read32(RAM_BASE + o);
        println!("{p:08x}");
    }
}

use flagset::{flags, FlagSet, Flags};

// U-Boot: data_training checks the training flag,
// then calls specific functions for each training
//
//  0xff: full training (all but CA training)
//  bit 0: CA training
//  bit 1: read gate training
//  bit 2: write leveling (data_training_wl)
//  bit 3: write training
//  bit 4: read training
fn train(cs: u32, dram_type: u32, training_flags: FlagSet<TrainingFlag>) {
    if training_flags.contains(TrainingFlag::WriteLeveling) {
        train_write_leveling(cs, dram_type);
    }
    if training_flags.contains(TrainingFlag::ReadGate) {
        train_read_gate(cs, dram_type);
    }
    if training_flags.contains(TrainingFlag::Read) {
        todo!("train_read")
    }
    if training_flags.contains(TrainingFlag::Write) {
        todo!("train_write")
    }
}

flags! {
    pub enum TrainingFlag: u8 {
        Ca = 1 << 0,
        ReadGate = 1 << 1,
        WriteLeveling = 1 << 2,
        Write = 1 << 3,
        Read = 1 << 4,
        All = 0b11110,
    }
}

fn train_write_leveling(cs: u32, dram_type: u32) {
    let is_auto_zq_enabled = upctl2_disable_zq_cs();
    write32(CRU_NS_VPLL_CFG0, is_auto_zq_enabled as u32);

    let v = read32(UPCTL2_MSTR2);
    write32(UPCTL2_MSTR2, v & !(1 << 1));

    let cur_fsp = read32(UPCTL2_MSTR2) & 0b11;
    let o = get_fsp_offset(cur_fsp);
    let r = UPCTL2_INIT3 + o;
    let xx = read32(r);

    let vx = if dram_type != 0 && dram_type != 3 {
        xx & 0xff
    } else {
        xx & 0x3fff | 0x4000
    };
    //
}

const RANK4_ENABLED: u32 = 1 << 20;

fn train_read_gate(cs: u32, dram_type: u32) {
    let phy0300 = read32(DDR_PHY_0300);

    let v = read32(DDR_PHY_0000);
    let rank4_was_disabled = v & RANK4_ENABLED == 0;

    // For CS > 1, ensure that rank 4 is enabled
    if rank4_was_disabled && cs > 1 {
        write32(DDR_PHY_0000, v | RANK4_ENABLED);
    }
    let phy0000 = read32(DDR_PHY_0000);

    // prepare
    let nv = phy0300 & 0xffffff9f | 0x40;
    for r in (DDR_PHY_0300..DDR_PHY_0300 + 0x0a80).step_by(0x180) {
        write32(r, nv);
    }

    let is_auto_zq_enabled = upctl2_disable_zq_cs();

    if dram_type == 0 {
        // TODO
    }

    let v = read32(DDR_PHY_0004);
    // bit 2..5: cal_cs_sel
    // Position of 0 determines rank: 0b1110 means rank 0, 0b0111 means rank 3.
    // 0b0000 means RX-DQS calibration result auto-switches as per DFI command
    // after RX-DQS training.
    write32(
        DDR_PHY_0004,
        v & 0xffff_ffc3 | ((!(1 << (cs & 0x1f)) & 0b1111) << 2),
    );
    // Start RX-DQS calibration.
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v | 1);

    let cal_res = check_calibration();

    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & !1);
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & 0xffff_ffc3);

    upctl2_restore_zq_cs(is_auto_zq_enabled);
    upctl2_dbg_rank01_refresh(8);

    let channel_en = (phy0000 >> 8) & 0b11111;
    if DEBUG {
        println!("Channel A DQ 0..7  enabled: {}", channel_en & (1 << 0) != 0);
        println!("Channel A DQ 8..15 enabled: {}", channel_en & (1 << 1) != 0);
        println!("Channel B DQ 0..7  enabled: {}", channel_en & (1 << 2) != 0);
        println!("Channel B DQ 8..15 enabled: {}", channel_en & (1 << 3) != 0);
        println!("Channel C DQ 0..7  enabled: {}", channel_en & (1 << 4) != 0);
    }

    if channel_en != cal_res {
        panic!("channel_en does not match calibration result: {cal_res}");
    }

    // restore
    for r in (DDR_PHY_0300..DDR_PHY_0300 + 0x0a80).step_by(0x180) {
        write32(r, phy0300);
    }

    if rank4_was_disabled {
        let v = read32(DDR_PHY_0000);
        write32(DDR_PHY_0000, v & !RANK4_ENABLED);
    }

    let v = read32(DDR_PHY_0448);
    let xx = v >> ((cs & 1) * 16);
    if xx & 0x7ff != 0 {
        println!("hmm {v:08x} {cs} {xx:03x}");
    }
}

fn check_calibration() -> u32 {
    let t0 = crate::arm::get_time();
    for _ in 0..50 {
        let v = read32(DDR_PHY_020C);
        if v & (1 << 5) != 0 {
            panic!("RX-DQS calibration error");
        }
        if v & (1 << 6) != 0 {
            let t1 = crate::arm::get_time();
            println!("RX-DQS calibration done in {}us", t1 - t0);
            // each of the lowest bits means calibration done for byte 0..4
            return v & 0b11111;
        }
        udelay(1);
    }
    0
}

// counterparts to UPCTL2_DBG_CMD
const DBG_RANK0_BUSY: u32 = 1 << 0;
const DBG_RANK1_BUSY: u32 = 1 << 1;
const DBG_STAT_BUSY: u32 = DBG_RANK0_BUSY | DBG_RANK1_BUSY;

fn upctl2_dbg_rank01_refresh(n: usize) {
    for _ in 0..n {
        while read32(UPCTL2_DBG_STAT) & DBG_STAT_BUSY != 0 {}
        // NOTE: 0x3 should also do; bits 2-3 are supposedly reserved
        write32(UPCTL2_DBG_CMD, 0x0000_000f);
    }
}

// U-Boot pctl_rest_zqcs_aref
fn upctl2_restore_zq_cs(auto_zq_was_disabled: bool) {
    if auto_zq_was_disabled {
        let v0 = read32(UPCTL2_ZQ_CTRL0);
        write32(UPCTL2_ZQ_CTRL0, v0 & !(1 << 31));
    }
    // enable auto refresh
    let v = read32(UPCTL2_REFRESH_CTRL4);
    write32(UPCTL2_REFRESH_CTRL4, v & !1);

    let v = read32(UPCTL2_REFRESH_CTRL4);
    write32(UPCTL2_REFRESH_CTRL4, v ^ (1 << 1));
}

// U-Boot drivers/ram/rockchip/sdram_pctl_px30.c pctl_dis_zqcs_aref
// Returns whether ZQ CS was already disabled.
fn upctl2_disable_zq_cs() -> bool {
    let v0 = read32(UPCTL2_ZQ_CTRL0);
    let auto_zq_was_disabled = v0 & (1 << 31) != 0;
    // ensure ZQ CS is disabled
    if auto_zq_was_disabled {
        write32(UPCTL2_ZQ_CTRL0, v0 | (1 << 31));
    }
    // disable auto refresh
    let v = read32(UPCTL2_REFRESH_CTRL4);
    write32(UPCTL2_REFRESH_CTRL4, v | 1);

    let v = read32(UPCTL2_REFRESH_CTRL4);
    write32(UPCTL2_REFRESH_CTRL4, v ^ (1 << 1));

    auto_zq_was_disabled
}

// U-Boot drivers/ram/rockchip/sdram_pctl_px30.c pctl_write_mr
fn upctl2_write_mr(rank: u32, mr: u32, val: u16, p4: u32) {
    println!("upctl2_write_mr rank {rank} mr {mr} val {val:02x}");

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

// U-Boot pctl_read_mr
// NOTE: MR is Mode Register.
// See also: https://www.mindshare.com/files/MindShare_DRAM_QRG_v5a.pdf
fn upctl2_read_mr(rank: u32, mr: u32, dram_type: u32) -> u8 {
    // NOTE: We might move this out, since parameters are just forwarded.
    upctl2_prep_poll_mr(rank, mr);

    let v = read32(DDR_GRF_STATUS00);
    let v = if dram_type < 9 {
        // We should get here.
        read32(DDR_GRF_STATUS01) >> 8
    } else {
        v
    };
    v as u8
}

const DEBUG: bool = true;

/*
 * drivers/ram/rockchip/sdram_pctl_px30.c pctl_read_mr()
 * cs = chip select
 * mr = mode register?
 *
 * rank = 1: cs0
 * rank = 2: cs1
 */
fn upctl2_prep_poll_mr(rank: u32, mr: u32) {
    write32(UPCTL2_MR_CTRL0, (rank << 4) | 1);
    write32(UPCTL2_MR_CTRL1, mr << 8);

    let v = read32(UPCTL2_MR_CTRL0);
    write32(UPCTL2_MR_CTRL0, v | (1 << 31));
    // wait for bit to be cleared
    while read32(UPCTL2_MR_CTRL0) & (1 << 31) != 0 {}

    while read32(UPCTL2_MR_STAT) & UPCTL2_MR_WR_BUSY != 0 {}
}

const INDEX_DATA: [u32; 9] = [
    0x00AA, 0x01A9, 0x018A, 0x016B, //
    0x014C, 0x005C, 0x0099, 0x009A, //
    0x007B,
];

// Possible results: 0..=8
fn find_addrmap_index(vxx: u32) -> Option<usize> {
    for (i, c) in INDEX_DATA.iter().enumerate() {
        let x = *c;
        // NOTE: ^ is XOR
        if (c ^ vxx) & 0x1f == 0 && // asd
            (vxx & 0xe0) <= (x & 0xe0) && // asd
            (vxx & 0x100) <= (x & 0x100)
        {
            return Some(i);
        }
    }
    None
}

fn upctl2_config(reg_vals: &[RegVal], p1: u32, p2: u32) {
    fill_regs(UPCTL2_BASE, reg_vals);

    let v = read32(UPCTL2_POWER_TIMING);
    let m = 0xff00_ffe0;
    write32(UPCTL2_POWER_TIMING, v & m | ((p1 & 0xff) << 16) | p2 & 0x1f);

    let v = read32(UPCTL2_HWLP_CTRL);
    let m = 0xf000_ffff;
    write32(UPCTL2_HWLP_CTRL, v & m | 0x50000);

    let v = read32(UPCTL2_ZQ_CTRL0);
    write32(UPCTL2_ZQ_CTRL0, v | 0x80000000);
}

fn fill_regs(base: usize, data: &[RegVal]) {
    for e in data {
        write32(base + e.offset as usize, e.value);
    }
}

// NOTE: The first value here has been changed to 0x4304_1401 ( | 0x400 ) in
// the first round of dram_init_main.
const UPCTL2_CFG0: [RegVal; 25] = [
    RegVal {
        offset: 0x0000, // MSTR
        // NOTE: value overridden in control flow in dram_init_main, condition
        // for setting or clearing bit 10 (0x400) seems to be a fixed constant.
        value: 0x4304_1001 | (1 << 10),
    },
    RegVal {
        offset: 0x0064, // refresh timing
        value: 0x0027_0039,
    },
    RegVal {
        offset: 0x00d0, // INIT0
        value: 0x0002_0051,
    },
    RegVal {
        offset: 0x00d4, // INIT1
        value: 0x0021_0000,
    },
    RegVal {
        offset: 0x00d8, // INIT2
        value: 0x0000_0100,
    },
    RegVal {
        offset: 0x00dc, // INIT3
        value: 0x0310_0000,
    },
    RegVal {
        offset: 0x00e0, // INIT4
        value: 0x0,
    },
    RegVal {
        offset: 0x00e4, // INIT5
        value: 0x90000,
    },
    RegVal {
        offset: 0x00f4, // RANK_CTRL
        value: 0xF022F,
    },
    RegVal {
        offset: 0x0100, // DRAM_TIMING0
        value: 0x0709_0b06,
    },
    RegVal {
        offset: 0x0104, // DRAM_TIMING1
        value: 0x0005_0209,
    },
    RegVal {
        offset: 0x0108, // DRAM_TIMING2
        value: 0x0303_0307,
    },
    RegVal {
        offset: 0x010c, // DRAM_TIMING3
        value: 0x0020_2006,
    },
    RegVal {
        offset: 0x0110, // DRAM_TIMING4
        value: 0x0302_0203,
    },
    RegVal {
        offset: 0x0114, // DRAM_TIMING5
        value: 0x0303_0202,
    },
    RegVal {
        offset: 0x0120, // DRAM_TIMING8
        value: 0x0000_0903,
    },
    RegVal {
        offset: 0x0180, // ZQ_CTRL0
        value: 0x0080_0020,
    },
    RegVal {
        offset: 0x0184, // ZQ_CTRL1
        value: 0x0,
    },
    RegVal {
        offset: 0x0190, // DFI_TIMING0
        value: 0x0701_0001,
    },
    RegVal {
        offset: 0x0198, // DFI_LP_CFG0
        value: 0x0a00_0101,
    },
    RegVal {
        offset: 0x01a0, // DFI_UPDATE0
        value: 0xc040_0003,
    },
    RegVal {
        offset: 0x0240, // ODT_CFG
        value: 0x0600_0600,
    },
    RegVal {
        offset: 0x0244, // ODT_MAP
        value: 0x0000_0201,
    },
    RegVal {
        offset: 0x0250, // SCHED
        value: 0x0000_1f00,
    },
    RegVal {
        offset: 0x0490, // PCTRL_N
        value: 0x1,
    },
];

// NOTE: The first value here has been changed to 0x4304_1401 ( | 0x400 ) in
// the first round of dram_init_main.
const UPCTL2_CFG3: [RegVal; 31] = [
    RegVal {
        offset: 0x0,
        value: 0x83081020,
    },
    RegVal {
        offset: 0x64,
        value: 0x13002E,
    },
    RegVal {
        offset: 0xD0,
        value: 0x2013E,
    },
    RegVal {
        offset: 0xD4,
        value: 0x210000,
    },
    RegVal {
        offset: 0xD8,
        value: 0x202,
    },
    RegVal {
        offset: 0xDC,
        value: 0x240012,
    },
    RegVal {
        offset: 0xE0,
        value: 0x310000,
    },
    RegVal {
        offset: 0xE8,
        value: 0x100000,
    },
    RegVal {
        offset: 0xEC,
        value: 0x0,
    },
    RegVal {
        offset: 0xF4,
        value: 0xF022F,
    },
    RegVal {
        offset: 0x100,
        value: 0xC070507,
    },
    RegVal {
        offset: 0x104,
        value: 0x5040B,
    },
    RegVal {
        offset: 0x108,
        value: 0x4070C0D,
    },
    RegVal {
        offset: 0x10C,
        value: 0x505000,
    },
    RegVal {
        offset: 0x110,
        value: 0x3040204,
    },
    RegVal {
        offset: 0x114,
        value: 0x4050303,
    },
    RegVal {
        offset: 0x118,
        value: 0x1010004,
    },
    RegVal {
        offset: 0x11C,
        value: 0x301,
    },
    RegVal {
        offset: 0x120,
        value: 0x303,
    },
    RegVal {
        offset: 0x130,
        value: 0x40000,
    },
    RegVal {
        offset: 0x134,
        value: 0x100002,
    },
    RegVal {
        offset: 0x138,
        value: 0x2F,
    },
    RegVal {
        offset: 0x180,
        value: 0xA200A2,
    },
    RegVal {
        offset: 0x184,
        value: 0x900000,
    },
    RegVal {
        offset: 0x190,
        value: 0x7040000,
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
        value: 0x905092C,
    },
    RegVal {
        offset: 0x244,
        value: 0x101,
    },
    RegVal {
        offset: 0x250,
        value: 0x1F00,
    },
    RegVal {
        offset: 0x0490, // PCTRL_N
        value: 0x1,
    },
];

// https://www.rockchip.fr/RK809%20datasheet%20V1.01.pdf
const PMIC_ADDR: u8 = 0x20;

const DUMP_OTP_NS: bool = false;

// U-Boot: drivers/ram/rockchip/sdram_rv1126.c  rv1126_dmc_init
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
    // NOTE: This should get the DRAM size; U-Boot:
    // rk3568_dmc_probe
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
    let post_init = true;
    // sdram_init_detect ?
    sdram_init(post_init);
}
