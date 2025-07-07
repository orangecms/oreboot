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

use crate::arm::{get_time, udelay};
use crate::i2c::{i2c_init, i2c_read};
use crate::mem_map::{
    CRU_NS_BASE, CRU_S_BASE, DDR_GRF_BASE, DDR_PHY_BASE, PMU_GRF_BASE, RESX_BASE, SRAM_BASE,
    SYS_SGRF_BASE, UPCTL2_BASE,
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

// aka MSCH, another sorta GRF? The manual just say "reserved".
const RESX_0008: usize = RESX_BASE + 0x0008;
const RESX_000C: usize = RESX_BASE + 0x000c;
const RESX_0010: usize = RESX_BASE + 0x0010;
const RESX_0014: usize = RESX_BASE + 0x0014;
const RESX_0018: usize = RESX_BASE + 0x0018;
const RESX_001C: usize = RESX_BASE + 0x001c;
const RESX_0020: usize = RESX_BASE + 0x0020;
const RESX_0024: usize = RESX_BASE + 0x0024;

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
const DDR_PHY_0084: usize = DDR_PHY_BASE + 0x0084;
const DDR_PHY_008C: usize = DDR_PHY_BASE + 0x008c;
const DDR_PHY_0094: usize = DDR_PHY_BASE + 0x0094;
const DDR_PHY_00A0: usize = DDR_PHY_BASE + 0x00a0;
const DDR_PHY_00AC: usize = DDR_PHY_BASE + 0x00ac;
const DDR_PHY_00C0: usize = DDR_PHY_BASE + 0x00c0;
// PLL
const DDR_PHY_00D0: usize = DDR_PHY_BASE + 0x00d0;
const DDR_PHY_00F0: usize = DDR_PHY_BASE + 0x00f0;
const DDR_PHY_00F4: usize = DDR_PHY_BASE + 0x00f4;
const DDR_PHY_00F8: usize = DDR_PHY_BASE + 0x00f8;
const DDR_PHY_01B0: usize = DDR_PHY_BASE + 0x01b0;
const DDR_PHY_01F4: usize = DDR_PHY_BASE + 0x01f4;
const DDR_PHY_020C: usize = DDR_PHY_BASE + 0x020C;
const DDR_PHY_0230: usize = DDR_PHY_BASE + 0x0230;

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
const PMU_GRF_OS3: usize = PMU_GRF_BASE + 0x020c;

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
fn phy_cfg(phy_cfg: &PhyCfg, chan_cfg: &Config) {
    phy_pll_set(chan_cfg.dram_freq * MEGA, 0);

    for p in phy_cfg.iter() {
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

    let vx = match chan_cfg.chan_bus_width {
        1 => v | ((1 << v0) | (1 << v1)) << 8,
        2 => v | 0x0f00,
        _ => v | 0x100 << v0,
    };
    const ENABLE_ECC: bool = false;
    let mut vxo = if ENABLE_ECC { vx | 0x1000 } else { vx };

    if chan_cfg.rank == 4 {
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

// TODO: split up into ChannelConfig + BaseParams ?
// U-Boot calls the first part sdram_cap_info as part of "channel"
// (which also includes the NOC timings) and the second part sdram_base_params.
struct Config {
    rank: u32,
    column: u32,
    bank_num: u32,
    chan_bus_width: u32,
    die_bus_width: u32,
    row_3_4: u32,
    cs0_row: u32,
    cs1_row: u32,
    cs0_high16bit_row: u32,
    cs1_high16bit_row: u32,
    ddr_config: u32,
    unk1: u32,

    dram_freq: u32,
    dram_type: u32,
    num_channels: u32,
    stride: u32,
    odt: u32,
}

// U-Boot arch/arm/include/asm/arch-rockchip/sdram_rv1126.h
struct MschNocTimings {
    ddrtiminga0: u32,
    ddrtimingb0: u32,
    ddrtimingc0: u32,
    ddr4_timing: u32,
    devtodev: u32,
    ddr_mode: u32,
    agingx: u32,
}

// NOTE: These stem from the global config.
// They are called sr_idle and pd_idle in U-Boot and copied at runtime.
const SELF_REFRESH_IDLE: u32 = 0x005d;
const POWER_DOWN_IDLE: u32 = 0x000d;

// sdram_init_ / sdram_init_detect ?
fn sdram_init(
    cfg: &Config,
    msch_timings: &mut MschNocTimings,
    post_init: bool,
    ctl_cfg_mstr: Option<u32>,
) {
    println!("sdram_init");
    let ctl_cfg_mstr = ctl_cfg_mstr.unwrap_or(UPCTL2_CFG3_MSTR_DEFAULT);

    write32(DDR_GRF_CTRL0, 0x0002_0000);

    clk_set_dpll((cfg.dram_freq * MEGA) / 2);

    // reset 1
    write32(SYS_SGRF_0014, 0x0b00_0b00);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0002);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0180);

    udelay(10);

    // reset 2
    write32(SYS_SGRF_0014, 0x0b00_0b00);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0002);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0100);

    // TODO: What is the possible value range?
    // This check may be unnecessary.
    if cfg.dram_type < 9 {
        let m1 = if cfg.dram_type == 8 { 7 } else { cfg.dram_type };

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
    phy_cfg(&PHY_CFG3, cfg);

    // TODO: tweak this
    {
        // Each loop has 5 iterations
        match cfg.dram_type {
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
        // bit 0: pvt compensation disable
        write32(DDR_PHY_00C0, v | 1);

        // bit 10: enable 2t timing mode
        if ctl_cfg_mstr & (1 << 10) != 0 {
            let v = read32(DDR_PHY_00C0);
            // bit 18: cmd delay one ui
            // bit 17: cmd 2t mode
            write32(DDR_PHY_00C0, v | (0b11 << 17));
        }

        let v = read32(DDR_PHY_00AC);
        // bit 4: freq choose b
        write32(DDR_PHY_00AC, v | (1 << 4));

        let v = read32(DDR_PHY_0044);
        // bits 30..31: freq choose t; 00 = freq point 0
        write32(DDR_PHY_0044, v & !(0b11 << 30));
    }

    // reset?
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0000);

    write32(SYS_SGRF_0014, 0x0b00_0300);
    write32(CRU_S_CLK_SEL_CFG66, 0x0002_0000);
    write32(CRU_NS_SOFT_RESET_CFG27, 0x0180_0000);

    // U-Boot sdram_init_
    write32(UPCTL2_MSTR, ctl_cfg_mstr);
    upctl2_config(&UPCTL2_CFG3, SELF_REFRESH_IDLE, POWER_DOWN_IDLE);

    let v = read32(UPCTL2_PCFGR_N);
    write32(UPCTL2_PCFGR_N, v | (1 << 16));

    // drivers/ram/rockchip/sdram_rv1126.c
    // set frequency_mode
    let v = read32(UPCTL2_MSTR);
    write32(UPCTL2_MSTR, v | (1 << 29));
    // set target_frequency to Frequency 0
    let v = read32(UPCTL2_MSTR2);
    write32(UPCTL2_MSTR2, v & !(0b11));

    set_ds_odt(cfg.dram_freq, cfg.dram_type, 0);

    // similar to arch/arm/mach-rockchip/rk3036/sdram_rk3036.c  sdram_all_config
    // 0xd (13)
    let bw_plus_col = cfg.chan_bus_width + cfg.column;

    // 0 | (3 << 5) | 3 = 0b0110_0011 = 0x63
    // TODO: is this osreg?
    let vt = ((cfg.rank - 1) << 8) | ((cfg.cs0_row - 13) << 5) | (bw_plus_col - 10);
    println!("vt {vt:08x}");

    // bank_num is 3 -> 0x6b
    let vxx = if cfg.bank_num == 3 { vt | 8 } else { vt };

    // see calculate_ddrconfig

    // This searches for an index to address map data.
    // We currently hardcode that list.
    // TODO: actually use this resulting value.
    // Possible resulting values: 0..=8, 14, 17
    let idx = find_addrmap_index(vxx).unwrap_or_else(|| {
        if cfg.bank_num == 3 && bw_plus_col == 10 {
            14
        } else if cfg.rank != 1 || cfg.bank_num != 3 || cfg.cs0_row > 17 || bw_plus_col != 13 {
            // NOTE: This should never happen.
            // Do the check at build time, and do it earlier instead of here.
            panic!("DDR config error")
        } else {
            // We should be here.
            17
        }
    });
    // NOTE: unused; original code mutates config field12 (offset 0x30).
    let s_0030 = idx as u32;
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

    if cfg.rank == 1 {
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

    let vf = phy_measure_xx(cfg.dram_freq);
    println!("vf: {vf} (0x{vf:02x})");
    // This is used to adjust registers in multiple blocks.
    let a = (vf << 24) | (vf << 8);

    const BLOCK_SIZE: usize = 0x180;
    const BLOCK_COUNT: usize = 5;
    let mask = !((0x7f << 24) | (0x7f << 8));
    // NOTE: rank apparently could be hardcoded at build time.
    for i in 0..cfg.rank {
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

    // read train frequency update
    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v | (1 << 2));
    let v = read32(DDR_PHY_0094);
    write32(DDR_PHY_0094, v & !(1 << 2));

    if cfg.dram_type == 6 {
        todo!()
    } else if cfg.dram_type == 7 || cfg.dram_type == 8 {
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
    }

    train(
        0, // cs
        cfg,
        0,
        FlagSet::<TrainingFlag>::from(TrainingFlag::ReadGate),
    );

    let mr14 = upctl2_read_mr(1, 14, 7);
    // Does this value look familiar? Yes? We need to handle this!
    if mr14 == 0x4d {
        if cfg.dram_type < 9 {
            let v = read32(UPCTL2_INIT7);
            upctl2_write_mr(15, 14, v as u16, 7);
        }
        if post_init {
            for cs in 1..cfg.rank {
                println!("r/cs {cs}");
                train(
                    cs,
                    cfg,
                    0,
                    FlagSet::<TrainingFlag>::from(TrainingFlag::ReadGate),
                );
            }
        }

        dram_all_config(&cfg, msch_timings, s_0030);
        enable_low_power();
    }
    println!("sdram_init done");
}

// U-Boot drivers/ram/rockchip/sdram_rv1126.c dram_detect_cs1_row
fn dram_detect_cs1_row(cfg: &Config, channel: u32) -> u32 {
    if (channel < 2 && cfg.rank < 2) || (channel > 1 && cfg.rank != 4) {
        return 0;
    }

    let (mem_base, cs_add) = match (cfg.rank, channel) {
        (2..4, ..2) => (RAM_BASE, 0),
        _ => (RAM_BASE + 0x1000, 1),
    }; // 4K offset

    let cs_pst = read32(UPCTL2_ADDR_MAP_BASE) & 0x1f + 6 + 2;
    let cs_add = if cs_pst < 28 { cs_add + 1 } else { cs_add };

    let cs0_cap = 1 << (cs_pst & 0x1f);

    let bank = if cfg.dram_type == 0 {
        if cfg.die_bus_width == 0 {
            cfg.bank_num + 2
        } else {
            cfg.bank_num + 1
        }
    } else {
        0
    };

    let mask = if cfg.chan_bus_width == 2 {
        0xffff
    } else {
        0xff
    };

    // I thin that's it.
    let x = if cs_pst == 31 { 1 } else { 0 };
    let max_row = 32 + x - bank - cfg.column - cfg.chan_bus_width - cs_add;

    let row = if cfg.cs0_row < max_row {
        cfg.cs0_row
    } else {
        max_row
    };

    const PATTERN: u32 = 0x5aa5_f00f;

    let base = match channel {
        2 => mem_base,
        3 => mem_base + cs0_cap,
        _ => cs0_cap,
    };

    for r in 0..(row - 12) {
        let rr = row - r;
        write32(base, 0);

        let o = 1 << (rr + bank + cfg.column + cfg.chan_bus_width + cs_add - 1);
        let a = base + o;
        write32(a, PATTERN);
        let x = read32(a);
        if x & mask == PATTERN & mask && a & (mask as usize) == 0 {
            return rr;
        }
    }

    12
}

// U-Boot drivers/ram/rockchip/sdram_rv1126.c dram_all_config
fn dram_all_config(cfg: &Config, timings: &mut MschNocTimings, s_0030: u32) {
    // MSCH device config
    write32(RESX_0008, s_0030);

    let (os2, os3) = sdram_org_config(&cfg, 0);

    println!(" OS2 {os2:08x} OS3 {os3:08x}");

    write32(PMU_GRF_OS2, os2);
    write32(PMU_GRF_OS3, os3);

    let (m1, m2) = {
        let m1 = get_dram_size_factor(&cfg, 0, cfg.dram_type);
        let m2 = get_dram_size_factor(&cfg, 1, cfg.dram_type);
        if cfg.rank == 4 {
            todo!("rank == 4")
        } else if cfg.rank == 2 {
            todo!("rank == 2")
        } else {
            (m1, m2)
        }
    };

    let v = ((m2 >> 26) as u32 & 0xff) << 8 | ((m1 >> 26) as u32 & 0xff);
    // MSCH device size ?
    write32(RESX_000C, v);

    update_noc_timing(&cfg, timings);
    println!("update_noc_timing DONE");
}

// timingc0
//   0..3: burst penalty
//   4..7: reserved
//   8..13: wr to mwr
//   14..31: reserved
const BURST_PENALTY_MASK: u32 = 0b1111;
const WR_TO_MWR_MASK: u32 = 0b111111 << 8;
// ddr_mode
//   0: auto precharge
//   1: bypass filtering
//   2: faw bank
//   3..4: burst size
//   5..6: mwr size
//   7: reserved
//   8..15: force order
//   16..23: force order state
//   24..31: reserved
const BURST_SIZE_MASK: u32 = 0b11 << 3;
const MWR_SIZE_MASK: u32 = 0b11 << 5;

// U-Boot drivers/ram/rockchip/sdram_rv1126.c update_noc_timing
fn update_noc_timing(cfg: &Config, timings: &mut MschNocTimings) {
    let bus_width = 8 << (cfg.chan_bus_width & 0x1f);
    let burst_length = ((read32(UPCTL2_MSTR) >> 16) & 0xf) << 1;

    let bl_bw_8 = burst_length * (bus_width / 8);

    let burst_size = match bl_bw_8 {
        16 => 0,
        32 => 1,
        64 => 2,
        _ => 3,
    };

    let f = (16 / bl_bw_8).min(1);
    let burst_penalty = f * burst_length / 2;

    let (tc0, mode) = if cfg.dram_type < 9 {
        let wrtomwr = 3 * burst_penalty;
        let m = !(WR_TO_MWR_MASK | BURST_PENALTY_MASK);
        let tc0 = timings.ddrtimingc0 & m | (wrtomwr << 8) | burst_penalty;

        let mwr_size = if bus_width == 16 { 1 } else { 2 };
        let m = !(MWR_SIZE_MASK | BURST_SIZE_MASK);
        let mode = timings.ddr_mode & m | (mwr_size << 5) | (burst_size << 3);

        (tc0, mode)
    } else {
        let m = !BURST_SIZE_MASK;
        let mode = timings.ddr_mode & m | (burst_size << 3);

        let m = !BURST_PENALTY_MASK;
        let tc0 = timings.ddrtimingc0 & m | burst_penalty;

        (tc0, mode)
    };
    // NOTE: Those are now mutations. Not sure if really necessary.
    timings.ddrtimingc0 = tc0;
    timings.ddr_mode = mode;

    write32(RESX_0010, timings.ddrtiminga0);
    write32(RESX_0014, timings.ddrtimingb0);
    write32(RESX_0018, timings.ddrtimingc0);
    write32(RESX_0020, timings.devtodev);
    write32(RESX_0024, timings.ddr_mode);
    write32(RESX_001C, timings.ddr4_timing);
}

// TODO: What is p2?
fn get_dram_size_factor(cfg: &Config, p2: u32, dram_type: u32) -> u64 {
    let x0 = if dram_type == 0 {
        match cfg.die_bus_width {
            0 => 2,
            _ => 1,
        }
    } else {
        0
    };
    // 1 + 11 + 3 = 15
    let x0 = (x0 + cfg.chan_bus_width + cfg.column + cfg.bank_num) as u64;

    // 15 + 17 = 32 (0x20)
    let s_pow = (x0 + cfg.cs0_row as u64) & 0x3f;
    let x1: u64 = 1 << s_pow; // 2 ** s_pow

    let (x2, x3, x4) = if cfg.rank > 1 {
        todo!("rank > 1")
    } else {
        (0, 0, 0)
    };

    match p2 {
        0 => x1,
        1 => x2,
        _ => x1 + x2 + x3 + x4,
    }
}

// U-Boot drivers/ram/rockchip/sdram_common.c sdram_org_config
fn sdram_org_config(cfg: &Config, channel: u32) -> (u32, u32) {
    let v1 = cfg.dram_type << 13;
    let v2 = v1 | (cfg.num_channels - 1) * 0x1000;
    let s1 = (channel + 0x1e) & 0x1f;
    let s2 = (channel + 0x1c) & 0x1f;
    let v3 = v2 | cfg.row_3_4 << s1 | 1 << s2;
    let x16 = channel * 16;
    let v4 = v3 | (cfg.rank - 1) << ((x16 + 11) & 0x1f);
    let v5 = v4 | (cfg.column - 9) << ((x16 + 9) & 0x1f);
    let v6 = v5 | ((cfg.bank_num != 3) as u32) << ((x16 + 8) & 0x1f);
    let v7 = v6 | (2 >> (cfg.chan_bus_width & 0x1f)) << ((x16 + 2) & 0x1f);
    let v8 = v7 | (2 >> (cfg.die_bus_width & 0x1f)) << (x16 & 0x1f);

    let rm13 = cfg.cs0_row - 13;
    let x2 = channel * 2;

    let res1 = v8 | (rm13 & 3) << ((x16 + 6) & 0x1f);
    let res2 = (rm13 >> 2 & 1) << ((x2 + 5) & 0x1f);
    let (res1, res2) = if cfg.cs1_row == 0 {
        (res1, res2)
    } else {
        let v1 = res1 & (3 << ((x16 + 4) & 0x1f) ^ 0xffff_ffff);
        let v2 = res2 & (1 << ((x2 + 4) & 0x1f) ^ 0xffff_ffff);
        let res1 = v1 | (rm13 & 3) << ((x16 + 4) & 1);
        let res2 = v2 | ((rm13 >> 2) & 1) << ((x2 + 4) & 0x1f);
        (res1, res2)
    };
    let res2 = res2 | 0x2000_0000 | ((cfg.column - 9) << (x2 & 0x1f));
    (res1, res2)
}

fn ddr_set_rate() {
    //
}

// U-Boot drivers/ram/rockchip/sdram_rv1126.c enable_low_power
// see also low_power_update
fn enable_low_power() {
    write32(DDR_GRF_CTRL1, 0x1f1f_0617);

    // enable assertion of DFI DRAM clock disable
    let v = read32(UPCTL2_POWER_CTRL);
    write32(UPCTL2_POWER_CTRL, v | (1 << 3));

    // power down setting
    let v = read32(UPCTL2_POWER_CTRL);
    if POWER_DOWN_IDLE != 0 {
        write32(UPCTL2_POWER_CTRL, v | (1 << 1));
    } else {
        write32(UPCTL2_POWER_CTRL, v & !(1 << 1));
    }

    // self-refresh setting
    let v = read32(UPCTL2_POWER_CTRL);
    if SELF_REFRESH_IDLE != 0 {
        write32(UPCTL2_POWER_CTRL, v | (1 << 0));
    } else {
        write32(UPCTL2_POWER_CTRL, v & !(1 << 0));
    }
}

const DFI_LOW_POWER_BYPASS: u32 = 1 << 15;

fn low_power_update(x: u32) -> u32 {
    if x != 0 {
        let v = read32(DDR_PHY_0084);
        write32(DDR_PHY_0084, v & !DFI_LOW_POWER_BYPASS);
        let v = read32(UPCTL2_POWER_CTRL);
        write32(UPCTL2_POWER_CTRL, v | (x & 0xf));
    }
    // bit 0: self refresh enable
    // bit 1: power down enable
    // bit 2: deep power down enable
    // bit 3: DFI DRAM clock disable
    let pwr_ctl = read32(UPCTL2_POWER_CTRL);
    write32(UPCTL2_POWER_CTRL, pwr_ctl & !0xf);
    let v = read32(DDR_PHY_0084);
    write32(DDR_PHY_0084, v & DFI_LOW_POWER_BYPASS);

    pwr_ctl
}

// see U-Boot include/configs/rk3568_common.h CFG_SYS_SDRAM_BASE
const RAM_BASE: usize = 0x0;
// + 128K
const SHARE_MEM_BASE: usize = RAM_BASE + 0x10_0000;

// NOTE: Start at an offset to avoid accessing address 0, on which Rust errors.
fn dram_test() {
    let b = 0x1000;
    let pattern = 0xffaa_5500;
    for o in (0..64).step_by(4) {
        let a = b + o;
        // println!("write pattern {pattern:08x} to {a:08x}");
        write32(a, pattern);
    }
    for o in (0..64).step_by(4) {
        let a = b + o;
        let p = read32(a);
        println!("read back     {p:08x}  @ {a:08x}");
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
fn train(
    cs: u32,
    cfg: &Config,
    dst_fsp: u32,
    training_flags: FlagSet<TrainingFlag>,
) -> Result<(), ()> {
    if training_flags.contains(TrainingFlag::WriteLeveling) {
        train_write_leveling(cfg.rank, cs, cfg.dram_type);
    }
    if training_flags.contains(TrainingFlag::ReadGate) {
        if train_read_gate(cs, cfg.dram_type).is_err() {
            return Err(());
        }
    }
    if training_flags.contains(TrainingFlag::Read) {
        todo!("train_read")
    }
    if training_flags.contains(TrainingFlag::Write) {
        todo!("train_write")
    }

    Ok(())
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

fn train_write_leveling(rank: u32, cs: u32, dram_type: u32) {
    let was_auto_zq_enabled = upctl2_disable_zq_cs();

    let v = read32(DDR_PHY_00A0);
    // disable DQ write train auto
    write32(DDR_PHY_00A0, v & !1);

    let cur_fsp = read32(UPCTL2_MSTR2) & 0b11;
    let o = get_fsp_offset(cur_fsp);
    let r = UPCTL2_INIT3 + o;
    let init3 = read32(r);

    let wl_load_mode = if dram_type == 0 || dram_type == 3 {
        init3 & 0x3fff | 0x4000
    } else {
        init3 & 0xff
    };
    // write leveling load mode
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & 0x0000_ffff | (wl_load_mode << 16));

    if (dram_type == 0 || dram_type == 3) && rank == 2 {
        todo!()
    }

    // 8..11: write leveling cs select
    let m = 0b1111;
    let s = 8;
    let nv = !(1 << (cs & 0x1f)) & m;
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & !(m << s) | (nv << s));
    // Start write leveling.
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v | (1 << 6));

    let v = read32(DDR_PHY_0004);
    println!("DDR_PHY_0004 {v:08x} cs {:04b}", (v >> s) & m);

    check_wl();

    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & !(1 << 6));
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & !(m << s));

    if (dram_type == 0 || dram_type == 3) && rank == 2 {
        todo!()
    }

    upctl2_restore_zq_cs(was_auto_zq_enabled);
    upctl2_dbg_rank01_refresh(8);

    // TODO: another read gate training?!
}

fn check_wl() {
    // each of these bits is for each of bytes 0..4
    let m = 0b11111;
    let s = 8;
    let t0 = get_time();
    let v0 = (read32(DDR_PHY_0000) >> s) & m;
    for _ in 0..1000 {
        let v = read32(DDR_PHY_020C);
        if (v >> s) & m == v0 {
            let t1 = crate::arm::get_time();
            println!("write leveling done in {}us", t1 - t0);
        }
        udelay(1);
    }
    let t1 = crate::arm::get_time();
    println!("write leveling timeout after {}us", t1 - t0);
    let v = (read32(DDR_PHY_020C) >> s) & m;
    panic!("{v0:05b} != {v:05b}");
}

const RANK4_ENABLED: u32 = 1 << 20;

fn train_read_gate(cs: u32, dram_type: u32) -> Result<(), ()> {
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

    let was_auto_zq_enabled = upctl2_disable_zq_cs();

    if dram_type == 0 {
        todo!()
    }

    // bit 2..5: cal_cs_sel
    // Position of 0 determines rank: 0b1110 means rank 0, 0b0111 means rank 3.
    // 0b0000 means RX-DQS calibration result auto-switches as per DFI command
    // after RX-DQS training.
    let m = 0b1111;
    let s = 2;
    let nv = !(1 << (cs & 0x1f)) & m;
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & !(m << s) | (nv << s));
    // Start RX-DQS calibration.
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v | 1);

    let Ok(cal_res) = check_rx_dqs_calibration() else {
        return Err(());
    };

    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & !1);
    let v = read32(DDR_PHY_0004);
    write32(DDR_PHY_0004, v & 0xffff_ffc3);

    upctl2_restore_zq_cs(was_auto_zq_enabled);
    upctl2_dbg_rank01_refresh(8);

    let channel_en = (phy0000 >> 8) & 0b11111;
    if DEBUG {
        println!("Channel A DQ 0..7  enabled: {}", channel_en & (1 << 0) != 0);
        println!("Channel A DQ 8..15 enabled: {}", channel_en & (1 << 1) != 0);
        println!("Channel B DQ 0..7  enabled: {}", channel_en & (1 << 2) != 0);
        println!("Channel B DQ 8..15 enabled: {}", channel_en & (1 << 3) != 0);
        println!("Channel C DQ 0..7  enabled: {}", channel_en & (1 << 4) != 0);
    }

    // FIXME: DO NOT PANIC! Return an error instead. This is used for detection.
    if channel_en != cal_res {
        println!("channel_en does not match calibration result: {cal_res}");
        return Err(());
    }

    // restore
    for r in (DDR_PHY_0300..DDR_PHY_0300 + 0x0a80).step_by(0x180) {
        write32(r, phy0300);
    }

    if rank4_was_disabled {
        let v = read32(DDR_PHY_0000);
        write32(DDR_PHY_0000, v & !RANK4_ENABLED);
    }

    // TODO: What is this value?! (not covered in the manual)
    let v = read32(DDR_PHY_0448);
    let xx = v >> ((cs & 1) * 16);
    // NOTE: DO NOT PANIC! Return an error instead. This is used for detection.
    if xx & 0x7ff != 0 {
        println!("read gate training error; DDR_PHY_0448: {v:08x} {cs} {xx:03x}");
        Err(())
    } else {
        Ok(())
    }
}

fn check_rx_dqs_calibration() -> Result<u32, ()> {
    let t0 = get_time();
    for _ in 0..50 {
        let v = read32(DDR_PHY_020C);
        if v & (1 << 5) != 0 {
            println!("RX-DQS calibration error");
            return Err(());
        }
        if v & (1 << 6) != 0 {
            let t1 = crate::arm::get_time();
            println!("RX-DQS calibration done in {}us", t1 - t0);
            // each of the lowest bits means calibration done for byte 0..4
            return Ok(v & 0b11111);
        }
        udelay(1);
    }
    Ok(0)
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

// NOTE: The first value here, MSTR, is being changed to 0x4304_1401 ( | 0x400 )
// in the first round of dram_init_main.
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

const UPCTL2_CFG3_MSTR_DEFAULT: u32 = 0x8308_1020;

// NOTE: The first value here was for UPCTL2_MSTR.
// It needs to be changed multiple times, so exclude it here.
// Instead, we pass along an Option and fall back to the const aobve.
const UPCTL2_CFG3: [RegVal; 30] = [
    RegVal {
        // UPCTL2_REFRESH_TIMING
        offset: 0x0064,
        value: 0x0013_002E,
    },
    RegVal {
        // UPCTL2_INIT0
        offset: 0x00D0,
        value: 0x0002_013E,
    },
    RegVal {
        // UPCTL2_INIT1
        offset: 0x00D4,
        value: 0x0021_0000,
    },
    RegVal {
        // UPCTL2_INIT2
        offset: 0x00D8,
        value: 0x0000_0202,
    },
    RegVal {
        // UPCTL2_INIT3
        offset: 0x00DC,
        value: 0x0024_0012,
    },
    RegVal {
        // UPCTL2_INIT4
        offset: 0x00E0,
        value: 0x0031_0000,
    },
    RegVal {
        // UPCTL2_INIT6
        offset: 0x00E8,
        value: 0x0010_0000,
    },
    RegVal {
        // UPCTL2_INIT7
        offset: 0x00EC,
        value: 0x0,
    },
    RegVal {
        // UPCTL2_RANK_CTRL
        offset: 0x00F4,
        value: 0x0000_F022F,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING0
        offset: 0x0100,
        value: 0x0C07_0507,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING1
        offset: 0x0104,
        value: 0x0005_040B,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING2
        offset: 0x0108,
        value: 0x0040_70C0D,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING3
        offset: 0x010C,
        value: 0x0050_5000,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING4
        offset: 0x0110,
        value: 0x3040204,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING5
        offset: 0x0114,
        value: 0x0405_0303,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING6
        offset: 0x0118,
        value: 0x0101_0004,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING7
        offset: 0x011C,
        value: 0x0000_0301,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING8
        offset: 0x0120,
        value: 0x0000_0303,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING12
        offset: 0x0130,
        value: 0x0004_0000,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING13
        offset: 0x0134,
        value: 0x0010_0002,
    },
    RegVal {
        // UPCTL2_DRAM_TIMING14
        offset: 0x0138,
        value: 0x0000_002F,
    },
    RegVal {
        // UPCTL2_ZQ_CTRL0
        offset: 0x0180,
        value: 0x00A2_00A2,
    },
    RegVal {
        // UPCTL2_ZQ_CTRL1
        offset: 0x0184,
        value: 0x0090_0000,
    },
    RegVal {
        // UPCTL2_DFI_TIMING0
        offset: 0x190,
        value: 0x0704_0000,
    },
    RegVal {
        // UPCTL2_DFI_LP_CFG0
        offset: 0x0198,
        value: 0x0A00_0101,
    },
    RegVal {
        // UPCTL2_DFI_UPDATE0
        offset: 0x01A0,
        value: 0xC040_0003,
    },
    RegVal {
        // UPCTL2_ODT_CFG
        offset: 0x0240,
        value: 0x0905_092C,
    },
    RegVal {
        // UPCTL2_ODT_MAP
        offset: 0x0244,
        value: 0x0000_0101,
    },
    RegVal {
        // UPCTL2_SCHED
        offset: 0x0250,
        value: 0x0000_1F00,
    },
    RegVal {
        // UPCTL2_PCTRL_N
        offset: 0x0490,
        value: 0x0000_0001,
    },
];

// U-Boot ddr_set_rate_for_fsp
fn ddr_set_rate_for_fsp(cfg: &Config) {
    // U-Boot get_wrlvl_val
    let p_res = low_power_update(0);
    // NOTE: code here omitted; should be disabled
    let p_res = low_power_update(p_res);

    let odt_cfg = get_ddr_drv_odt_info(cfg.dram_type);

    // 0x210 0x144 0x210 0x210
    let freq0 = odt_cfg.ddr_freq_f0_f1 & 0xfff;
    let freq1 = (odt_cfg.ddr_freq_f0_f1 >> 12) & 0xfff;
    let freq2 = odt_cfg.ddr_freq_f2_f3 & 0xfff;
    let freq3 = (odt_cfg.ddr_freq_f2_f3 >> 12) & 0xfff;

    // TODO: zero out FSP params storage ...?
    write32(SHARE_MEM_BASE, 0x0);

    let p_res = low_power_update(0);

    const CMD_INV_DELAY_SEL_MASK: u32 = !(0b111111 << 6);
    let (v1, v2) = if cfg.dram_type < 9 {
        // PHY_01B0 10..15: cmd_invdelaysel
        // command TX delay line value OBS signal
        let v = read32(DDR_PHY_01B0);
        write32(DDR_PHY_01B0, v & CMD_INV_DELAY_SEL_MASK | 0x6000);

        let v1 = read32(DDR_PHY_0230) >> 16;

        let v = read32(DDR_PHY_01B0);
        write32(DDR_PHY_01B0, v & CMD_INV_DELAY_SEL_MASK | 0x8000);

        let v2 = read32(DDR_PHY_0230) >> 16;

        (v1, v2)
    } else {
        todo!()
    };

    train(
        0,
        cfg,
        0,
        FlagSet::<TrainingFlag>::from(TrainingFlag::WriteLeveling),
    );
}

fn upctl2_cfg_adjust_mstr(val: u32, cfg: &Config) -> u32 {
    // bits 12..13: data bus width
    // bits 24..25: active ranks
    // bits 30..31: device config
    let m = (0b11 << 30) | (0b11 << 24) | (0b11 << 12);
    let vx = val & !m;
    let dcfg = match cfg.die_bus_width {
        1 => 1 << 31,
        2 => (1 << 31) | (1 << 30),
        _ => 1 << 30,
    };
    let active_ranks = (1 << (cfg.rank & 0x1f)) - 1;
    let data_bus_width = 2 - cfg.chan_bus_width;
    // 12..13: data bus width
    // 24..25: active ranks
    vx | dcfg | (active_ranks << 24) | (data_bus_width << 12)
}

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

    // the vendor code has 4 configs and tries them in that order:
    // - DDR3
    // - DDR4
    // - LPDDR3
    // - LPDDR4

    // U-Boot defines this config in multiple parts, each defined in
    //  arch/arm/include/asm/arch-rockchip/sdram_common.h, e.g., sdram_cap_info
    // The config parts are assembled into one bigger struct in
    //  arch/arm/include/asm/arch-rockchip/sdram_rv1126.h
    let mut cfg = Config {
        rank: 1,
        column: 11,
        bank_num: 3,       // power of 2, i.e., 2^3=8
        chan_bus_width: 1, // 1 means 16bit
        die_bus_width: 1,  // 0 means 8bit
        row_3_4: 0,        // 0 means normal die, power of 2
        cs0_row: 17,
        cs1_row: 17,
        cs0_high16bit_row: 0,
        cs1_high16bit_row: 0,
        ddr_config: 0,
        unk1: 0,

        dram_freq: 324,
        dram_type: DdrType::LPDDR4 as u32,
        num_channels: 1,
        stride: 0,
        odt: 0,
    };

    let mut msch_timings = MschNocTimings {
        ddrtiminga0: 0x2F0D_060A,
        ddrtimingb0: 0x0602_0804,
        ddrtimingc0: 0x0000_0C04,
        ddr4_timing: 0x0000_0000,
        devtodev: 0x0000_1111,
        ddr_mode: 0x0000_0054,
        agingx: 0x0000_00FF,
    };

    let post_init = false;
    // sdram_init_detect ?
    sdram_init(&cfg, &mut msch_timings, post_init, None);

    if cfg.dram_type < 9 && cfg.dram_freq > 333 {
        todo!("data training with all options")
    }

    // NOTE: vendor code sets chan_bus_width to 1 here

    // LPDDR4 + LPDDR4X
    if cfg.dram_type == 7 || cfg.dram_type == 8 {
        let mr8 = upctl2_read_mr(1, 8, cfg.dram_type);
        let x = mr8 as u32 >> 2;
        cfg.column = 10;
        cfg.bank_num = 3;
        cfg.die_bus_width = 1;
        cfg.row_3_4 = x & 1;
        cfg.cs0_row = 14 + (((x & 0xf) + 1) >> 1);
        cfg.chan_bus_width = 2;
    } else if cfg.dram_type == 0 {
        todo!("DDR3")
    } else {
        todo!("NOT DDR3 nor LPDDR4(X)")
    }

    let power_ctl = read32(UPCTL2_POWER_CTRL);
    write32(UPCTL2_POWER_CTRL, 0);
    let xx = train(
        1,
        &cfg,
        0,
        FlagSet::<TrainingFlag>::from(TrainingFlag::ReadGate),
    );

    // LPDDR3, LPDDR4, LPDDR4X
    let v = if cfg.dram_type == 6 || cfg.dram_type == 7 || cfg.dram_type == 8 {
        println!("LPDDR3, LPDDR4 or LPDDR4X - detect CS 4");
        if xx.is_ok() {
            if train(
                3,
                &cfg,
                0,
                FlagSet::<TrainingFlag>::from(TrainingFlag::ReadGate),
            )
            .is_ok()
            {
                // NOTE: On success, vendor code prints "detect 4 cs"
                println!("CS 4 deteced");
                3
            } else {
                println!("CS 4 not deteced");
                1
            }
        } else {
            0
        }
    } else {
        xx.is_ok() as u32
    };
    cfg.rank = v + 1;

    if cfg.dram_type != 7 && cfg.dram_type != 8 {
        todo!("handle DDR type not LPDDR4 nor LPDDR4X")
    }
    // restore
    write32(UPCTL2_POWER_CTRL, power_ctl);

    cfg.ddr_config = cfg.cs0_row;

    if v == 0 {
        cfg.cs1_row = 0;
        cfg.unk1 = 0;
    } else {
        cfg.cs1_row = cfg.cs0_row;
        cfg.unk1 = cfg.cs0_row;
    }

    // This is the value written to UPCTL2_MSTR.
    let v = UPCTL2_CFG3_MSTR_DEFAULT;
    let mstr = upctl2_cfg_adjust_mstr(v, &cfg);

    let post_init = true;
    sdram_init(&cfg, &mut msch_timings, post_init, Some(mstr));

    todo!("more code");

    let cs1_row = dram_detect_cs1_row(&cfg, 1);
    // NOTE: original code overwrites the config! Is this necessary?
    cfg.cs1_row = cs1_row;

    if cs1_row != 0 {
        let d = cs1_row - 13;
        let v = read32(PMU_GRF_OS2);
        // U-Boot has a fancy macro, SYS_REG_ENC_CS1_ROW
        write32(PMU_GRF_OS2, v & !(3 << 3) | (d & 3) << 4);
        let v = read32(PMU_GRF_OS3);
        write32(PMU_GRF_OS3, v & !(1 << 4) | ((d >> 2) & 1) << 4);
    }
    let cs0_high16bit_row = dram_detect_cs1_row(&cfg, 2);
    cfg.cs0_high16bit_row = cs0_high16bit_row;
    let cs1_high16bit_row = dram_detect_cs1_row(&cfg, 3);
    cfg.cs1_high16bit_row = cs1_high16bit_row;

    cfg.ddr_config = cfg.cs0_row;
    // NOTE: This changes a field I haven't yet defined.
    // cfg.xxx = cfg.cs1_row;

    println!("size calculation");
    let f = get_dram_size_factor(&cfg, 3, cfg.dram_type) as u64;
    println!("  size factor: {f:08x}");

    let v = read32(DDR_GRF_SPLIT_CON);
    println!("    split con: {v:08x}");
    // bit 8: AXI split bypass (1) or enable (0)
    // bits 0..7: split address
    let split_address = if (v >> 8) & 1 == 0 { v & 0xff } else { 0 } as u64;

    let size = if cfg.row_3_4 == 0 {
        if split_address != 0 {
            split_address * 0x800000 + (f / 2)
        } else {
            f
        }
    } else {
        (f * 3) / 4
    };

    let dram_size_mb = size >> 20;
    // FIXME: I get 4096, but should be 2048
    println!("  {dram_size_mb} MB ({size} bytes)");

    ddr_set_rate_for_fsp(&cfg);

    if false {
        println!("DRAM test");
        dram_test();
    }

    todo!("draw the rest of the owl 🦉🖌️");
}
