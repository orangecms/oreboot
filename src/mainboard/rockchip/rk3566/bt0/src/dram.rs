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
const SYS_SGRF_0200: usize = SYS_SGRF_BASE + 0x0200;
const SYS_SGRF_0204: usize = SYS_SGRF_BASE + 0x0204;

const CRU_S_0208: usize = CRU_S_BASE + 0x0208;

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
fn upctl2_pre_init() {
    let v0248 = read32(UPCTL2_BASE + 0x0248);
    println!("{v0248:08x}");
    let v024c = read32(UPCTL2_BASE + 0x024C);
    println!("{v024c:08x}");
    let v = (v0248 >> 13) & 0xf;
    println!("{v:x}");
}

const PMU_GRF_OS2: usize = PMU_GRF_BASE + 0x0208;

fn cru_ns_xxx(p: u32) {
    let b = CRU_NS_BASE;
    let vp = p / 1000000;

    let m1 = match vp {
        ..101 => 6,
        ..201 => 4,
        ..800 => 2,
        _ => 1,
    };
    let m2 = match vp {
        ..151 => 6,
        ..800 => 4,
        _ => 2,
    };
    let m3 = match p {
        ..528000001 => 0x30000,
        _ => 0x30001,
    };

    write32(CRU_NS_BASE + 0x00c0, 0x000c_0000);
    write32(CRU_NS_BASE + 0x0128, 0x2000_2000);
    write32(
        CRU_NS_BASE + 0x0020,
        0x7fff_0000 | (m2 << 12) | m1 * m2 * vp / 24,
    );

    write32(CRU_NS_BASE + 0x0024, 0x11ff_1001 | (m1 << 6));
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

fn phy_smth(p1: u32, p2: u32) {
    let vp1 = p1 / 1000000;
    //
    let (m1, m2) = match vp1 {
        ..51 => (1, 5),
        ..101 => (1, 4),
        ..201 => (1, 3),
        ..401 => (1, 2),
        ..801 => (1, 1),
        _ => (0, 0),
    };
    let m3 = (p2 * 8 + 4) & 0x1f;
    let m4 = (p2 * 8 + 3) & 0x1f;

    let v = read32(DDR_PHY_00D0);

    // ^ 0xffffffff means inversion
    let m = v & (((7 << m3 | 1 << m4) ^ 0xffffffff) | m2 << m3 | m1 << m4);

    write32(DDR_PHY_00D0, m);
}

const DDR_GRF_0000: usize = DDR_GRF_BASE + 0x0000;
const DDR_GRF_000C: usize = DDR_GRF_BASE + 0x000c;

fn get_funny_bits() -> (u32, u32) {
    let vtt = read32(DDR_GRF_000C);
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
fn upctl2_phy_smth(s_0064: u32, dram_type: u32, smth: bool) {
    let cfg = cfg_for_dram_type(dram_type);

    // Those really depend on s_0064 and cfg; shortcut taken here.
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
    // 16 iterations
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
    // 16 iterations
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
    let v1 = if CFG_0X20.p0 & 0xfff < s_0064 {
        CFG_0X20.p2 // 0x0
    } else {
        CFG_0X20.p3 // 0x2225_2525
    };
    // 0x94 (148)
    let v1 = (v1 >> 14) & 0x3ff;

    // p6 & 0xfff = 0x14d
    let v2 = if cfg.p6 & 0xfff < s_0064 {
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
        ..630 => (v2 - 329) / 6 | 0x40, // we should be here
        _ => 114,
    };

    let v1 = match v1 {
        ..150 => 0, // we should be here
        ..450 => (v1 - 150) / 6,
        ..630 => (v1 - 329) / 6 | 0x40,
        _ => 114,
    };

    write32(UPCTL2_0320, 0);

    let p4_sby_0x1000 = if smth { 0x1000 } else { 0 };
    let o_base = p4_sby_0x1000 * 2;
    let o1 = o_base + 0x00e8;
    let o2 = o_base + 0x00ec;

    let v = read32(UPCTL2_BASE + o1);
    write32(UPCTL2_BASE + o1, (v & 0xffff_0000) | v2);

    let v = read32(UPCTL2_BASE + o2);
    write32(UPCTL2_BASE + o2, (v & 0xffff_0000) | v1);

    poll_upctl2_x();

    let o3 = o_base + 0x00dc;
    let v = read32(UPCTL2_BASE + o3);
    let v = v & 0xfd99;

    let p4_byte3 = cfg.p4 >> 24;

    let v3 = if p4_byte3 == 0x22 { v | (1 << 1) } else { v };
    // TODO: if !smth ...

    write32(UPCTL2_0320, 0);

    let v = read32(UPCTL2_BASE + o3);
    write32(UPCTL2_BASE + o3, (v & 0xffff_0000) | v3);

    poll_upctl2_x();
}

fn poll_upctl2_x() {
    write32(UPCTL2_0320, 1);
    while read32(UPCTL2_0324) & 1 == 0 {}
}

fn ddr_xxx(enable_ecc: bool) {
    // TODO: These values come from structs at the offsets encoded in the
    // variable names. Should we make those structs or simple parameters?
    let s_0000 = 0x1;
    let s_0004 = 0xc;
    let s_0008 = 0x3;
    let s_000c = 0x1;
    let s_0010 = 0x0;
    let s_0014 = 0x0;
    let s_0018 = 0x10;
    let s_001c = 0x10;

    // this may encode the DRAM speed
    let s_0064 = 0x144;
    // apparently s_0068 encodes the DRAM type
    let dram_type = 0x3;
    let s_007c = XX_PARAMS_0_UPCTL2[0].value;

    println!("ddr_xxx");
    write32(DDR_GRF_0000, 0x20000);

    cru_ns_xxx((s_0064 * 1000000) / 2);

    write32(SYS_SGRF_BASE + 0x0014, 0x0b00_0b00);
    write32(CRU_S_0208, 0x0002_0002);
    write32(CRU_NS_BASE + 0x046c, 0x0180_0180);
    udelay(10);
    write32(SYS_SGRF_BASE + 0x0014, 0x0b00_0b00);
    write32(CRU_S_0208, 0x0002_0002);
    write32(CRU_NS_BASE + 0x046c, 0x0180_0100);

    // TODO: What is the possible value range? This check may be unnecessary.
    if dram_type <= 8 {
        let m1 = if dram_type == 8 { 7 } else { dram_type };

        let x = (m1 & 0xc + 0x39 * 4) >> ((m1 & 0b11) << 3) & 0xff;
        let v = if x == 0xe4 {
            0xff80_e400
        } else {
            0xff80_0080 | (x << 8)
        };

        println!("DDR_GRF_000C: write {v:08x}");
        write32(DDR_GRF_000C, v);
    }

    phy_smth(s_0064 * 1000000, 0);

    // TODO: other rounds have different params / sizes thereof
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

    let v = read32(DDR_PHY_BASE) & 0xffffe0ff;
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
    // Each loop has 16 iterations
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

    write32(CRU_NS_BASE + 0x046c, 0x0180_0000);
    write32(SYS_SGRF_BASE + 0x0014, 0x0b00_0300);
    write32(CRU_S_0208, 0x0002_0000);
    write32(CRU_NS_BASE + 0x046c, 0x0180_0000);

    // NOTE: params list needs to be a param here as well
    upctl2_fill(&XX_PARAMS_0_UPCTL2, 0x005d, 0x000d);

    let v = read32(UPCTL2_0404);
    write32(UPCTL2_0404, v | 0x0001_0000);

    let v = read32(UPCTL2_0000);
    write32(UPCTL2_0000, v | 0x2000_0000);

    let v = read32(UPCTL2_0028);
    write32(UPCTL2_0028, v & 0xffff_fffc);

    upctl2_phy_smth(s_0064, dram_type, false);

    // 0xd
    let s_000c_0004 = s_000c + s_0004;

    // 3 * 0x20 | 0 | (-3) = 0xffff_fffc
    let vt = (s_0018 - 0xd) * 0x20 | (s_0000 - 1) * 0x100 | s_000c_0004 - 10;

    let vxx = if s_0008 == 3 { vt | 8 } else { vt };

    let idx = find_index(vxx);
    // TODO: logic

    write32(SYS_SGRF_BASE + 0x0014, 0x0b00_0000);
    write32(CRU_S_0208, 0x0002_0000);
    write32(CRU_NS_BASE + 0x046c, 0x0180_0000);

    // TODO: draw the rest of the owl 🦉🖌️

    println!("ddr_xxx done");
}

fn find_index(vxx: u32) -> usize {
    for (i, c) in DATA.iter().enumerate() {
        // NOTE: ^ is XOR
        if (c ^ vxx) & 0x1f == 0 && // asd
            (vxx & 0xe0) <= (c & 0xe0) && // asd
            (vxx & 0x100) <= (c & 0x100)
        {
            return i;
        }
    }
    10
}

const DATA: [u32; 9] = [
    0x00AA, 0x01A9, 0x018A, 0x016B, //
    0x014C, 0x005C, 0x0099, 0x009A, //
    0x007B,
];

const UPCTL2_0000: usize = UPCTL2_BASE + 0x0000;
const UPCTL2_0028: usize = UPCTL2_BASE + 0x0028;
const UPCTL2_0034: usize = UPCTL2_BASE + 0x0034;
const UPCTL2_0038: usize = UPCTL2_BASE + 0x0038;
const UPCTL2_0180: usize = UPCTL2_BASE + 0x0180;
const UPCTL2_0320: usize = UPCTL2_BASE + 0x0320;
const UPCTL2_0324: usize = UPCTL2_BASE + 0x0324;
const UPCTL2_0404: usize = UPCTL2_BASE + 0x0404;

fn upctl2_fill(reg_vals: &[RegVal], p1: u32, p2: u32) {
    fill_regs(UPCTL2_BASE, reg_vals);

    let v = read32(UPCTL2_0034);
    let m = 0xff00_ffe0;
    write32(UPCTL2_0034, v & m | ((p1 & 0xff) << 16) | p2 & 0x1f);

    let v = read32(UPCTL2_0038);
    let m = 0xf000_ffff;
    write32(UPCTL2_0038, v & m | 0x50000);

    let v = read32(UPCTL2_0180);
    write32(UPCTL2_0180, v | 0x80000000);
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

const DDR_PHY_0038: usize = DDR_PHY_BASE + 0x0038;
const DDR_PHY_0044: usize = DDR_PHY_BASE + 0x0044;
const DDR_PHY_008C: usize = DDR_PHY_BASE + 0x008c;
const DDR_PHY_0094: usize = DDR_PHY_BASE + 0x0094;
const DDR_PHY_00AC: usize = DDR_PHY_BASE + 0x00ac;
const DDR_PHY_00C0: usize = DDR_PHY_BASE + 0x00c0;
const DDR_PHY_00D0: usize = DDR_PHY_BASE + 0x00d0;
const DDR_PHY_00F0: usize = DDR_PHY_BASE + 0x00f0;
const DDR_PHY_00F4: usize = DDR_PHY_BASE + 0x00f4;
const DDR_PHY_00F8: usize = DDR_PHY_BASE + 0x00f8;
const DDR_PHY_0304: usize = DDR_PHY_BASE + 0x0304;

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

    let v0208 = read32(PMU_GRF_OS2);

    // Why is this being done here?
    crate::otp::otp_phy_init();

    if false {
        // we get 0 (reset value), would be non-zero for a second run...
        println!("PMU_GRF_OS2: {v0208:08x}");
        if v0208 != 0 {
            upctl2_pre_init();
        } else {
            println!("PMU_GRF_OS2 is 0, whoops");
        }
    }

    // TODO: only first round?
    let enable_ecc = true;
    ddr_xxx(enable_ecc);
}
