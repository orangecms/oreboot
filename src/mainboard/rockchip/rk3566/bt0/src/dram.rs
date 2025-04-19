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

struct XParam {
    v1: u32,
    v2: u32,
}

const XX_PARAMS_0: [XParam; 4] = [
    XParam {
        v1: 0x00,
        v2: 0x0000_1FA7,
    }, //
    XParam {
        v1: 0x08,
        v2: 0x0000_0000,
    }, //
    XParam {
        v1: 0x0c,
        v2: 0x0500_0000,
    }, //
    XParam {
        v1: 0x10,
        v2: 0x0500_0000,
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

    let v = read32(DDR_PHY_BASE + 0x00d0);

    // ^ 0xffffffff means inversion
    let m = v & (((7 << m3 | 1 << m4) ^ 0xffffffff) | m2 << m3 | m1 << m4);

    write32(DDR_PHY_BASE + 0x00d0, m);
}

fn ddr_xxx() {
    // NOTE: first round only, make parameter
    let vx = 0x3;

    println!("ddr_xxx");
    // ddr_xxx
    write32(DDR_GRF_BASE, 0x20000);
    // TODO: this really comes from a struct at 0x64
    let s_0064 = 0x144; // 324
    cru_ns_xxx((s_0064 * 1000000) / 2);

    write32(SYS_SGRF_BASE + 0x0014, 0x0b00_0b00);
    write32(CRU_S_0208, 0x0002_0002);
    write32(CRU_NS_BASE + 0x046c, 0x0180_0180);
    udelay(10);
    write32(SYS_SGRF_BASE + 0x0014, 0x0b00_0b00);
    write32(CRU_S_0208, 0x0002_0002);
    write32(CRU_NS_BASE + 0x046c, 0x0180_0100);

    if vx <= 8 {
        let m1 = if vx == 8 { 7 } else { vx };

        let x = (m1 & 0xc + 0x39 * 4) >> ((m1 & 0b11) << 3) & 0xff;
        let v = if x == 0xe4 {
            0xff80_e400
        } else {
            0xff80_0080 | (x << 8)
        };

        write32(DDR_GRF_BASE + 0x000c, v);
    }

    phy_smth(s_0064 * 1000000, 0);

    // TODO: other rounds have different params / sizes thereof
    for p in XX_PARAMS_0.iter() {
        let r = DDR_PHY_BASE + p.v1 as usize;
        let v = if p.v1 * 4 < 9 {
            let vx = read32(r);
            (vx & 0xc0ffffff) | p.v2
        } else {
            p.v2
        };
        write32(r, v)
    }

    println!("ddr_xxx done");
}

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

    ddr_xxx();
}
