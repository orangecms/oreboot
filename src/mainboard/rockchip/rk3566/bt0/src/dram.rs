use util::mmio::{read32, write32};

use crate::arm::udelay;
use crate::i2c::{i2c_init, i2c_read};
use crate::mem_map::{
    CRU_NS_BASE, DDR_GRF_BASE, PMU_GRF_BASE, SRAM_BASE, SYS_SGRF_BASE, UPCTL2_BASE,
};

// SGRF: security subsystem (?)
// https://www.kernel.org/doc/Documentation/devicetree/bindings/soc/rockchip/grf.txt
// https://www.rockchip.fr/Rockchip%20RK3288%20TRM%20V1.0%20Part%201-System%20and%20System%20Control.pdf
const SYS_SGRF_0200: usize = SYS_SGRF_BASE + 0x0200;
const SYS_SGRF_0204: usize = SYS_SGRF_BASE + 0x0204;
const SYS_SGRF_0208: usize = SYS_SGRF_BASE + 0x0208;

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

fn cru_ns_xxx(v: u32) {
    let b = CRU_NS_BASE;
    let vb = v / 1000000;

    let m1 = match vb {
        ..101 => 6,
        ..201 => 4,
        ..800 => 2,
        _ => 1,
    };
    let m2 = match vb {
        ..151 => 6,
        ..800 => 4,
        _ => 2,
    };
    let m3 = match v {
        ..528000001 => 0x30000,
        _ => 0x30001,
    };

    write32(CRU_NS_BASE + 0x00c0, 0x000c_0000);
    write32(CRU_NS_BASE + 0x0128, 0x2000_2000);
    write32(
        CRU_NS_BASE + 0x0020,
        0x7fff_0000 | (m2 << 12) | m1 * m2 * vb / 24,
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

fn ddr_xxx() {
    println!("ddr_xxx");
    // ddr_xxx
    write32(DDR_GRF_BASE, 0x20000);
    // TODO: this really comes from a struct
    let v = 0x144; // 324
    cru_ns_xxx((v * 1000000) / 2);
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
