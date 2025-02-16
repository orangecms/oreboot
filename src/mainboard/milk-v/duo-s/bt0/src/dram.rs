use oreboot_arch::riscv64::util::delay as opdelay;
use util::{read32, read64, write32};

// https://ddr-phy.org/
// DFI = DDR PHY Interface

use crate::mem_map::{
    CLK_GEN_PLL_CTRL_BASE, DDR_BIST_BASE, DDR_CFG_BASE, DDR_TOP_BASE, DRAM_BASE, PHYD_APB,
    PHYD_BASE, PHY_VERSION, TOP_BASE,
};
use crate::{axi_mon, ddr_bist, ddr_ctrl, ddr_phy, ddr_pll};

// TODO: All of this would be a build-time config.
const DO_BIST: bool = true;

const DBG_SHMOO: bool = false;
const DDR3: bool = true;
const DDR3_1866: bool = true;

const DFITMG0: usize = DDR_CFG_BASE + 0x0190;
const DFITMG1: usize = DDR_CFG_BASE + 0x0194;

// plat/cv181x/include/ddr/ddr_pkg_info.h
#[derive(Debug)]
#[repr(u32)]
pub enum DramType {
    Unknown = 0,
    NY4GbitDDR3 = 1,
    NY2GbitDDR3 = 2,
    ESMT1GbitDDR3 = 3,
    ESMTN25512MbitDDR2 = 4,
    ETRON1Gbit = 5,
    ESMT2GbitDDR3 = 6,
    PM2G = 7,
    PM1G = 8,
    ETRON512MbitDDR2 = 9,
    ESMTN251GbitDDR3 = 10,
}

impl From<u8> for DramType {
    fn from(value: u8) -> Self {
        match value {
            0 => panic!(),
            1 => Self::NY4GbitDDR3,
            2 => Self::NY2GbitDDR3,
            3 => Self::ESMT1GbitDDR3,
            4 => Self::ESMTN25512MbitDDR2,
            5 => Self::ETRON1Gbit,
            6 => Self::ESMT2GbitDDR3,
            7 => Self::PM2G,
            8 => Self::PM1G,
            9 => Self::ETRON512MbitDDR2,
            10 => Self::ESMTN251GbitDDR3,
            _ => panic!(),
        }
    }
}

#[derive(Eq, PartialEq)]
pub enum DdrType {
    Ddr2, // data rate = 1333
    Ddr3, // data rate = 1866
    Unknown,
}

fn get_ddr_type(dram_type: &DramType) -> DdrType {
    match dram_type {
        DramType::ESMTN25512MbitDDR2 | DramType::ETRON512MbitDDR2 => DdrType::Ddr2,
        DramType::NY4GbitDDR3
        | DramType::NY2GbitDDR3
        | DramType::ESMT1GbitDDR3
        | DramType::ETRON1Gbit
        | DramType::ESMT2GbitDDR3
        | DramType::PM2G
        | DramType::PM1G
        | DramType::ESMTN251GbitDDR3 => DdrType::Ddr3,
        DramType::Unknown => DdrType::Unknown,
    }
}

// plat/cv181x/ddr/ddr_sys.c
fn cvx16_setting_check() {
    println!("/ cvx16_setting_check");

    // NOTE: On SG2002 and SG2000 (Duo S), I get 20210920 - looking like year/month/day
    let phy_reg_version = read32(PHY_VERSION);
    println!("  phy_reg_version {phy_reg_version:08x}");

    // NOTE: Those were commented out in the vendor code as well.
    // write32(DFITMG0, 0x048a8305);
    // write32(DFITMG1, 0x00070202);

    let v = read32(DFITMG0);
    let dfi_tphy_wrlat = v & 0b11111;
    let dfi_tphy_wrdata = (v >> 8) & 0b111111;
    let dfi_t_rddata_en = (v >> 16) & 0b1111111;
    let dfi_t_ctrl_delay = (v >> 24) & 0b111111;
    let v = read32(DFITMG1);
    let dfi_t_wrdata_delay = (v >> 16) & 0b11111;

    println!("  dfi_t_ctrl_delay   {dfi_t_ctrl_delay}");
    println!("  dfi_t_rddata_en    {dfi_t_rddata_en}");
    println!("  dfi_tphy_wrlat     {dfi_tphy_wrlat}");
    println!("  dfi_tphy_wrdata    {dfi_tphy_wrdata}");
    println!("  dfi_t_wrdata_delay {dfi_t_wrdata_delay}");

    // TODO: other DRAM variants
    // 1866
    assert_eq!(dfi_tphy_wrlat, 0x5, "dfi_tphy_wrlat, wanted 0x5");
    assert_eq!(dfi_tphy_wrdata, 0x3, "dfi_tphy_wrdata, wanted 0x3");
    assert_eq!(dfi_t_rddata_en, 0xa, "dfi_t_rddata_en, wanted 0xa");
    assert_eq!(dfi_t_wrdata_delay, 0x7, "dfi_t_wrdata_delay, wanted 0x7");
    println!("\\ cvx16_setting_check finish");
}

// plat/cv181x/ddr/cvx16_pinmux.c
pub fn cvx16_pinmux(dram_type: &DramType) {
    println!("/ cvx16_pinmux start");
    /*
    // PHYA pin mux registers / params
    00
    swap_ca0    [4:     0]
    swap_ca1    [12:    8]
    swap_ca2    [20:   16]
    swap_ca3    [28:   24]
    04
    swap_ca4    [4:     0]
    swap_ca5    [12:    8]
    swap_ca6    [20:   16]
    swap_ca7    [28:   24]
    08
    swap_ca8    [4:     0]
    swap_ca9    [12:    8]
    swap_ca10   [20:   16]
    swap_ca11   [28:   24]
    0c
    swap_ca12   [4:     0]
    swap_ca13   [12:    8]
    swap_ca14   [20:   16]
    swap_ca15   [28:   24]
    10
    swap_ca16   [4:     0]
    swap_ca17   [12:    8]
    swap_ca18   [20:   16]
    swap_ca19   [28:   24]
    14
    swap_ca20   [4:     0]
    swap_ca21   [12:    8]
    swap_ca22   [20:   16]
    18
    swap_cke0   [0:0]
    swap_cs0    [4:4]
    1c
    data_byte_swap_slice0    [1:     0]
    data_byte_swap_slice1    [9:     8]
    20
    swap_byte0_dq0_mux    [3:     0]
    swap_byte0_dq1_mux    [7:     4]
    swap_byte0_dq2_mux    [11:    8]
    swap_byte0_dq3_mux    [15:   12]
    swap_byte0_dq4_mux    [19:   16]
    swap_byte0_dq5_mux    [23:   20]
    swap_byte0_dq6_mux    [27:   24]
    swap_byte0_dq7_mux    [31:   28]
    24
    swap_byte0_dm_mux     [3:     0]
    28
    swap_byte1_dq0_mux    [3:     0]
    swap_byte1_dq1_mux    [7:     4]
    swap_byte1_dq2_mux    [11:    8]
    swap_byte1_dq3_mux    [15:   12]
    swap_byte1_dq4_mux    [19:   16]
    swap_byte1_dq5_mux    [23:   20]
    swap_byte1_dq6_mux    [27:   24]
    swap_byte1_dq7_mux    [31:   28]
    2c
    swap_byte1_dm_mux     [3:     0]
    */
    match dram_type {
        // Duo S
        DramType::NY4GbitDDR3 => {
            println!("pin mux for NY 4Gbit DDR3");
            write32(0x0000 + PHYD_BASE, 0x12141013);
            write32(0x0004 + PHYD_BASE, 0x0C041503);
            write32(0x0008 + PHYD_BASE, 0x06050001);
            write32(0x000C + PHYD_BASE, 0x08070B02);
            write32(0x0010 + PHYD_BASE, 0x0A0F0E09);
            write32(0x0014 + PHYD_BASE, 0x0016110D);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x02136574);
            write32(0x0024 + PHYD_BASE, 0x00000008);
            write32(0x0028 + PHYD_BASE, 0x76512308);
            write32(0x002C + PHYD_BASE, 0x00000004);
        }
        // Duo 256, LicheeRV Nano
        DramType::NY2GbitDDR3 => {
            println!("pin mux for NY 2Gbit DDR3");
            write32(0x0000 + PHYD_BASE, 0x08070D09);
            write32(0x0004 + PHYD_BASE, 0x0605020B);
            write32(0x0008 + PHYD_BASE, 0x14040100);
            write32(0x000C + PHYD_BASE, 0x15030E0C);
            write32(0x0010 + PHYD_BASE, 0x0A0F1213);
            write32(0x0014 + PHYD_BASE, 0x00111016);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x82135764);
            write32(0x0024 + PHYD_BASE, 0x00000000);
            write32(0x0028 + PHYD_BASE, 0x67513028);
            write32(0x002C + PHYD_BASE, 0x00000004);
        }
        // also used in vendor code for DDR3_1G
        DramType::ESMT1GbitDDR3 => {
            write32(0x0000 + PHYD_BASE, 0x08070B09);
            write32(0x0004 + PHYD_BASE, 0x05000206);
            write32(0x0008 + PHYD_BASE, 0x0C04010D);
            write32(0x000C + PHYD_BASE, 0x15030A14);
            write32(0x0010 + PHYD_BASE, 0x10111213);
            write32(0x0014 + PHYD_BASE, 0x000F160E);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x31756024);
            write32(0x0024 + PHYD_BASE, 0x00000008);
            write32(0x0028 + PHYD_BASE, 0x26473518);
            write32(0x002C + PHYD_BASE, 0x00000000);
        }
        DramType::ESMTN25512MbitDDR2 => {
            write32(0x0000 + PHYD_BASE, 0x0C06080B);
            write32(0x0004 + PHYD_BASE, 0x070D0904);
            write32(0x0008 + PHYD_BASE, 0x00010502);
            write32(0x000C + PHYD_BASE, 0x110A0E03);
            write32(0x0010 + PHYD_BASE, 0x0F141610);
            write32(0x0014 + PHYD_BASE, 0x00151312);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x71840532);
            write32(0x0024 + PHYD_BASE, 0x00000006);
            write32(0x0028 + PHYD_BASE, 0x76103425);
            write32(0x002C + PHYD_BASE, 0x00000008);
        }
        DramType::ESMT2GbitDDR3 => {
            write32(0x0000 + PHYD_BASE, 0x080B0D06);
            write32(0x0004 + PHYD_BASE, 0x09010407);
            write32(0x0008 + PHYD_BASE, 0x1405020C);
            write32(0x000C + PHYD_BASE, 0x15000E03);
            write32(0x0010 + PHYD_BASE, 0x0A0F1213);
            write32(0x0014 + PHYD_BASE, 0x00111016);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x82135764);
            write32(0x0024 + PHYD_BASE, 0x00000000);
            write32(0x0028 + PHYD_BASE, 0x67513208);
            write32(0x002C + PHYD_BASE, 0x00000004);
        }
        DramType::ETRON1Gbit => {
            write32(0x0000 + PHYD_BASE, 0x0B060908);
            write32(0x0004 + PHYD_BASE, 0x02000107);
            write32(0x0008 + PHYD_BASE, 0x0C05040D);
            write32(0x000C + PHYD_BASE, 0x13141503);
            write32(0x0010 + PHYD_BASE, 0x160A1112);
            write32(0x0014 + PHYD_BASE, 0x000F100E);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x28137564);
            write32(0x0024 + PHYD_BASE, 0x00000000);
            write32(0x0028 + PHYD_BASE, 0x76158320);
            write32(0x002C + PHYD_BASE, 0x00000004);
        }
        DramType::ESMTN251GbitDDR3 => {
            write32(0x0000 + PHYD_BASE, 0x08060B09);
            write32(0x0004 + PHYD_BASE, 0x02040701);
            write32(0x0008 + PHYD_BASE, 0x0C00050D);
            write32(0x000C + PHYD_BASE, 0x13150314);
            write32(0x0010 + PHYD_BASE, 0x10111216);
            write32(0x0014 + PHYD_BASE, 0x000F0A0E);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x82135674);
            write32(0x0024 + PHYD_BASE, 0x00000000);
            write32(0x0028 + PHYD_BASE, 0x76153280);
            write32(0x002C + PHYD_BASE, 0x00000004);
        }
        DramType::ETRON512MbitDDR2 => {
            write32(0x0000 + PHYD_BASE, 0x070B090C);
            write32(0x0004 + PHYD_BASE, 0x04050608);
            write32(0x0008 + PHYD_BASE, 0x0E02030D);
            write32(0x000C + PHYD_BASE, 0x110A0100);
            write32(0x0010 + PHYD_BASE, 0x0F131614);
            write32(0x0014 + PHYD_BASE, 0x00151012);
            write32(0x0018 + PHYD_BASE, 0x00000000);
            write32(0x001C + PHYD_BASE, 0x00000100);
            write32(0x0020 + PHYD_BASE, 0x86014532);
            write32(0x0024 + PHYD_BASE, 0x00000007);
            write32(0x0028 + PHYD_BASE, 0x76012345);
            write32(0x002C + PHYD_BASE, 0x00000008);
        }
        DramType::Unknown | _ => {
            println!("  DRAM vendor unknown");
        }
    }
    // The following are from within ifdefs in the vendor code.
    // Many of them were actually duplicates, omitted/deduped here.
    const DDR2_512: bool = false;
    const DDR2_PINMUX: bool = false;
    const DDR3_PINMUX: bool = false;
    const DDR3_DBG: bool = false;
    if DDR2_512 {
        println!("pin mux X16 mode DDR2 512 setting");
        write32(0x0000 + PHYD_BASE, 0x0C06080B);
        write32(0x0004 + PHYD_BASE, 0x090D0204);
        write32(0x0008 + PHYD_BASE, 0x01050700);
        write32(0x000C + PHYD_BASE, 0x160A0E03);
        write32(0x0010 + PHYD_BASE, 0x0F141110);
        write32(0x0014 + PHYD_BASE, 0x00151312);
        write32(0x0018 + PHYD_BASE, 0x00000000);
        write32(0x001C + PHYD_BASE, 0x00000100);
        write32(0x0020 + PHYD_BASE, 0x60851243);
        write32(0x0024 + PHYD_BASE, 0x00000007);
        write32(0x0028 + PHYD_BASE, 0x67012354);
        write32(0x002C + PHYD_BASE, 0x00000008);
    }
    if DDR2_PINMUX || DDR3_PINMUX {
        println!("pin mux X16 mode DDR3 6mil setting");
        write32(0x0000 + PHYD_BASE, 0x020E0D00);
        write32(0x0004 + PHYD_BASE, 0x07090806);
        write32(0x0008 + PHYD_BASE, 0x0C05010B);
        write32(0x000C + PHYD_BASE, 0x12141503);
        write32(0x0010 + PHYD_BASE, 0x100A0413);
        write32(0x0014 + PHYD_BASE, 0x00160F11);
        write32(0x0018 + PHYD_BASE, 0x00000000);
        write32(0x001C + PHYD_BASE, 0x00000001);
        write32(0x0020 + PHYD_BASE, 0x40613578);
        write32(0x0024 + PHYD_BASE, 0x00000002);
        write32(0x0028 + PHYD_BASE, 0x03582467);
        write32(0x002C + PHYD_BASE, 0x00000001);
    }
    if DDR3_DBG {
        println!("pin mux X16 mode DDR3 debug setting");
        write32(0x0000 + PHYD_BASE, 0x0002080E);
        write32(0x0004 + PHYD_BASE, 0x04060D01);
        write32(0x0008 + PHYD_BASE, 0x090C030B);
        write32(0x000C + PHYD_BASE, 0x05071412);
        write32(0x0010 + PHYD_BASE, 0x0A151013);
        write32(0x0014 + PHYD_BASE, 0x0016110F);
        write32(0x0018 + PHYD_BASE, 0x00000000);
        write32(0x001C + PHYD_BASE, 0x00000100);
        write32(0x0020 + PHYD_BASE, 0x30587246);
        write32(0x0024 + PHYD_BASE, 0x00000001);
        write32(0x0028 + PHYD_BASE, 0x26417538);
        write32(0x002C + PHYD_BASE, 0x00000000);
    }
    println!("\\ cvx16_pinmux finish");
}

// This is a full duplicate in the vendor code:
// plat/cv181x/ddr/ddr_config/ddr_auto_x16/ddr_patch_regs.c
// plat/cv181x/ddr/ddr_config/ddr3_1866_x16/ddr_patch_regs.c
fn ddr_patch_set() {
    println!("/ ddr_patch_set start");
    if false {
        // tune damp
        write32(PHYD_BASE + 0x0150, 0x00000005);
        // CSB & CA driving
        write32(PHYD_BASE + 0x097c, 0x08080404);
        // CLK driving
        write32(PHYD_BASE + 0x0980, 0x08080808);
    }

    if false {
        if DDR3_1866 {
            // DQ driving // BYTE0
            write32(PHYD_BASE + 0x0a38, 0x00000606);
            // DQS driving // BYTE0
            write32(PHYD_BASE + 0x0a3c, 0x06060606);
            // DQ driving // BYTE1
            write32(PHYD_BASE + 0x0a78, 0x00000606);
            // DQS driving // BYTE1
            write32(PHYD_BASE + 0x0a7c, 0x06060606);
        } else {
            // DQ driving // BYTE0
            write32(PHYD_BASE + 0x0a38, 0x00000808);
            // DQS driving // BYTE0
            write32(PHYD_BASE + 0x0a3c, 0x04040404);
            // DQ driving // BYTE1
            write32(PHYD_BASE + 0x0a78, 0x00000808);
            // DQS driving // BYTE1
            write32(PHYD_BASE + 0x0a7c, 0x04040404);
        }

        // trigger level
        // BYTE0
        write32(PHYD_BASE + 0x0b24, 0x00100010);
        // BYTE1
        write32(PHYD_BASE + 0x0b54, 0x00100010);

        // APHY TX VREFDQ rangex2 [1]
        // VREF DQ
        write32(PHYD_BASE + 0x0410, 0x00120002);
        //APHY TX VREFCA rangex2 [1]
        // VREF CA
        write32(PHYD_BASE + 0x0414, 0x00100002);

        // tx dline code
        //  BYTE0 DQ
        let dq0 = 0x08000a00;
        let v = if DDR3_1866 { 0x06430643 } else { 0x06430644 };
        for r in (0x00..0x10).step_by(4) {
            write32(dq0 + r, v);
        }
        let v = if DDR3_1866 { 0x00000643 } else { 0x00000644 };
        write32(dq0 + 0x10, v);
        let v = if DDR3_1866 { 0x0a7e007e } else { 0x0d000000 };
        write32(dq0 + 0x14, v);
        //  BYTE1 DQ
        let dq1 = 0x08000a40;
        let v = if DDR3_1866 { 0x06430648 } else { 0x06430644 };
        for r in (0x00..0x10).step_by(4) {
            write32(dq1 + r, v);
        }
        let v = if DDR3_1866 { 0x00000648 } else { 0x00000644 };
        write32(dq1 + 0x10, v);
        let v = if DDR3_1866 { 0x0a7e007e } else { 0x0d000000 };
        write32(dq1 + 0x14, v);

        //APHY RX TRIG rangex2[18] & disable lsmode[0]
        //f0_param_phya_reg_rx_byte0_en_lsmode[0]
        //f0_param_phya_reg_byte0_en_rec_vol_mode[12]
        //f0_param_phya_reg_rx_byte0_force_en_lvstl_odt[16]
        //f0_param_phya_reg_rx_byte0_sel_dqs_rec_vref_mode[8]
        //param_phya_reg_rx_byte0_en_trig_lvl_rangex2[18]
        // BYTE0 [0]
        write32(PHYD_BASE + 0x0500, 0x00041001);
        //f0_param_phya_reg_rx_byte1_en_lsmode[0]
        //f0_param_phya_reg_byte1_en_rec_vol_mode[12]
        //f0_param_phya_reg_rx_byte0_force_en_lvstl_odt[16]
        //f0_param_phya_reg_rx_byte0_sel_dqs_rec_vref_mode[8]
        //param_phya_reg_rx_byte0_en_trig_lvl_rangex2[18]
        // BYTE1 [0]
        write32(PHYD_BASE + 0x0540, 0x00041001);

        ////////  FOR U02 ///////
        /////////// U02 enable DQS voltage mode receiver
        // f0_param_phya_reg_tx_byte0_en_tx_de_dqs[20]
        write32(PHYD_BASE + 0x0504, 0x00100000);
        // f0_param_phya_reg_tx_byte1_en_tx_de_dqs[20]
        write32(PHYD_BASE + 0x0544, 0x00100000);
        /////////// U02 enable MASK voltage mode receiver
        // param_phya_reg_rx_sel_dqs_wo_pream_mode[2]
        write32(PHYD_BASE + 0x0138, 0x00000014);
    }

    // BYTE0 RX DQ deskew
    let v = if DDR3_1866 { 0x00020402 } else { 0x02000202 };
    write32(PHYD_BASE + 0x0b00, v);
    let v = if DDR3_1866 { 0x05020401 } else { 0x00020000 };
    write32(PHYD_BASE + 0x0b04, v);
    // BYTE0  DQ8 deskew [6:0] neg DQS  [15:8]  ;  pos DQS  [23:16]
    let v = if DDR3_1866 { 0x00313902 } else { 0x002d3202 };
    write32(PHYD_BASE + 0x0b08, v);

    // BYTE1 RX DQ deskew
    let v = if DDR3_1866 { 0x06000100 } else { 0x04020603 };
    write32(PHYD_BASE + 0x0b30, v);
    let v = if DDR3_1866 { 0x02010303 } else { 0x00060203 };
    write32(PHYD_BASE + 0x0b34, v);
    // BYTE1  DQ8 deskew [6:0] neg DQS  [15:8]  ;  pos DQS  [23:16]
    let v = if DDR3_1866 { 0x00323900 } else { 0x00313503 };
    write32(PHYD_BASE + 0x0b38, v);

    if false {
        //Read gate TX dline + shift
        let v = if DDR3_1866 { 0x00000a14 } else { 0x0000081e };
        // BYTE0
        write32(PHYD_BASE + 0x0b0c, v);
        // BYTE1
        write32(PHYD_BASE + 0x0b3c, v);

        // CKE dline + shift CKE0 [6:0]+[13:8] ; CKE1 [22:16]+[29:24]
        write32(PHYD_BASE + 0x0930, 0x04000400);
        // CSB dline + shift CSB0 [6:0]+[13:8] ; CSB1 [22:16]+[29:24]
        write32(PHYD_BASE + 0x0934, 0x04000400);
    }

    println!("\\ ddr_patch_set finish");
}

// plat/cv181x/ddr/ddr_sys.c
fn cvx16_en_rec_vol_mode(ddr_type: &DdrType) {
    println!("/ cvx16_en_rec_vol_mode start");
    if *ddr_type == DdrType::Ddr2 {
        write32(PHYD_BASE + 0x0500, 0x00001001);
        write32(PHYD_BASE + 0x0540, 0x00001001);
    }
    println!("\\ cvx16_en_rec_vol_mode finish");
}

fn cvx16_dram_cap_check(size: u32) {
    // TODO
}

// fsbl plat/cv181x/ddr/ddr_sys_bring_up.c ddr_sys_bring_up
pub fn init(ddr_data_rate: usize, dram_type: &DramType) {
    let ddr_type = &get_ddr_type(dram_type);
    let (reg_set, reg_span, reg_step) = ddr_pll::get_pll_settings(ddr_data_rate);
    // NOTE: cvx16_pll_init is called from within pll_init in the vendor code
    ddr_pll::cvx16_pll_init(reg_set, reg_span, reg_step, dram_type);
    ddr_ctrl::init();
    // cvx16_ctrlupd_short();

    // release ddrc soft reset
    println!("Release DDR controller from reset");
    ddr_ctrl::reset();

    // set axi QOS
    // M1 = 0xA (VIP realtime)
    // M2 = 0x8 (VIP offline)
    // M3 = 0x7 (CPU)
    // M4 = 0x0 (TPU)
    // M5 = 0x9 (Video codec)
    // M6 = 0x2 (high speed peri)
    write32(TOP_BASE + 0x01D8, 0x007788aa);
    write32(TOP_BASE + 0x01DC, 0x00002299);

    ddr_phy::phy_init();
    cvx16_setting_check();
    cvx16_pinmux(dram_type);
    ddr_patch_set();
    cvx16_en_rec_vol_mode(ddr_type);
    ddr_pll::cvx16_set_dfi_init_start();

    ddr_pll::cvx16_ddr_phy_power_on_seq1();

    ddr_pll::cvx16_polling_dfi_init_start();
    ddr_pll::cvx16_int_isr_08();
    // NOTE: Vendor code calls cvx16_chg_pll_freq() within cvx16_ddr_phy_power_on_seq2().
    ddr_pll::change_pll_freq(reg_set, reg_span, reg_step);
    // NOTE: Vendor code does both cvx16_ddr_phy_power_on_seq2() and
    //  cvx16_set_dfi_init_complete() at the end of cvx16_int_isr_08()
    ddr_pll::cvx16_ddr_phy_power_on_seq2();
    ddr_pll::cvx16_set_dfi_init_complete();

    // ddr_pll::change_pll_freq(reg_set, reg_span, reg_step);
    ddr_pll::cvx16_ddr_phy_power_on_seq3();
    ddr_pll::cvx16_wait_for_dfi_init_complete();
    ddr_ctrl::cvx16_polling_synp_normal_mode();

    if DO_BIST {
        ddr_bist::cvx16_bist_wr_prbs_init();
        if ddr_bist::bist().is_err() {
            panic!("BIST fail");
        }
    }

    ddr_ctrl::low_patch();
    println!("ctrl_low_patch finish");

    // CHECK
    if *ddr_type != DdrType::Ddr2 {
        ddr_bist::cvx16_wrlvl_req(ddr_type);
        println!("cvx16_wrlvl_req finish");
    }

    if DO_BIST {
        ddr_bist::cvx16_bist_wr_prbs_init();
        if ddr_bist::bist().is_err() {
            panic!("BIST fail");
        }
    }

    ddr_bist::cvx16_rdglvl_req(ddr_type);
    println!("cvx16_rdglvl_req finish");

    if DO_BIST {
        ddr_bist::cvx16_bist_wr_prbs_init();
        if ddr_bist::bist().is_err() {
            panic!("BIST fail");
        }
    }

    //ERROR("AXI mon setting for latency histogram.\n");
    //axi_mon_set_lat_bin_size(0x5);

    if DBG_SHMOO {
        /*
        const DPHY_WDQ: usize = PHYD_BASE + 0x0190;
        // dfi_wdq_lvl_vref_start [6:0]
        // dfi_wdq_lvl_vref_end [14:8]
        // dfi_wdq_lvl_vref_step [19:16]
        write32(DPHY_WDQ, 0x00021E02);
        // pi_wdq_lvl_dly_step[23:20]
        const PI_WDQ_LVL_DELAY: usize = PHYD_BASE + 0x00a4;
        write32(PI_WDQ_LVL_DELAY, 0x01220504);
        // write start   shift = 5  /  dline = 78
        let r = PHYD_BASE + 0x00a0;
        write32(r, 0x0d400578);
        // write
        println!("wdqlvl_M1_ALL_DQ_DM\n");
        // cvx16_wdqlvl_req(data_mode, lvl_mode)
        println!("cvx16_wdqlvl_sw_req dq/dm");
        // console_getc();
        cvx16_wdqlvl_sw_req(1, 2);
        // cvx16_wdqlvl_status();
        println!("cvx16_wdqlvl_req dq/dm finish");

        println!("cvx16_wdqlvl_sw_req dq");
        // console_getc();
        cvx16_wdqlvl_sw_req(1, 1);
        // cvx16_wdqlvl_status();
        println!("cvx16_wdqlvl_req dq finish");

        println!("cvx16_wdqlvl_sw_req dm");
        // console_getc();
        cvx16_wdqlvl_sw_req(1, 0);
        // cvx16_wdqlvl_status();
        */
        println!("cvx16_wdqlvl_req dm finish");
    } else {
        println!(" wdqlvl_M1_ALL_DQ_DM");
        // sso_8x1_c(5, 15, 0, 1, &sram_sp);
        // mode = write, input int fmin = 5, input int fmax = 15,
        // input int sram_st = 0, output int sram_sp

        ddr_bist::cvx16_wdqlvl_req(1, ddr_bist::LvlMode::WdqAndWdmLvl);
        println!("  cvx16_wdqlvl_req dq/dm finish");
        ddr_bist::cvx16_wdqlvl_req(1, ddr_bist::LvlMode::WdqLvl);
        println!("  cvx16_wdqlvl_req dq finish");
        ddr_bist::cvx16_wdqlvl_req(1, ddr_bist::LvlMode::WdmLvl);
        println!("  cvx16_wdqlvl_req dm finish");
        if DO_BIST {
            ddr_bist::cvx16_bist_wr_prbs_init();
            if ddr_bist::bist().is_err() {
                panic!("BIST fail");
            }
        }
    }

    /*
    if DBG_SHMOO {
        // param_phyd_pirdlvl_dly_step [3:0]
        // param_phyd_pirdlvl_vref_step [11:8]
        write32(PHYD_BASE + 0x0088, 0x0A010212);

        //read
        println!("cvx16_rdlvl_req start");
        // console_getc();
        println!("SW mode 1, sram write/read continuous goto");
        cvx16_rdlvl_sw_req(1);
        // cvx16_rdlvl_status();
        println!("cvx16_rdlvl_req finish");
    } else {
        // cvx16_rdlvl_req
        // mode = 'h0  : MPR mode, DDR3 only.
        // mode = 'h1  : sram write/read continuous goto
        // mode = 'h2  : multi- bist write/read
        // mode = 'h10 : with Error enject,  multi- bist write/read
        // mode = 'h12 : with Error enject,  multi- bist write/read
        let v = read32(PHYD_BASE + 0x008c);
        // param_phyd_pirdlvl_capture_cnt
        let v = (v & (0b1111 << 4)) | 0x1;
        write32(PHYD_BASE + 0x008c + PHYD_BASE, v);

        println!("mode multi- bist write/read");
        // mode multi- PRBS bist write/read
        // cvx16_rdlvl_req(2);
        // mode multi- SRAM bist write/read
        cvx16_rdlvl_req(1);
        println!("cvx16_rdlvl_req finish");

        if DO_BIST {
            cvx16_bist_wr_prbs_init();
            if let Err(()) = bist() {
                panic!("ERROR bist_fail");
            }
        }
    }
    */

    /*
    if DBG_SHMOO_CA {
        //CA training
        NOTICE("\n===== calvl_req =====\n"); console_getc();
        // sso_8x1_c(5, 15, sram_sp, 1, &sram_sp_1);
        calvl_req(cap);
    }

    if DBG_SHMOO_CS {
        //CS training
        NOTICE("\n===== cslvl_req =====\n"); console_getc();
        // sso_8x1_c(5, 15, sram_sp, 1, &sram_sp_1);
        cslvl_req(cap);
    }
    */

    if DBG_SHMOO {
        /*
        cvx16_dll_cal_status();
        cvx16_wrlvl_status();
        cvx16_rdglvl_status();
        cvx16_rdlvl_status();
        cvx16_wdqlvl_status();
        */
    }

    ddr_ctrl::high_patch();

    let dram_cap_in_mbyte = ddr_bist::detect_dram_size(ddr_type);
    println!("dram_cap_in_mbyte: {dram_cap_in_mbyte}");
    ddr_ctrl::update_by_dram_size(dram_cap_in_mbyte);
    println!("ctrl_init_update_by_dram_size finish");
    println!("dram_cap_in_mbyte: {dram_cap_in_mbyte}");
    cvx16_dram_cap_check(dram_cap_in_mbyte);
    println!("cvx16_dram_cap_check finish");

    // clk_gating_enable
    ddr_pll::cvx16_clk_gating_enable();
    println!("cvx16_clk_gating_enable finish");

    if DO_BIST {
        ddr_bist::cvx16_bist_wr_prbs_init();
        if ddr_bist::bist().is_err() {
            println!("ERROR prbs bist_fail");
            panic!("DDR BIST FAIL");
        }
        /*
        cvx16_bist_wr_sram_init();
        if let Err(()) = bist() {
            println!("ERROR sram bist_fail");
            panic!("ERROR bist_fail");
        }
        */
        println!("DDR BIST PASS");
    }

    /*
    #ifdef FULL_MEM_BIST
        //full memory
        // sso_8x1_c(5, 15, 0, 1, &sram_sp);
        // sso_8x1_c(5, 15, sram_sp, 1, &sram_sp);

        NOTICE("====FULL_MEM_BIST====\n");
        bist_result = bist_all_dram(0, cap);
        if (bist_result == 0) {
            NOTICE("bist_all_dram(prbs): ERROR bist_fail\n");
        } else {
            NOTICE("bist_all_dram(prbs): BIST PASS\n");
        }

        bist_result = bist_all_dram(1, cap);
        if (bist_result == 0) {
            NOTICE("bist_all_dram(sram): ERROR bist_fail\n");
        } else {
            NOTICE("bist_all_dram(sram): BIST PASS\n");
        }

        bist_result = bist_all_dram(2, cap);
        if (bist_result == 0) {
            NOTICE("bist_all_dram(01): ERROR bist_fail\n");
        } else {
            NOTICE("bist_all_dram(01): BIST PASS\n");
        }

        NOTICE("===== BIST END ======\n");
    #endif //FULL_MEM_BIST
    */

    /*
    if FULL_MEM_BIST_FOREVER {
        println!("Start DRAM stress test");
        bist_all_dram_forever(cap);
    }
    */

    // ERROR("AXI mon setting for latency histogram.\n");
    axi_mon::axi_mon_latency_setting(0x5);

    // ERROR("AXI mon 0 register dump before start.\n");
    // dump_axi_mon_reg(AXIMON_M1_WRITE);
    // ERROR("AXI mon 1 register dump before start.\n");
    // dump_axi_mon_reg(AXIMON_M1_READ);

    axi_mon::axi_mon_start_all();
}
