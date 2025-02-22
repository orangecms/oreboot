use oreboot_arch::riscv64::util::delay as opdelay;
use util::mmio::{read32, write32};

use crate::ddr_ctrl::POWER_CONTROL;
use crate::dram::DramType;
use crate::mem_map::{
    CLK_GEN_PLL_CTRL_BASE, DDR_CFG_BASE, DDR_TOP_BASE, PHYD_APB, PHYD_BASE, PHY_VERSION,
};

const DEBUG: bool = true;

// NOTE: SSC_EN is commented out in plat/cv18{0,1}x/ddr/ddr.mk
const SSC_EN: bool = false;
// NOTE: SSC_BYPASS is never set
const SSC_BYPASS: bool = false;

// TRM alpha p53
const PLL_G6_BASE: usize = CLK_GEN_PLL_CTRL_BASE + 0x0900;
const DPLL_SSC_SYN_CTRL: usize = PLL_G6_BASE + 0x0050;
const DPLL_SSC_SYN_SET: usize = PLL_G6_BASE + 0x0054;
const DPLL_SSC_SYN_SPAN: usize = PLL_G6_BASE + 0x0058;
const DPLL_SSC_SYN_STEP: usize = PLL_G6_BASE + 0x005C;

// PHYD APB
// aka TOP_REG_...
const PHYD_TX_BYTE: usize = PHYD_APB + 0x0000;
const RESETZ_DIV: usize = PHYD_APB + 0x0004;
const RESETZ_DQS: usize = PHYD_APB + 0x0008;
const DDR_PLL_TEST: usize = PHYD_APB + 0x0010;
const DDR_PLL: usize = PHYD_APB + 0x000c;
const DDR_PLL_MAS_RSTZ_DIV: u32 = 1 << 7;

const PHYD_TX_SEL: usize = PHYD_APB + 0x001c;

const TX_VREF_PD: usize = PHYD_APB + 0x0028;

const PHYA_CA_PD: usize = PHYD_APB + 0x0040;
const CLOCK_GATING_ENABLE: usize = PHYD_APB + 0x0044;
const PHYD_SPEED: usize = PHYD_APB + 0x004c;
const ZQ_240_OPTION: usize = PHYD_APB + 0x0054;
const GPO_SETTING: usize = PHYD_APB + 0x0058;

// PHYD
const PHYD_DLL_CTRL: usize = PHYD_BASE + 0x0040;
const PHYD_DLL_RX_START_CAL: u32 = 1 << 1;
const PHYD_DLL_TX_START_CAL: u32 = 1 << 17;

const PHYD_SHIFT_GATING_EN: usize = PHYD_BASE + 0x00f4;

const PHYD_TX_CA: usize = PHYD_BASE + 0x0130;
const PHYD_SEL_CKE: usize = PHYD_BASE + 0x0154;

// PHY Version (?)
const PHYD_DLL_STATUS: usize = PHY_VERSION + 0x0014;
const PHYD_DLL_STATUS_DONE: u32 = 1 << 16;

// DDR TOP
const DFI_CA_0: usize = DDR_TOP_BASE + 0x0000;
const DFI_CA_1: usize = DDR_TOP_BASE + 0x0004;
const DFI_CA_2: usize = DDR_TOP_BASE + 0x0008;
const DFI_CA_3: usize = DDR_TOP_BASE + 0x000c;

pub fn cvx16_dll_cal_status() {
    let v = read32(PHYD_BASE + 0x3014);
    let m = 0xff;
    let rx_dll_code = (v >> 8) & m;
    let tx_dll_code = (v >> 24) & m;
    if (!((rx_dll_code > 0x2b) && (rx_dll_code < 0x30))) {
        println!("ERROR! rx_dll_code dly_sel result fail, not 0x2b~0x30 {rx_dll_code:02x}");
    }
    if (!((tx_dll_code > 0x2b) && (tx_dll_code < 0x30))) {
        println!("ERROR! tx_dll_code dly_sel result fail, not 0x2b~0x30 {tx_dll_code:02x}");
    }
}

fn cvx16_dll_cal() {
    println!("/ cvx16_dll_cal start");
    let v = read32(PHYD_SPEED);
    let (en_pll_speed_chg, curr_pll_speed, next_pll_speed) = get_pll_speed_change(v);
    // stop calibration and update
    let v = read32(PHYD_DLL_CTRL);
    let v = v & !PHYD_DLL_RX_START_CAL & !PHYD_DLL_TX_START_CAL;
    write32(PHYD_DLL_CTRL, v);
    // only do calibration and update when high speed
    if (curr_pll_speed > 0) {
        let v = read32(PHYD_DLL_CTRL);
        let v = v | PHYD_DLL_RX_START_CAL | PHYD_DLL_TX_START_CAL;
        write32(PHYD_DLL_CTRL, v);
        while read32(PHYD_DLL_STATUS) & PHYD_DLL_STATUS_DONE == 0 {}
        println!("  DLL lock");
        // opdelay(1000);
        println!("  DLL UPD");
        if DEBUG {
            cvx16_dll_cal_status();
        }
    }
    println!("\\ cvx16_dll_cal finish");
}

// DFI = DDR PHY Interface
// https://www.synopsys.com/blogs/chip-design/mastering-ddr-phy-interoperability-dfi.html
// plat/cv181x/ddr/ddr_sys.c
pub fn cvx16_set_dfi_init_start() {
    println!("/ cvx16_set_dfi_init start");
    // synp setting
    // phy is ready for initial dfi_init_start request
    // set umctl2 to trigger dfi_init_start
    write32(DDR_CFG_BASE + 0x0320, 0x0);
    // dfi_init_start @ rddata[5];
    let v = read32(DDR_CFG_BASE + 0x01b0);
    write32(DDR_CFG_BASE + 0x01b0, v | (1 << 5));
    write32(DDR_CFG_BASE + 0x0320, 0x1);
    println!("\\ set_dfi_init_start finish");
}

pub fn cvx16_wait_for_dfi_init_complete() {
    println!("/ wait_for_dfi_init_complete start");
    while read32(DDR_CFG_BASE + 0x01bc) & 0x1 == 0 {}
    write32(DDR_CFG_BASE + 0x0320, 0x0);
    let v = read32(DDR_CFG_BASE + 0x01b0);
    let m = !(0b111111);
    write32(DDR_CFG_BASE + 0x01b0, (v & m) | 5);
    write32(DDR_CFG_BASE + 0x0320, 0x1);
    println!("\\ wait_for_dfi_init_complete finish");
}

pub fn cvx16_polling_dfi_init_start() {
    println!("/ first dfi_init_start");
    while read32(PHYD_BASE + 0x3028) & (1 << 8) == 0 {}
    println!("\\ cvx16_polling_dfi_init_start finish");
}

pub fn cvx16_set_dfi_init_complete() {
    println!("/ cvx16_set_dfi_init_complete start");
    opdelay(20000);
    // rddata[8] = 1;
    write32(PHYD_BASE + 0x0120, 0x00000010);
    println!("  set init_complete = 1 ...");
    // param_phyd_clkctrl_init_complete   <= int_regin[0];
    write32(PHYD_BASE + 0x0118, 0x1);
    println!("\\ cvx16_set_dfi_init_complete finish");
}

pub fn cvx16_dfi_ca_park_prbs(cap_enable: bool) {
    // param_phyd_sw_dfi_phyupd_req =1
    write32(PHYD_BASE + 0x0174, 0x1);
    // param_phyd_to_reg_dfi_phyupd_req  8   8
    // param_phyd_to_reg_dfi_phyupd_ack  9   9
    while (read32(PHYD_BASE + 0x3030) >> 8) & 0b11 != 0b11 {}

    // DDR3
    //   cfg_det_en = 0b1;
    //   cfg_cs_det_en = 0b1;
    //   cap_prbs_en = 0b1;
    //   cfg_cs_polarity = 0b1;
    //   cap_prbs_1t = 0b0;
    //   cfg_ca_reference = {0b0,0x0_ffff,0x7,0x0,0b1,0b0,0b1,0b1};
    //   cfg_cs_retain_cycle = 0b0000_0001;
    //   cfg_ca_retain_cycle = 0b0000_0000;
    //   cfg_ca_park_value = 0x3fff_ffff;

    let dfi_ca_park_misc = if cap_enable { 0x1B } else { 0 };
    write32(DFI_CA_0, dfi_ca_park_misc);
    println!("    dfi_ca_park_prbs enable = {cap_enable}");

    // dfi_ca_park_retain_cycle;
    write32(DFI_CA_1, 0x1);
    // dfi_ca_park_ca_ref
    write32(DFI_CA_2, 0x1ffffcb);
    // dfi_ca_park_ca_park
    write32(DFI_CA_3, 0x3fffffff);

    // param_phyd_sw_dfi_phyupd_req_clr =1
    write32(PHYD_BASE + 0x0174, 0x00000010);
}

pub fn cvx16_ddr_phy_power_on_seq1() {
    println!("/ ddr_phy_power_on_seq1 start");
    // RESETZ/CKE PD=0
    let v = read32(PHYA_CA_PD);
    const TX_CA_PD_CKE0: u32 = 1 << 24;
    const TX_CA_PD_RESETZ: u32 = 1 << 30;
    write32(PHYA_CA_PD, v & !TX_CA_PD_CKE0 & !TX_CA_PD_RESETZ);
    println!("  Reset PD");

    _ = read32(PHYA_CA_PD);
    write32(PHYA_CA_PD, 0);
    println!("  Set PHYA CA PD to all 0");

    // TOP_REG_TX_SEL_GPIO = 1 (DQ)
    const TX_SEL_GPIO: u32 = 1 << 7;
    let v = read32(PHYD_TX_SEL);
    write32(PHYD_TX_SEL, v | TX_SEL_GPIO);
    println!("  TX sel GPIO = 1");

    // DQ PD=0
    // TOP_REG_TX_BYTE0_PD
    // TOP_REG_TX_BYTE1_PD
    write32(PHYD_TX_BYTE, 0);
    println!("  TX BYTE PD = 0");

    // TOP_REG_TX_SEL_GPIO = 0 (DQ)
    let v = read32(PHYD_TX_SEL);
    write32(PHYD_TX_SEL, v & !TX_SEL_GPIO);
    println!("  TX sel GPIO = 0");

    println!("\\ ddr_phy_power_on_seq1 finish");
}

pub fn cvx16_ddr_phy_power_on_seq2() {
    println!("/ cvx16_ddr_phy_power_on_seq2 start");

    // OEN
    // param_phyd_sel_cke_oenz        <= `PI_SD int_regin[0];
    let v = read32(PHYD_SEL_CKE);
    write32(PHYD_SEL_CKE, v & !(1));
    // param_phyd_tx_ca_oenz          <= `PI_SD int_regin[0];
    // param_phyd_tx_ca_clk0_oenz     <= `PI_SD int_regin[8];
    // param_phyd_tx_ca_clk1_oenz     <= `PI_SD int_regin[16];
    write32(PHYD_TX_CA, 0x0);

    println!("  DLL calibration if necessary ...");
    cvx16_dll_cal();
    println!("  DLL calibration done");

    const DO_ZQ_CAL: bool = false;
    if DO_ZQ_CAL {
        println!("  ZQCAL if necessary ...");
        // zqcal hw mode
        //  bit0: offset_cal
        //  bit1: pl_en
        //  bit2: step2_en
        // cvx16_ddr_zqcal_hw_isr8(0x7);
        println!("  ZQCAL done");
    } else {
        println!("  cv181x without ZQ Calibration ...");
    }

    const DO_ZQ240_CAL: bool = false;
    if DO_ZQ240_CAL {
        println!("  ZQ240 calibration if necessary ...");
        // cvx16_ddr_zq240_cal();//zq240_cal
        println!("  ZQ240 cal done");
    } else {
        println!("  cv181x without ZQ240 Calibration ...");
    }

    const DO_ZQ_CAL_VAR: bool = false;
    if DO_ZQ_CAL_VAR {
        //  zq_cal_var();
    } else {
        println!("  ZQ calculate variation not run");
    }

    write32(PHYA_CA_PD, 0x80000000);
    println!("  All PHYA CA PD = 0 ...");
    write32(PHYD_APB + 0x00, 0x00000000);
    println!("  TX_BYTE PD = 0 ...");
    println!("\\ cvx16_ddr_phy_power_on_seq2 finish");
}

pub fn cvx16_ddr_phy_power_on_seq3() {
    println!("/ ddr_phy_power_on_seq3 start");
    // RESETYZ/CKE OENZ
    // param_phyd_sel_cke_oenz        <= `PI_SD int_regin[0];
    let v = read32(PHYD_SEL_CKE);
    write32(PHYD_SEL_CKE, v & !(0x1));
    // param_phyd_tx_ca_oenz          <= `PI_SD int_regin[0];
    // param_phyd_tx_ca_clk0_oenz     <= `PI_SD int_regin[8];
    // param_phyd_tx_ca_clk1_oenz     <= `PI_SD int_regin[16];
    write32(PHYD_TX_CA, 0x0);
    println!("  --> ca_oenz  ca_clk_oenz !!!");

    // clock gated for power save
    // param_phya_reg_tx_byte0_en_extend_oenz_gated_dline <= `PI_SD int_regin[0];
    // param_phya_reg_tx_byte1_en_extend_oenz_gated_dline <= `PI_SD int_regin[1];
    // param_phya_reg_tx_byte2_en_extend_oenz_gated_dline <= `PI_SD int_regin[2];
    // param_phya_reg_tx_byte3_en_extend_oenz_gated_dline <= `PI_SD int_regin[3];
    let v = read32(PHYD_BASE + 0x0204);
    write32(PHYD_BASE + 0x0204, v | (1 << 18));
    let v = read32(PHYD_BASE + 0x0224);
    write32(PHYD_BASE + 0x0224, v | (1 << 18));
    println!("  --> en clock gated for power save !!!");
    println!("\\ ddr_phy_power_on_seq3 finish");
}

// NOTE: CTRL settings are hardcoded; for SSC, add params to this fn
pub fn set_dpll_ssc_syn(reg_set: u32, reg_span: u32, reg_step: u32) {
    write32(DPLL_SSC_SYN_SET, reg_set);
    // 15..0
    write32(DPLL_SSC_SYN_SPAN, reg_span);
    // 23..0
    write32(DPLL_SSC_SYN_STEP, reg_step);

    const FIX_DIV: u32 = 1 << 6;
    const EXT_PULSE: u32 = 1 << 5;
    const BYPASS: u32 = 1 << 4;
    const MODE_MASK: u32 = 0b11 << 2;
    const EN_SSC: u32 = 1 << 1;
    const SW_UP: u32 = 1 << 0;
    let v = read32(DPLL_SSC_SYN_CTRL);
    println!("DPLL_SSC_SYN_CTRL {v:032b}");
    // invert SW_UP
    let neg_sw_up = !(v & SW_UP) & SW_UP;
    let m = !(FIX_DIV | EXT_PULSE | BYPASS | MODE_MASK | EN_SSC | SW_UP);
    let v = (v & m) | EXT_PULSE | neg_sw_up;
    println!("DPLL_SSC_SYN_CTRL {v:032b}");
    write32(DPLL_SSC_SYN_CTRL, v);
}

pub fn cvx16_pll_init(reg_set: u32, reg_span: u32, reg_step: u32, dram_type: &DramType) {
    println!("cvx16_pll_init");
    // opdelay(10);
    write32(TX_VREF_PD, 0x0000_0000);
    write32(ZQ_240_OPTION, 0x0008_0001);

    let x_mem_freq_2133 = false;

    // TODO: check vendor code again for real variants, it is a mess
    // use dram_type for real?
    const TX_DDR3_GPO_IN: u32 = 1 << 16;
    let v = match (dram_type, x_mem_freq_2133) {
        (_, false) => TX_DDR3_GPO_IN,
        (_, true) => 0,
    };
    write32(GPO_SETTING, 0x0100_0808 | v);

    if SSC_EN {
        /*
        //==============================================================
        // Enable SSC
        //==============================================================
        rddata = reg_set; // TOP_REG_SSC_SET
        write32(0x54 + 0x03002900, rddata);
        rddata = get_bits_from_value(reg_span, 15, 0); // TOP_REG_SSC_SPAN
        write32(0x58 + 0x03002900, rddata);
        rddata = get_bits_from_value(reg_step, 23, 0); // TOP_REG_SSC_STEP
        write32(0x5C + 0x03002900, rddata);
        KC_MSG("reg_step = %lx\n", reg_step);

        rddata = read32(0x50 + 0x03002900);
        rddata = modified_bits_by_value(rddata, ~get_bits_from_value(rddata, 0, 0), 0, 0); // TOP_REG_SSC_SW_UP
        rddata = modified_bits_by_value(rddata, 1, 1, 1); // TOP_REG_SSC_EN_SSC
        rddata = modified_bits_by_value(rddata, 0, 3, 2); // TOP_REG_SSC_SSC_MODE
        rddata = modified_bits_by_value(rddata, 0, 4, 4); // TOP_REG_SSC_BYPASS
        rddata = modified_bits_by_value(rddata, 1, 5, 5); // extpulse
        rddata = modified_bits_by_value(rddata, 0, 6, 6); // ssc_syn_fix_div
        write32(0x50 + 0x03002900, rddata);
        */
        println!("  SSC enabled");
    } else if SSC_BYPASS {
        /*
        rddata = (reg_set & 0xfc000000) + 0x04000000; // TOP_REG_SSC_SET
        write32(0x54 + 0x03002900, rddata);
        rddata = get_bits_from_value(reg_span, 15, 0); // TOP_REG_SSC_SPAN
        write32(0x58 + 0x03002900, rddata);
        rddata = get_bits_from_value(reg_step, 23, 0); // TOP_REG_SSC_STEP
        write32(0x5C + 0x03002900, rddata);
        rddata = read32(0x50 + 0x03002900);
        rddata = modified_bits_by_value(rddata, ~get_bits_from_value(rddata, 0, 0), 0, 0); // TOP_REG_SSC_SW_UP
        rddata = modified_bits_by_value(rddata, 0, 1, 1); // TOP_REG_SSC_EN_SSC
        rddata = modified_bits_by_value(rddata, 0, 3, 2); // TOP_REG_SSC_SSC_MODE
        rddata = modified_bits_by_value(rddata, 0, 4, 4); // TOP_REG_SSC_BYPASS
        rddata = modified_bits_by_value(rddata, 1, 5, 5); // TOP_REG_SSC_EXTPULSE
        rddata = modified_bits_by_value(rddata, 1, 6, 6); // ssc_syn_fix_div
        write32(0x50 + 0x03002900, rddata);
        */
        println!("  SSC bypassed");
    } else {
        set_dpll_ssc_syn(reg_set, reg_span, reg_step);
        println!("  SSC off");
    }

    // opdelay(1000);
    // DDRPLL setting
    //[0]    = 1;      //TOP_REG_DDRPLL_EN_DLLCLK
    //[1]    = 1;      //TOP_REG_DDRPLL_EN_LCKDET
    //[2]    = 0;      //TOP_REG_DDRPLL_EN_TST
    //[5:3]  = 0b001; //TOP_REG_DDRPLL_ICTRL
    //[6]    = 0;      //TOP_REG_DDRPLL_MAS_DIV_SEL
    //[8]    = 1;      //TOP_REG_DDRPLL_SEL_4BIT
    //[10:9] = 0b01;  //TOP_REG_DDRPLL_SEL_MODE
    //[12:11]= 0b00;  //Rev
    //[13]   = 0;      //TOP_REG_DDRPLL_SEL_LOW_SPEED
    //[14]   = 0;      //TOP_REG_DDRPLL_MAS_DIV_OUT_SEL
    //[15]   = 0;      //TOP_REG_DDRPLL_PD
    let v = read32(DDR_PLL);
    write32(DDR_PLL, (v & 0xffff_0000) | 0x030b);

    let v = read32(DDR_PLL_TEST);
    write32(DDR_PLL_TEST, v & 0xffff_ff00);

    write32(RESETZ_DIV, 0x1);

    let v = read32(DDR_PLL);
    write32(DDR_PLL, v | DDR_PLL_MAS_RSTZ_DIV);

    println!("Wait for PLL LOCK");
    while read32(DDR_PLL_TEST) & (1 << 15) == 0 {}
    println!("PLL init done.");
}

pub fn get_pll_settings(ddr_data_rate: usize) -> (u32, u32, u32) {
    let freq_in = 752;
    let mod_freq = 100;
    let dev_freq = 15;
    println!("Data rate = {ddr_data_rate}");
    let mut tar_freq = ddr_data_rate >> 4;
    if SSC_EN {
        tar_freq = (tar_freq as f32 * 0.985) as usize;
    };
    println!("tar_freq {tar_freq}");

    let reg_set = (freq_in * 67108864 / tar_freq) as u32;
    let reg_span = ((tar_freq * 250) / mod_freq) as u32;
    let reg_step = reg_set * dev_freq / (reg_span * 1000);
    println!("reg_set  {:032b} ({reg_set})", reg_set);
    println!("reg_span {:032b} ({reg_span})", reg_span);
    println!("reg_step {:032b} ({reg_step})", reg_step);

    (reg_set, reg_span, reg_step)
}

// pass in result of reading PHYD_BASE + 0x004c
pub fn get_pll_speed_change(v: u32) -> (bool, u32, u32) {
    // TOP_REG_EN_PLL_SPEED_CHG
    // <= #RD (~pwstrb_mask[0] & TOP_REG_EN_PLL_SPEED_CHG) |  pwstrb_mask_pwdata[0];
    // TOP_REG_CUR_PLL_SPEED   [1:0]
    // <= #RD (~pwstrb_mask[5:4] & TOP_REG_CUR_PLL_SPEED[1:0]) |  pwstrb_mask_pwdata[5:4];
    // TOP_REG_NEXT_PLL_SPEED  [1:0]
    // <= #RD (~pwstrb_mask[9:8] & TOP_REG_NEXT_PLL_SPEED[1:0]) |  pwstrb_mask_pwdata[9:8];
    let en_pll_speed = v & 0b1 == 1;
    let curr_pll_speed = (v >> 4) & 0b11;
    let next_pll_speed = (v >> 8) & 0b11;
    println!("  en_pll_speed     {en_pll_speed}");
    println!("  curr_pll_speed   {curr_pll_speed}");
    println!("  next_pll_speed   {next_pll_speed}");
    (en_pll_speed, curr_pll_speed, next_pll_speed)
}

pub fn change_pll_freq(reg_set: u32, reg_span: u32, reg_step: u32) {
    println!("/ change_pll_freq start");
    println!("  Change PLL frequency if necessary ...");
    write32(RESETZ_DIV, 0);
    write32(RESETZ_DQS, 0);
    let v = read32(DDR_PLL);
    write32(DDR_PLL, v & !DDR_PLL_MAS_RSTZ_DIV);

    // NOTE: Reading a register may have meaning in hardware.
    // Yes, the vendor code reads this 6x. It _may_ have an effect.
    for _ in 0..5 {
        read32(PHYD_SPEED);
    }
    let v = read32(PHYD_SPEED);
    let (en_chg, curr_speed, next_speed) = get_pll_speed_change(v);

    let v = (v & !(0b11 << 4)) | (next_speed << 4);
    let v = (v & !(0b11 << 8)) | (curr_speed << 8);
    if (en_chg) {
        match next_speed {
            0 => {
                write32(PHYD_SPEED, v);
                cvx16_clk_div40();
            }
            1 => {
                write32(PHYD_SPEED, v);
                cvx16_clk_div2();
            }
            2 => {
                write32(PHYD_SPEED, v);
                cvx16_clk_normal(reg_set, reg_span, reg_step);
            }
            _ => {}
        }
        // opdelay(100000);  //  1000ns
    }

    // NOTE: similar to cvx16_pll_init
    write32(RESETZ_DIV, 1);
    let v = read32(DDR_PLL);
    write32(DDR_PLL, v | DDR_PLL_MAS_RSTZ_DIV);
    write32(RESETZ_DQS, 1);

    const DDR_PLL_SLV_LOCK: u32 = 1 << 15;
    println!("  Wait for DDR PLL_SLV_LOCK = 1...");
    while read32(DDR_PLL_TEST) & DDR_PLL_SLV_LOCK == 0 {
        opdelay(200);
    }

    println!("\\ change_pll_freq finish");
}

pub fn cvx16_clk_div40() {
    println!("  clk_div40");
    let v = read32(DDR_PLL);
    // TOP_REG_DDRPLL_SEL_LOW_SPEED = 1
    write32(DDR_PLL, v | (1 << 13));
}

pub fn cvx16_clk_div2() {
    println!("  clk_div2");
    let v = read32(DDR_PLL);
    // TOP_REG_DDRPLL_MAS_DIV_OUT_SEL 1
    write32(DDR_PLL, v | (1 << 14));
}

pub fn cvx16_clk_normal(reg_set: u32, reg_span: u32, reg_step: u32) {
    println!("  clk_normal");
    let v = read32(DDR_PLL);
    // TOP_REG_DDRPLL_SEL_LOW_SPEED 0
    // TOP_REG_DDRPLL_MAS_DIV_OUT_SEL 0
    write32(DDR_PLL, v & !((1 << 13) | (1 << 14)));

    // NOTE: similar to cvx16_pll_init
    if SSC_EN {
        /*
        write32(0x54 + 0x03002900, reg_set);
        // TOP_REG_SSC_SPAN
        rddata = get_bits_from_value(reg_span, 15, 0);
        write32(0x58 + 0x03002900, rddata);
        // TOP_REG_SSC_STEP
        rddata = get_bits_from_value(reg_step, 23, 0);
        write32(0x5C + 0x03002900, rddata);
        rddata = read32(0x50 + 0x03002900);
        // TOP_REG_SSC_SW_UP
        rddata = modified_bits_by_value(rddata, ~get_bits_from_value(rddata, 0, 0), 0, 0);
        // TOP_REG_SSC_EN_SSC
        rddata = modified_bits_by_value(rddata, 1, 1, 1);
        // TOP_REG_SSC_SSC_MODE
        rddata = modified_bits_by_value(rddata, 0, 3, 2);
        // TOP_REG_SSC_BYPASS
        rddata = modified_bits_by_value(rddata, 0, 4, 4);
        // extpulse
        rddata = modified_bits_by_value(rddata, 1, 5, 5);
        // ssc_syn_fix_div
        rddata = modified_bits_by_value(rddata, 0, 6, 6);
        write32(0x50 + 0x03002900, rddata);
        */
        println!("  SSC enabled");
    }
    if SSC_BYPASS {
        /*
        // TOP_REG_SSC_SET
        rddata = (reg_set & 0xfc000000) + 0x04000000;
        write32(0x54 + 0x03002900, rddata);
        // TOP_REG_SSC_SPAN
        rddata = get_bits_from_value(reg_span, 15, 0);
        write32(0x58 + 0x03002900, rddata);
        // TOP_REG_SSC_STEP
        rddata = get_bits_from_value(reg_step, 23, 0);
        write32(0x5C + 0x03002900, rddata);
        rddata = read32(0x50 + 0x03002900);
        // TOP_REG_SSC_SW_UP
        rddata = modified_bits_by_value(rddata, ~get_bits_from_value(rddata, 0, 0), 0, 0);
        // TOP_REG_SSC_EN_SSC
        rddata = modified_bits_by_value(rddata, 0, 1, 1);
        // TOP_REG_SSC_SSC_MODE
        rddata = modified_bits_by_value(rddata, 0, 3, 2);
        // TOP_REG_SSC_BYPASS
        rddata = modified_bits_by_value(rddata, 0, 4, 4);
        // TOP_REG_SSC_EXTPULSE
        rddata = modified_bits_by_value(rddata, 1, 5, 5);
        // ssc_syn_fix_div
        rddata = modified_bits_by_value(rddata, 1, 6, 6);
        */
        println!("  SSC bypassed");
    } else {
        set_dpll_ssc_syn(reg_set, reg_span, reg_step);
        println!("  SSC off");
    }
    println!("  back to original frequency");
}

pub fn cvx16_int_isr_08() {
    println!("/ cvx16_int_isr_08 start");
    write32(PHYD_BASE + 0x0118, 0x0);
    let v = read32(PHYD_SPEED);
    let _ = get_pll_speed_change(v);
    println!("\\ cvx16_int_isr_08 finish");
}

pub fn cvx16_clk_gating_enable() {
    println!("/ cvx16_clk_gating_enable");
    // TOP_REG_CG_EN_PHYD_TOP      0
    // TOP_REG_CG_EN_CALVL         1
    // TOP_REG_CG_EN_WRLVL         2
    // N/A                         3
    // TOP_REG_CG_EN_WRDQ          4
    // TOP_REG_CG_EN_RDDQ          5
    // TOP_REG_CG_EN_PIGTLVL       6
    // TOP_REG_CG_EN_RGTRACK       7
    // TOP_REG_CG_EN_DQSOSC        8
    // TOP_REG_CG_EN_LB            9
    // TOP_REG_CG_EN_DLL_SLAVE     10 //0:a-on
    // TOP_REG_CG_EN_DLL_MST       11 //0:a-on
    // TOP_REG_CG_EN_ZQ            12
    // TOP_REG_CG_EN_PHY_PARAM     13 //0:a-on
    // 0b10110010000001
    write32(CLOCK_GATING_ENABLE, 0x00002C81);
    // #ifdef _mem_freq_1333
    // #ifdef DDR2
    let v = read32(DDR_CFG_BASE + 0x190);
    let v = v & !(0b11111 << 24) | (6 << 24);
    write32(DDR_CFG_BASE + 0x190, v);
    // #endif
    write32(PHYD_SHIFT_GATING_EN, 0x00030033);
    // phyd_stop_clk
    let v = read32(POWER_CONTROL);
    write32(POWER_CONTROL, v | (1 << 9));
    // dfi read/write clock gatting
    let v = read32(DDR_CFG_BASE + 0x148);
    write32(DDR_CFG_BASE + 0x148, v | (1 << 23) | (1 << 31));

    // disable clock gating
    // write32(CLOCK_GATING_CONTROL , 0x00000fff);
    // println!("axi disable clock gating");
    println!("\\ cvx16_clk_gating_enable finish");
}

pub fn cvx16_clk_gating_disable() {
    // TOP_REG_CG_EN_PHYD_TOP      0
    // TOP_REG_CG_EN_CALVL         1
    // TOP_REG_CG_EN_WRLVL         2
    // N/A                         3
    // TOP_REG_CG_EN_WRDQ          4
    // TOP_REG_CG_EN_RDDQ          5
    // TOP_REG_CG_EN_PIGTLVL       6
    // TOP_REG_CG_EN_RGTRACK       7
    // TOP_REG_CG_EN_DQSOSC        8
    // TOP_REG_CG_EN_LB            9
    // TOP_REG_CG_EN_DLL_SLAVE     10 //0:a-on
    // TOP_REG_CG_EN_DLL_MST       11 //0:a-on
    // TOP_REG_CG_EN_ZQ            12
    // TOP_REG_CG_EN_PHY_PARAM     13 //0:a-on
    // 0b01001011110101
    write32(CLOCK_GATING_ENABLE, 0x000012F5);
    write32(PHYD_SHIFT_GATING_EN, 0x00000000);
    // phyd_stop_clk
    let v = read32(POWER_CONTROL);
    let v = v & !(1 << 9);
    write32(POWER_CONTROL, v);
    // dfi read/write clock gatting
    let v = read32(DDR_CFG_BASE + 0x148);
    let v = v & !((1 << 23) | (1 << 31));
    write32(DDR_CFG_BASE + 0x148, v);
    println!("  clk_gating_disable");

    // disable clock gating
    // write32(CLOCK_GATING_CONTROL , 0x00000fff);
    // println!("axi disable clock gating");
}
