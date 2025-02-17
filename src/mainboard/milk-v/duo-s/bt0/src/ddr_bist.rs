use util::{read32, read64, write32};

use crate::dram::DdrType;
use crate::mem_map::{DDR_BIST_BASE, DDR_CFG_BASE, PHYD_BASE, PHY_BASE};
use crate::{ddr_bist, ddr_ctrl, ddr_pll};

use ddr_ctrl::REFRESH_CONTROL3;

const DDR4: bool = false;
const X16_MODE: bool = true;

const BIST_OP_WRITE: u32 = 1 << 30;
const BIST_OP_READ: u32 = 2 << 30;
const BIST_OP_GOTO: u32 = 3 << 30;

// command queue: 6 registers
// 31..30: op code; 1: write, 2: read
// 29..21: start
// 20..12: stop
// 8: DQ invert
// 7: DM invert
// 6..4: DQ rotate
// 3..0: repetitions

pub enum DataMode {
    PhydPattern,
    BistReadWrite,
    MultiBistReadWriteWithErrorInject1,
    MultiBistReadWriteWithErrorInject2,
}

pub enum LvlMode {
    WdmLvl,
    WdqLvl,
    WdqAndWdmLvl,
}

pub enum XMode {
    Mpr,
    SramWriteReadContinuousGoto,
    MultiBistWriteRead,
    MultiBistReadWriteWithErrorInject1,
    MultiBistReadWriteWithErrorInject2,
}

fn get_sram_sp_and_set_sso_period() -> u32 {
    let fmax = 15;
    let fmin = 5;
    let fdiff = (fmax - fmin + 1);
    // 8*f/4 -1
    let sram_sp = 9 * (fmin + fmax) * fdiff / 2 / 4 + fdiff;

    // bist sso_period
    write32(DDR_BIST_BASE + 0x24, (fmax << 8) + fmin);

    sram_sp
}

fn bist_cmd_rw() {
    let sram_sp = get_sram_sp_and_set_sso_period();
    let base1 = (511 << 12) | (5 << 9);
    let base2 = (sram_sp << 12) | (6 << 9);
    write32(DDR_BIST_BASE + 0x40, BIST_OP_WRITE | base1);
    write32(DDR_BIST_BASE + 0x44, BIST_OP_READ | base1);
    write32(DDR_BIST_BASE + 0x48, BIST_OP_WRITE | base2);
    write32(DDR_BIST_BASE + 0x4c, BIST_OP_READ | base2);
    //                                      addr_not_reset   loop_cnt
    write32(DDR_BIST_BASE + 0x50, BIST_OP_GOTO | (0 << 20) | (1 << 0));
    // NOP
    write32(DDR_BIST_BASE + 0x54, 0);
}

fn bist_x_init_finish() {
    // specified DDR space
    write32(DDR_BIST_BASE + 0x10, 0x00000000);
    write32(DDR_BIST_BASE + 0x14, 0x000fffff);
    // specified AXI address step
    let v = if X16_MODE { 0x00000004 } else { 0x00000008 };
    write32(DDR_BIST_BASE + 0x18, v);
}

pub fn cvx16_bist_wr_prbs_init() {
    println!("    bist_wr_prbs_init");
    // bist clock enable
    write32(DDR_BIST_BASE + 0x0, 0x00060006);

    let base_cmd = (511 << 12) | (5 << 9);
    // W  1~17  prbs  repeat0
    let cmd1 = BIST_OP_WRITE | base_cmd;
    // R  1~17  prbs  repeat0
    let cmd2 = BIST_OP_READ | base_cmd;
    // write cmd queue
    write32(DDR_BIST_BASE + 0x40, cmd1);
    write32(DDR_BIST_BASE + 0x44, cmd2);
    // NOP
    for i in 2..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }

    bist_x_init_finish();
    println!("    bist_wr_prbs_init done");
}

fn cvx16_bist_rdlvl_init(mode: XMode) {
    println!("    bist_rdlvl_init");
    // bist clock enable
    write32(DDR_BIST_BASE + 0x0, 0x00060006);

    match mode {
        XMode::Mpr => {
            // MPR mode
            // TODO
        }
        XMode::SramWriteReadContinuousGoto => bist_cmd_rw(),
        // TODO
        _ => {}
    }

    bist_x_init_finish();
    println!("    bist_rdlvl_init done");
}

pub fn cvx16_bist_wrlvl_init() {
    println!("    bist_wrlvl_init");
    // bist clock enable
    write32(DDR_BIST_BASE + 0x0, 0x00060006);

    let cmd = BIST_OP_WRITE | (5 << 9);
    write32(DDR_BIST_BASE + 0x40, cmd);
    // NOP
    for i in 1..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }

    bist_x_init_finish();
    println!("    bist_wrlvl_init done");
}

pub fn cvx16_bist_rdglvl_init() {
    println!("    bist_rdglvl_init");
    // bist clock enable
    write32(DDR_BIST_BASE + 0x0, 0x00060006);

    let cmd = BIST_OP_READ | (0 << 21) | (3 << 12) | (5 << 9);
    write32(DDR_BIST_BASE + 0x40, cmd);
    // NOP
    for i in 1..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }

    bist_x_init_finish();
    println!("    bist_rdglvl_init done");
}

pub fn cvx16_bist_wdmlvl_init() {
    println!("    bist_wdmlvl_init");
    // bist clock enable
    write32(DDR_BIST_BASE + 0x0, 0x00060006);

    let sram_sp = get_sram_sp_and_set_sso_period();
    println!("      sram_sp = {sram_sp:08x}");

    let cmd1 = BIST_OP_WRITE | (sram_sp << 12) | (3 << 9);
    let cmd2 = BIST_OP_WRITE | (sram_sp << 12) | (7 << 9);
    let cmd3 = BIST_OP_READ | (sram_sp << 12) | (7 << 9);
    write32(DDR_BIST_BASE + 0x40, cmd1);
    write32(DDR_BIST_BASE + 0x44, cmd2);
    write32(DDR_BIST_BASE + 0x48, cmd3);
    // NOP
    for i in 3..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }

    bist_x_init_finish();
    println!("    bist_wdmlvl_init done");
}

fn cvx16_bist_wdqlvl_init(mode: &DataMode) {
    println!("    bist_wdqlvl_init");
    // bist clock enable
    write32(DDR_BIST_BASE + 0x0, 0x00060006);

    match mode {
        DataMode::PhydPattern => {
            let base_cmd = (0 << 21) | (3 << 12) | (0b0101 << 9);
            let cmd1 = BIST_OP_WRITE | base_cmd;
            let cmd2 = BIST_OP_READ | base_cmd;
            write32(DDR_BIST_BASE + 0x40, cmd1);
            write32(DDR_BIST_BASE + 0x44, cmd2);
            // NOP
            for i in 0..4 {
                write32(DDR_BIST_BASE + 0x48 + i * 4, 0);
            }
        }
        DataMode::BistReadWrite => bist_cmd_rw(),
        DataMode::MultiBistReadWriteWithErrorInject1 => {
            // TODO
        }
        DataMode::MultiBistReadWriteWithErrorInject2 => {
            // TODO
        }
        _ => {
            // TODO
        }
    }
    bist_x_init_finish();
    println!("    bist_wdqlvl_init done");
}

pub fn cvx16_wdqlvl_req(data_mode: &DataMode, lvl_mode: LvlMode) {
    // NOTE: training need ctrl_low_patch first
    let (
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en, //
    ) = ddr_ctrl::pwrctl_init();

    ddr_pll::cvx16_clk_gating_disable();
    println!("   cvx16_dfi_ca_park_prbs  start");
    ddr_pll::cvx16_dfi_ca_park_prbs(true);
    println!("   cvx16_dfi_ca_park_prbs  done");

    // param_phyd_piwdqlvl_dq_mode
    // <= #RD (~pwstrb_mask[12] & param_phyd_piwdqlvl_dq_mode) | pwstrb_mask_pwdata[12];
    // param_phyd_piwdqlvl_dm_mode
    // <= #RD (~pwstrb_mask[13] & param_phyd_piwdqlvl_dm_mode) | pwstrb_mask_pwdata[13];
    // 13: param_phyd_piwdqlvl_dm_mode
    // 12: param_phyd_piwdqlvl_dq_mode
    let bb = match lvl_mode {
        LvlMode::WdmLvl => 1 << 13,
        LvlMode::WdqLvl => 1 << 12,
        LvlMode::WdqAndWdmLvl => (1 << 13) | (1 << 12),
    };
    let v = read32(0x00BC + PHYD_BASE);
    write32(0x00BC + PHYD_BASE, v & !(0b11 << 12) | bb);

    match lvl_mode {
        LvlMode::WdmLvl => {
            let v = read32(DDR_CFG_BASE + 0xC);
            write32(DDR_CFG_BASE + 0xC, v | (1 << 17));
            // cvx16_bist_wdmlvl_init(sram_sp);
            cvx16_bist_wdmlvl_init();
        }
        _ => {
            // bist setting for dfi rdglvl
            // data_mode = 0x0 : phyd pattern
            // data_mode = 0x1 : bist read/write
            // data_mode = 0x11: with Error enject,  multi- bist write/read
            // data_mode = 0x12: with Error enject,  multi- bist write/read
            // cvx16_bist_wdqlvl_init(data_mode, sram_sp);
            cvx16_bist_wdqlvl_init(data_mode);
        }
    }

    // param_phyd_dfi_wdqlvl
    let v = read32(PHYD_BASE + 0x018C);
    println!("      phyd_dfi_wdqlvl {v:08x}");
    // req
    let v = v | 0b1;
    let vref_train_en = match lvl_mode {
        LvlMode::WdmLvl => 0,
        _ => 1,
    };
    let bist_data_en = match *data_mode {
        DataMode::BistReadWrite
        | DataMode::MultiBistReadWriteWithErrorInject1
        | DataMode::MultiBistReadWriteWithErrorInject2 => 1,
        _ => 0,
    };
    let clr_mask = !((1 << 10) | (1 << 4));
    let v = (v & clr_mask) | (vref_train_en << 10) | (bist_data_en << 4);
    write32(PHYD_BASE + 0x018C, v);
    println!("      phyd_dfi_wdqlvl {v:08x}");

    println!("    wait retraining finish ...");
    //[0] param_phyd_dfi_wrlvl_done
    //[1] param_phyd_dfi_rdglvl_done
    //[2] param_phyd_dfi_rdlvl_done
    //[3] param_phyd_dfi_wdqlvl_done
    while read32(PHYD_BASE + 0x3444) & (1 << 3) == 0 {}

    let v = read32(DDR_CFG_BASE + 0xC);
    write32(DDR_CFG_BASE + 0xC, v & !(1 << 7));
    // BIST clock disable
    write32(DDR_BIST_BASE + 0x0, 0x00040000);

    ddr_pll::cvx16_dfi_ca_park_prbs(false);

    ddr_ctrl::pwrctl_restore(
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en,
    );

    // cvx16_wdqlvl_status();
    ddr_pll::cvx16_clk_gating_enable();
}

pub fn cvx16_wrlvl_req(ddr_type: &DdrType) {
    // NOTE: training need ctrl_low_patch first
    // wrlvl response only DQ0
    write32(PHYD_BASE + 0x005C, 0x00FE0000);

    let (
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en, //
    ) = ddr_ctrl::pwrctl_init();

    ddr_pll::cvx16_clk_gating_disable();

    // save ctrl wr_odt_en
    let v = read32(DDR_CFG_BASE + 0x244);
    let wr_odt_en = v & 0b1;

    // bist setting for dfi wrlvl
    cvx16_bist_wrlvl_init();

    // RFSHCTL3.dis_auto_refresh = 1
    // let v = read32(REFRESH_CONTROL3);
    // write32(REFRESH_CONTROL3, v | 1);

    let ddr3 = *ddr_type == DdrType::Ddr3;
    if ddr3 {
        let mut rtt_nom = 0;
        if (wr_odt_en == 1) {
            println!("wr_odt_en = 1 ...");

            let v = read32(DDR_CFG_BASE + 0xe0);
            // save rtt_wr bits 26..25
            let rtt_wr = (v >> 25) & 0b11;
            if (rtt_wr != 0x0) {
                // disable rtt_wr
                let v = v & !(0b11 << 25);
                // MR2
                ddr_ctrl::cvx16_synp_mrw(0x2, v >> 16);
                // set rtt_nom
                rtt_nom = read32(DDR_CFG_BASE + 0xdc);
                // rtt_nom[2]=0
                rtt_nom = rtt_nom & !(1 << 9);
                // rtt_nom[1]=rtt_wr[1]
                let b = (rtt_wr >> 1) & 0b1;
                rtt_nom = rtt_nom & !(1 << 6) | (b << 6);
                // rtt_nom[1]=rtt_wr[0]
                let b = rtt_wr & 0b1;
                rtt_nom = rtt_nom & !(1 << 2) | (b << 2);
                println!("dodt for wrlvl setting");
            }
        } else {
            println!("rtt_nom for wrlvl setting");
            println!("wr_odt_en = 0 ...");

            // set rtt_nom = 120ohm
            rtt_nom = read32(DDR_CFG_BASE + 0xdc);
            // rtt_nom[2]=0
            rtt_nom = rtt_nom & !(1 << 9);
            // rtt_nom[1]=1
            rtt_nom = rtt_nom | (1 << 6);
            // rtt_nom[1]=0
            rtt_nom = rtt_nom & !(1 << 2);
            ddr_ctrl::cvx16_synp_mrw(0x1, rtt_nom & 0xffff);
        }
        // Write leveling enable
        rtt_nom = rtt_nom | (1 << 7);
        ddr_ctrl::cvx16_synp_mrw(0x1, rtt_nom & 0xffff);
        println!("DDR3 MRS rtt_nom ...");
    }

    if DDR4 {
        let v = read32(DDR_CFG_BASE + 0xdc);
        // Write leveling enable
        let v = v | (1 << 7);
        ddr_ctrl::cvx16_synp_mrw(0x1, v & 0xffff);
    }

    let v = read32(PHYD_BASE + 0x0180);
    // param_phyd_dfi_wrlvl_req
    let v = v | 1;
    // param_phyd_dfi_wrlvl_odt_en
    let v = v & !(1 << 4) | (wr_odt_en << 4);
    write32(PHYD_BASE + 0x0180, v);

    println!("wait retraining finish ...");
    //[0] param_phyd_dfi_wrlvl_done
    //[1] param_phyd_dfi_rdglvl_done
    //[2] param_phyd_dfi_rdlvl_done
    //[3] param_phyd_dfi_wdqlvl_done
    while read32(PHYD_BASE + 0x3444) & (1 << 0) == 0 {}
    // BIST clock disable
    write32(DDR_BIST_BASE + 0x0, 0x00040000);

    // dis_auto_refresh = 0
    let v = read32(REFRESH_CONTROL3);
    write32(REFRESH_CONTROL3, v & !(0b1));

    if ddr3 {
        let v = read32(DDR_CFG_BASE + 0xdc);
        // let v = v & !(1 << 7);
        // Write leveling disable
        ddr_ctrl::cvx16_synp_mrw(0x1, v & 0xffff);
        let v = read32(DDR_CFG_BASE + 0xe0);
        // MR2
        ddr_ctrl::cvx16_synp_mrw(0x2, v >> 16);
    }

    if DDR4 {
        let v = read32(DDR_CFG_BASE + 0xdc);
        // let v = v & !(1 << 7);
        // Write leveling disable
        ddr_ctrl::cvx16_synp_mrw(0x1, v & 0xffff);
    }

    ddr_ctrl::pwrctl_restore(
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en,
    );

    // cvx16_wrlvl_status();
    ddr_pll::cvx16_clk_gating_enable();
}

pub fn cvx16_wdqlvl_sw_req(x: u32, y: u32) {
    //
}

pub fn cvx16_bist_wr_sram_init() {
    // TODO
}

pub fn cvx16_rdlvl_req(mode: XMode) {
    // Note: training need ctrl_low_patch first
    let (
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en, //
    ) = ddr_ctrl::pwrctl_init();
    ddr_pll::cvx16_clk_gating_disable();

    println!("   cvx16_dfi_ca_park_prbs  start");
    ddr_pll::cvx16_dfi_ca_park_prbs(true);
    println!("   cvx16_dfi_ca_park_prbs  done");

    const PI_READ_LEVEL: usize = PHYD_BASE + 0x0080;
    const PI_READ_LEVEL_DESKEW_START_MASK: u32 = 0b1111111 << 16;
    const PI_READ_LEVEL_DESKEW_END_MASK: u32 = 0b1111111 << 24;
    let deskew_start = 0x20;
    let deskew_end = 0x1f;
    let v = read32(PI_READ_LEVEL);
    let m = PI_READ_LEVEL_DESKEW_START_MASK | PI_READ_LEVEL_DESKEW_END_MASK;
    let v = (v & !m) | (deskew_start << 16) | (deskew_end << 24);
    write32(PI_READ_LEVEL, v);

    const PI_READ_LEVEL_X: usize = PHYD_BASE + 0x008c;
    const PI_READ_LEVEL_RX_INIT_DESKEW_EN: u32 = 1 << 1;
    const PI_READ_LEVEL_VREF_TRAINING_EN: u32 = 1 << 2;
    const PI_READ_LEVEL_RDVLD_TRAINING_EN: u32 = 1 << 3;
    let v = read32(PI_READ_LEVEL_X);
    let vref_training_en = v & PI_READ_LEVEL_VREF_TRAINING_EN > 0;
    let m = PI_READ_LEVEL_RX_INIT_DESKEW_EN
        | PI_READ_LEVEL_VREF_TRAINING_EN
        | PI_READ_LEVEL_RDVLD_TRAINING_EN;
    write32(PI_READ_LEVEL_X, v & !m);

    let v = read32(PHYD_BASE + 0x0188);
    let ddr3_mpr_mode = v & (1 << 4) > 0;
    if ddr3_mpr_mode {
        let v = read32(REFRESH_CONTROL3);
        write32(REFRESH_CONTROL3, v | 1);
        // MR3
        let v = read32(DDR_CFG_BASE + 0xe0);
        // Dataflow from MPR
        let v = v | (1 << 2);
        ddr_ctrl::cvx16_synp_mrw(0x3, v & 0xffff);
    }

    // bist setting for dfi rdglvl
    cvx16_bist_rdlvl_init(mode);

    fn train(i: u32) {
        println!("  dfi rdlvl req {i}");
        let v = read32(PHYD_BASE + 0x0188);
        // param_phyd_dfi_rdlvl_req
        write32(PHYD_BASE + 0x0188, v | 1);
        println!("  wait retraining finish ...");
        //[0] param_phyd_dfi_wrlvl_done
        //[1] param_phyd_dfi_rdglvl_done
        //[2] param_phyd_dfi_rdlvl_done
        //[3] param_phyd_dfi_wdqlvl_done
        while read32(PHYD_BASE + 0x3444) & (1 << 2) == 0 {}
    }
    train(1);

    if vref_training_en {
        println!("  VREF training");
        let v = read32(PI_READ_LEVEL_X);
        // disable Vref training enable
        write32(PI_READ_LEVEL_X, v & !PI_READ_LEVEL_VREF_TRAINING_EN);
        println!("  final training, keep rx trig_lvl");
        train(2);
        let v = read32(PI_READ_LEVEL_X);
        // restore Vref training enable
        write32(PI_READ_LEVEL_X, v | PI_READ_LEVEL_VREF_TRAINING_EN);
    }

    // if ddr3 &&
    if ddr3_mpr_mode {
        // MR3
        let v = read32(DDR_CFG_BASE + 0xe0);
        // Dataflow from MPR
        let v = v & !(1 << 2);
        ddr_ctrl::cvx16_synp_mrw(0x3, v & 0xffff);
        let v = read32(REFRESH_CONTROL3);
        write32(REFRESH_CONTROL3, v & !1);
    }

    cvx16_rdvld_train();

    // refresh control disable...

    // BIST clock disable
    write32(DDR_BIST_BASE + 0x0, 0x00040000);
    ddr_pll::cvx16_dfi_ca_park_prbs(false);

    ddr_ctrl::pwrctl_restore(
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en,
    );

    // cvx16_rdlvl_status();
    ddr_pll::cvx16_clk_gating_enable();
}

fn cvx16_rdvld_train() {
    cvx16_bist_wr_prbs_init();
    // cvx16_bist_wr_sram_init();

    let byte0_vld = read32(PHYD_BASE + 0x0b14);
    let byte1_vld = read32(PHYD_BASE + 0x0b44);
    let rdvld = read32(PHY_BASE + 0x0094);
    let rdvld_offset = rdvld & 0b1111;

    let m = 0b11111 << 16;

    for i in (1..9).rev() {
        write32(PHYD_BASE + 0x0b14, (byte0_vld & !m) | (i << 16));
        write32(PHYD_BASE + 0x0b44, (byte1_vld & !m) | (i << 16));
        // one step too far
        if bist().is_err() {
            write32(PHYD_BASE + 0x0b14, (byte0_vld & !m) | ((i + 1) << 16));
            write32(PHYD_BASE + 0x0b44, (byte1_vld & !m) | ((i + 1) << 16));
            break;
        }
    }
}

pub fn cvx16_rdlvl_sw_req(x: u32) {
    //
}

pub fn cvx16_rdglvl_req(ddr_type: &DdrType) {
    // NOTE: training need ctrl_low_patch first
    let (
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en, //
    ) = ddr_ctrl::pwrctl_init();

    ddr_pll::cvx16_clk_gating_disable();

    // dis_auto_refresh = 1
    // let v = read32(REFRESH_CONTROL3);
    // write32(REFRESH_CONTROL3, v | 1);

    let ddr3 = *ddr_type == DdrType::Ddr3;
    let ddr3_mpr_mode = read32(PHYD_BASE + 0x0184) & (1 << 4) != 0;

    if ddr3 && ddr3_mpr_mode {
        // dis_auto_refresh =1
        let v = read32(REFRESH_CONTROL3);
        write32(REFRESH_CONTROL3, v | 0x1);
        // MR3
        let v = read32(DDR_CFG_BASE + 0xe0);
        // Dataflow from MPR
        let v = v | (1 << 2);
        ddr_ctrl::cvx16_synp_mrw(0x3, v & 0xffff);
    }

    // bist setting for dfi rdglvl
    cvx16_bist_rdglvl_init();

    // param_phyd_dfi_rdglvl_req
    let v = read32(PHYD_BASE + 0x0184);
    write32(PHYD_BASE + 0x0184, v | 1);

    println!("wait retraining finish ...");
    //[0] param_phyd_dfi_wrlvl_done
    //[1] param_phyd_dfi_rdglvl_done
    //[2] param_phyd_dfi_rdlvl_done
    //[3] param_phyd_dfi_wdqlvl_done
    while read32(PHYD_BASE + 0x3444) & (1 << 1) == 0 {}
    // BIST clock disable
    write32(DDR_BIST_BASE + 0x0, 0x00040000);

    if ddr3 && ddr3_mpr_mode {
        // MR3
        let v = read32(DDR_CFG_BASE + 0xe0);
        // Normal operation
        let v = v & !(1 << 2);
        ddr_ctrl::cvx16_synp_mrw(0x3, v & 0xffff);
        // dis_auto_refresh = 0
        let v = read32(REFRESH_CONTROL3);
        write32(REFRESH_CONTROL3, v & !1);
    }

    ddr_ctrl::pwrctl_restore(
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en,
    );

    // cvx16_rdglvl_status();
    ddr_pll::cvx16_clk_gating_enable();
}

pub fn bist() -> Result<(), ()> {
    // bist enable
    let v = if X16_MODE { 0x00030003 } else { 0x00010001 };
    write32(DDR_BIST_BASE + 0x0, v);
    println!(">> BIST start");
    let res = loop {
        let r = read32(DDR_BIST_BASE + 0x0080);
        if r & (1 << 2) != 0 {
            break r;
        }
    };
    let success = res & (1 << 3) == 0;
    let (odd, even) = if success {
        // read err_data
        let o = read64(DDR_BIST_BASE + 0x0088);
        let e = read64(DDR_BIST_BASE + 0x0090);
        (o, e)
    } else {
        (0, 0)
    };
    // BIST disable
    write32(DDR_BIST_BASE + 0x0, 0x00050000);

    if success {
        println!("-  BIST success");
        Ok(())
    } else {
        println!("-  BIST err_data_odd  {odd:016x}");
        println!("-  BIST err_data_even {even:016x}");
        Err(())
    }
}

fn bist_poll() -> u32 {
    // BIST enable
    write32(DDR_BIST_BASE + 0x0, 0x00010001);
    // poll for BIST done
    let res = loop {
        let r = read32(DDR_BIST_BASE + 0x80);
        if r & (1 << 2) != 0 {
            break r;
        }
    };
    // BIST disable
    write32(DDR_BIST_BASE + 0x0, 0x00010000);
    println!("          BIST poll: {res:08x}");
    res
}

fn bist_write_prbs() {
    // write PRBS to 0x0 as background
    let cmd = BIST_OP_WRITE | (3 << 12) | (5 << 9);
    write32(DDR_BIST_BASE + 0x40, cmd);
    // NOP
    for i in 1..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }
}

fn bist_write_16_ui_prbs() {
    // write 16 UI~prbs
    let cmd = BIST_OP_WRITE | (3 << 12) | (5 << 9) | (1 << 8);
    write32(DDR_BIST_BASE + 0x40, cmd);
    // NOP
    for i in 1..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }
}

fn bist_read_16_ui_prbs() {
    // read 16 UI prbs
    let cmd = BIST_OP_READ | (3 << 12) | (5 << 9);
    write32(DDR_BIST_BASE + 0x40, cmd);
    // NOP
    for i in 1..6 {
        write32(DDR_BIST_BASE + 0x40 + i * 4, 0);
    }
}

pub fn ddr3_get_cap_in_mbyte() -> u32 {
    let mut cap_in_mbyte = 4;
    // Axsize = 3, axlen = 4, cgen
    write32(DDR_BIST_BASE + 0x0, 0x000e0006);
    // DDR space
    write32(DDR_BIST_BASE + 0x10, 0x00000000);
    write32(DDR_BIST_BASE + 0x14, 0xffffffff);
    // specified AXI address step
    write32(DDR_BIST_BASE + 0x18, 0x00000004);

    bist_write_prbs();
    let mut res = bist_poll();
    // BIST may fail stop the loop (?)
    while cap_in_mbyte < 15 {
        cap_in_mbyte += 1;
        println!("    cap_in_mbyte = {cap_in_mbyte}");
        // DDR space
        write32(DDR_BIST_BASE + 0x10, 1 << (cap_in_mbyte + 20 - 4));
        // write ~PRBS to (0x1 << *dram_cap_in_mbyte)
        bist_write_16_ui_prbs();
        res = bist_poll();
        // check PRBS at 0x0
        bist_read_16_ui_prbs();
        res = bist_poll();

        if res & (1 << 3) != 0 {
            break;
        }
    }
    cap_in_mbyte
}

pub fn detect_dram_size(ddr_type: &DdrType) -> u32 {
    let cap_in_mbyte = match *ddr_type {
        DdrType::Ddr3 => ddr3_get_cap_in_mbyte(),
        DdrType::Ddr2 => 6,
        _ => 0,
    };

    // save dram_cap_in_mbyte
    write32(PHYD_BASE + 0x0208, cap_in_mbyte);

    // clock gen: BIST clock disable
    write32(DDR_BIST_BASE + 0x0, 0x00040000);

    cap_in_mbyte
}
