use util::{read32, write32};

use crate::ddr_bist;
use crate::mem_map::{DDR_BIST_BASE, DDR_CFG_BASE, DDR_TOP_BASE, PHYD_BASE};

const DDR_INIT_SPEED_UP: bool = false;
const DDR_DODT: bool = false;

// DDR TOP
const CLOCK_GATING_CONTROL: usize = DDR_TOP_BASE + 0x0014;
const DDRC_RESET: usize = DDR_TOP_BASE + 0x0020;

// DDR CFG
pub const POWER_CONTROL: usize = DDR_CFG_BASE + 0x0030;
const POWER_CONTROL_SELF_REFRESH_SW: u32 = 1 << 5;
const POWER_CONTROL_DFI_DRAM_CLOCK_EN: u32 = 1 << 3;
const POWER_CONTROL_DEEP_POWER_DOWN_EN: u32 = 1 << 2;
const POWER_CONTROL_POWER_DOWN_EN: u32 = 1 << 1;
const POWER_CONTROL_SELF_REFRESH_EN: u32 = 1 << 0;

pub const REFRESH_CONTROL3: usize = DDR_CFG_BASE + 0x0060;

const PSTAT: usize = DDR_CFG_BASE + 0x03fc;
const PORT_OFFSET: usize = 0x0490;
const PORT_CTRL_0_EN: usize = DDR_CFG_BASE + PORT_OFFSET + 0xb0 * 0;
const PORT_CTRL_1_EN: usize = DDR_CFG_BASE + PORT_OFFSET + 0xb0 * 1;
const PORT_CTRL_2_EN: usize = DDR_CFG_BASE + PORT_OFFSET + 0xb0 * 2;
const PORT_CTRL_3_EN: usize = DDR_CFG_BASE + PORT_OFFSET + 0xb0 * 3;

pub fn cvx16_polling_synp_normal_mode() {
    println!("/ polling_synp_normal_mode start");
    // synp ctrl operating_mode
    while let v = read32(DDR_CFG_BASE + 0x0004) & 0b111 {
        println!("  operating_mode {v}");
        if v == 1 {
            break;
        }
    }
    println!("\\ polling_synp_normal_mode finish");
}

pub fn cvx16_synp_mrw(addr: u32, data: u32) {
    // ZQCTL0.dis_auto_zq to 1.
    let v = read32(DDR_CFG_BASE + 0x180);
    let init_dis_auto_zq = if v >> 31 == 0 {
        write32(DDR_CFG_BASE + 0x180, v | (1 << 31));
        println!("    non-lp4 Write ZQCTL0.dis_auto_zq to 1");
        // opdelay(256);
        println!("  Wait tzqcs = 128 cycles");
        true
    } else {
        false
    };
    // Poll MRSTAT.mr_wr_busy until it is 0
    println!("  Poll MRSTAT.mr_wr_busy until it is 0");
    while read32(DDR_CFG_BASE + 0x18) & (1 << 0) != 0 {}
    println!("    non-lp4 Poll MRSTAT.mr_wr_busy finish");
    // Write the MRCTRL0.mr_type, MRCTRL0.mr_addr, MRCTRL0.mr_rank
    // and (for MRWs) MRCTRL1.mr_data
    // rddata[31:0]  = 0;
    // rddata[0]     = 0;       // mr_type  0:write   1:read
    // rddata[5:4]   = 1;       // mr_rank
    // rddata[15:12] = addr;    // mr_addr
    let v = (0b01 << 4) | ((addr & 0b1111) << 12);
    write32(DDR_CFG_BASE + 0x10, v);
    println!("    non-lp4 Write the MRCTRL0");
    // rddata[31:0] = 0;
    // rddata[15:0] = data;     // mr_data
    write32(DDR_CFG_BASE + 0x14, data & 0xffff);
    println!("    non-lp4 Write the MRCTRL1");
    // Write MRCTRL0.mr_wr to 1
    let v = read32(DDR_CFG_BASE + 0x10);
    write32(DDR_CFG_BASE + 0x10, v | (1 << 31));
    println!("    non-lp4 Write MRCTRL0.mr_wr to 1");
    if init_dis_auto_zq {
        // ZQCTL0.dis_auto_zq to 0.
        let v = read32(DDR_CFG_BASE + 0x180);
        write32(DDR_CFG_BASE + 0x180, v & !(1 << 31));
        println!("    non-lp4 Write ZQCTL0.dis_auto_zq to 0");
    }
}

pub fn pwrctl_init() -> (u32, u32, u32, u32) {
    // Disable all ports except 0
    write32(PORT_CTRL_1_EN, 0x0);
    write32(PORT_CTRL_2_EN, 0x0);
    write32(PORT_CTRL_3_EN, 0x0);
    // Poll rd_port_busy_n = 0 + wr_port_busy_n = 0
    while read32(PSTAT) != 0 {
        println!("  Poll PSTAT.rd_port_busy_n = 0");
    }

    let v = read32(POWER_CONTROL);
    // save for later
    let selfref_sw = (v >> 5) & 0b1;
    let en_dfi_dram_clk_disable = (v >> 3) & 0b1;
    let powerdown_en = (v >> 1) & 0b1;
    let selfref_en = v & 0b1;

    let v = v & !POWER_CONTROL_SELF_REFRESH_SW;
    let v = v & !POWER_CONTROL_DFI_DRAM_CLOCK_EN;
    // for non-mDDR/non-LPDDR2/non-LPDDR3,
    // this must not be set to 1
    // let v = v & !POWER_CONTROL_DEEP_POWER_DOWN_EN;
    // disable powerdown and self refresh
    let v = v & !POWER_CONTROL_POWER_DOWN_EN;
    let v = v & !POWER_CONTROL_SELF_REFRESH_EN;
    write32(POWER_CONTROL, v);

    (
        selfref_sw,
        en_dfi_dram_clk_disable,
        powerdown_en,
        selfref_en,
    )
}

pub fn pwrctl_restore(
    selfref_sw: u32,
    en_dfi_dram_clk_disable: u32,
    powerdown_en: u32,
    selfref_en: u32,
) {
    // dis_auto_refresh = 0
    // let v = read32(REFRESH_CONTROL3);
    // write32(REFRESH_CONTROL3, v & !(0b1));
    // restore powerdown_en, selfref_en
    let v = read32(POWER_CONTROL);
    let v = (v & !POWER_CONTROL_SELF_REFRESH_SW) | (selfref_sw << 5);
    let v = (v & !POWER_CONTROL_DFI_DRAM_CLOCK_EN) | (en_dfi_dram_clk_disable << 3);
    // deeppowerdown_en, non-mDDR/non-LPDDR2/non-LPDDR3,
    // this must not be set to 1
    // let v = v & !(1 << 2);
    let v = (v & !POWER_CONTROL_POWER_DOWN_EN) | (powerdown_en << 1);
    let v = v & !POWER_CONTROL_SELF_REFRESH_EN | selfref_en;
    write32(POWER_CONTROL, v);

    // Reenable ports 1-3
    write32(PORT_CTRL_1_EN, 1);
    write32(PORT_CTRL_2_EN, 1);
    write32(PORT_CTRL_3_EN, 1);
}

pub fn reset() {
    write32(DDRC_RESET, 0x0);
}

// plat/cv181x/ddr/ddr_config/ddr3_1866_x16/ddrc_init.c
pub fn init() {
    println!("DDRC init");
    let v = read32(DDR_CFG_BASE + 0xc);
    println!("DDRC 0x000c {v:08x}");
    // "ctcq" (qctc)
    write32(DDR_CFG_BASE + 0xc, 0x63746371);
    let v = read32(DDR_CFG_BASE + 0xc);
    println!("DDRC 0x000c {v:08x}");
    // PATCH0.use_blk_ext}:0:2:=0x1
    // PATCH0.dis_auto_ref_cnt_fix:2:1:=0x0
    // PATCH0.dis_auto_ref_algn_to_8:3:1:=0x0
    // PATCH0.starve_stall_at_dfi_ctrlupd:4:1:=0x1
    // PATCH0.starve_stall_at_abr:5:1:=0x1
    // PATCH0.dis_rdwr_switch_at_abr:6:1:=0x1
    // PATCH0.dfi_wdata_same_to_axi:7:1:=0x0
    // PATCH0.pagematch_limit_threshold:8:3=0x3
    // PATCH0.qos_sel:12:2:=0x2
    // PATCH0.burst_rdwr_xpi:16:4:=0x4
    // PATCH0.always_critical_when_urgent_hpr:20:1:=0x1
    // PATCH0.always_critical_when_urgent_lpr:21:1:=0x1
    // PATCH0.always_critical_when_urgent_wr:22:1:=0x1
    // PATCH0.disable_hif_rcmd_stall_path:24:1:=0x1
    // PATCH0.disable_hif_wcmd_stall_path:25:1:=0x1
    // PATCH0.derate_sys_en:29:1:=0x1
    // PATCH0.ref_4x_sys_high_temp:30:1:=0x1
    write32(DDR_CFG_BASE + 0x044, 0x00000000);
    // PATCH1.ref_adv_stop_threshold:0:7:=0x0
    // PATCH1.ref_adv_dec_threshold:8:7:=0x0
    // PATCH1.ref_adv_max:16:7:=0x0
    write32(DDR_CFG_BASE + 0x148, 0x999F0000);
    // PATCH4.t_phyd_rden:16:6=0x0
    // PATCH4.phyd_rd_clk_stop:23:1=0x0
    // PATCH4.t_phyd_wren:24:6=0x0
    // PATCH4.phyd_wr_clk_stop:31:1=0x0
    // auto gen.
    write32(DDR_CFG_BASE + 0x0, 0x81041401);
    write32(DDR_CFG_BASE + 0x30, 0x00000000);
    write32(DDR_CFG_BASE + 0x34, 0x00930001);
    write32(DDR_CFG_BASE + 0x38, 0x00020000);
    write32(DDR_CFG_BASE + 0x50, 0x00201070);
    write32(DDR_CFG_BASE + 0x60, 0x00000000);
    write32(DDR_CFG_BASE + 0x64, 0x007100A4);
    write32(DDR_CFG_BASE + 0xc0, 0x00000000);
    write32(DDR_CFG_BASE + 0xc4, 0x00000000);
    if DDR_INIT_SPEED_UP {
        write32(DDR_CFG_BASE + 0xd0, 0x00010002);
        write32(DDR_CFG_BASE + 0xd4, 0x00020000);
    } else {
        write32(DDR_CFG_BASE + 0xd0, 0x000100E5);
        write32(DDR_CFG_BASE + 0xd4, 0x006A0000);
    }
    write32(DDR_CFG_BASE + 0xdc, 0x1F140040);
    if DDR_DODT {
        write32(DDR_CFG_BASE + 0xe0, 0x04600000);
    } else {
        write32(DDR_CFG_BASE + 0xe0, 0x00600000);
    }
    write32(DDR_CFG_BASE + 0x0e4, 0x000B03BF);
    write32(DDR_CFG_BASE + 0x100, 0x0E111F10);
    write32(DDR_CFG_BASE + 0x104, 0x00030417);
    write32(DDR_CFG_BASE + 0x108, 0x0507060A);
    write32(DDR_CFG_BASE + 0x10c, 0x00002007);
    write32(DDR_CFG_BASE + 0x110, 0x07020307);
    write32(DDR_CFG_BASE + 0x114, 0x05050303);
    write32(DDR_CFG_BASE + 0x120, 0x00000907);
    write32(DDR_CFG_BASE + 0x13c, 0x00000000);
    write32(DDR_CFG_BASE + 0x180, 0xC0960026);
    write32(DDR_CFG_BASE + 0x184, 0x00000001);
    // phyd related
    write32(DDR_CFG_BASE + 0x190, 0x048a8305);
    // DFITMG0.dfi_t_ctrl_delay:24:5:=0x4
    // DFITMG0.dfi_rddata_use_dfi_phy_clk:23:1:=0x1
    // DFITMG0.dfi_t_rddata_en:16:7:=0xa
    // DFITMG0.dfi_wrdata_use_dfi_phy_clk:15:1:=0x1
    // DFITMG0.dfi_tphy_wrdata:8:6:=0x3
    // DFITMG0.dfi_tphy_wrlat:0:6:=0x5
    write32(DDR_CFG_BASE + 0x194, 0x00070202);
    // DFITMG1.dfi_t_cmd_lat:28:4:=0x0
    // DFITMG1.dfi_t_parin_lat:24:2:=0x0
    // DFITMG1.dfi_t_wrdata_delay:16:5:=0x7
    // DFITMG1.dfi_t_dram_clk_disable:8:5:=0x2
    // DFITMG1.dfi_t_dram_clk_enable:0:5:=0x2
    write32(DDR_CFG_BASE + 0x198, 0x07c13121);
    // DFILPCFG0.dfi_tlp_resp:24:5:=0x7
    // DFILPCFG0.dfi_lp_wakeup_dpd:20:4:=0xc
    // DFILPCFG0.dfi_lp_en_dpd:16:1:=0x1
    // DFILPCFG0.dfi_lp_wakeup_sr:12:4:=0x3
    // DFILPCFG0.dfi_lp_en_sr:8:1:=0x1
    // DFILPCFG0.dfi_lp_wakeup_pd:4:4:=0x2
    // DFILPCFG0.dfi_lp_en_pd:0:1:=0x1
    write32(DDR_CFG_BASE + 0x19c, 0x00000021);
    // DFILPCFG1.dfi_lp_wakeup_mpsm:4:4:=0x2
    // DFILPCFG1.dfi_lp_en_mpsm:0:1:=0x1
    // auto gen.
    write32(DDR_CFG_BASE + 0x1a0, 0xC0400018);
    write32(DDR_CFG_BASE + 0x1a4, 0x00FE00FF);
    write32(DDR_CFG_BASE + 0x1a8, 0x80000000);
    write32(DDR_CFG_BASE + 0x1b0, 0x000002C1);
    write32(DDR_CFG_BASE + 0x1c0, 0x00000001);
    write32(DDR_CFG_BASE + 0x1c4, 0x00000001);
    // address map, auto gen.
    write32(DDR_CFG_BASE + 0x200, 0x00001F1F);
    write32(DDR_CFG_BASE + 0x204, 0x00070707);
    write32(DDR_CFG_BASE + 0x208, 0x00000000);
    write32(DDR_CFG_BASE + 0x20c, 0x1F000000);
    write32(DDR_CFG_BASE + 0x210, 0x00001F1F);
    write32(DDR_CFG_BASE + 0x214, 0x060F0606);
    write32(DDR_CFG_BASE + 0x218, 0x06060606);
    write32(DDR_CFG_BASE + 0x21c, 0x00000606);
    write32(DDR_CFG_BASE + 0x220, 0x00003F3F);
    write32(DDR_CFG_BASE + 0x224, 0x06060606);
    write32(DDR_CFG_BASE + 0x228, 0x06060606);
    write32(DDR_CFG_BASE + 0x22c, 0x001F1F06);
    // auto gen.
    write32(DDR_CFG_BASE + 0x240, 0x08000610);
    if DDR_DODT {
        write32(DDR_CFG_BASE + 0x244, 0x00000001);
    } else {
        write32(DDR_CFG_BASE + 0x244, 0x00000000);
    }
    write32(DDR_CFG_BASE + 0x250, 0x00003F85);
    // SCHED.opt_vprw_sch:31:1:=0x0
    // SCHED.rdwr_idle_gap:24:7:=0x0
    // SCHED.go2critical_hysteresis:16:8:=0x0
    // SCHED.lpddr4_opt_act_timing:15:1:=0x0
    // SCHED.lpr_num_entries:8:7:=0x1f
    // SCHED.autopre_rmw:7:1:=0x1
    // SCHED.dis_opt_ntt_by_pre:6:1:=0x0
    // SCHED.dis_opt_ntt_by_act:5:1:=0x0
    // SCHED.opt_wrcam_fill_level:4:1:=0x0
    // SCHED.rdwr_switch_policy_sel:3:1:=0x0
    // SCHED.pageclose:2:1:=0x1
    // SCHED.prefer_write:1:1:=0x0
    // SCHED.dis_opt_wrecc_collision_flush:0:1:=0x1
    write32(DDR_CFG_BASE + 0x254, 0x00000000);
    // SCHED1.page_hit_limit_rd:28:3:=0x0
    // SCHED1.page_hit_limit_wr:24:3:=0x0
    // SCHED1.visible_window_limit_rd:20:3:=0x0
    // SCHED1.visible_window_limit_wr:16:3:=0x0
    // SCHED1.delay_switch_write:12:4:=0x0
    // SCHED1.pageclose_timer:0:8:=0x0
    // auto gen.
    write32(DDR_CFG_BASE + 0x25c, 0x100000F0);
    // PERFHPR1.hpr_xact_run_length:24:8:=0x20
    // PERFHPR1.hpr_max_starve:0:16:=0x6a
    write32(DDR_CFG_BASE + 0x264, 0x100000F0);
    // PERFLPR1.lpr_xact_run_length:24:8:=0x20
    // PERFLPR1.lpr_max_starve:0:16:=0x6a
    write32(DDR_CFG_BASE + 0x26c, 0x100000F0);
    // PERFWR1.w_xact_run_length:24:8:=0x20
    // PERFWR1.w_max_starve:0:16:=0x1a8
    write32(DDR_CFG_BASE + 0x300, 0x00000000);
    // DBG0.dis_max_rank_wr_opt:7:1:=0x0
    // DBG0.dis_max_rank_rd_opt:6:1:=0x0
    // DBG0.dis_collision_page_opt:4:1:=0x0
    // DBG0.dis_act_bypass:2:1:=0x0
    // DBG0.dis_rd_bypass:1:1:=0x0
    // DBG0.dis_wc:0:1:=0x0
    write32(DDR_CFG_BASE + 0x304, 0x00000000);
    // DBG1.dis_hif:1:1:=0x0
    // DBG1.dis_dq:0:1:=0x0
    write32(DDR_CFG_BASE + 0x30c, 0x00000000);
    write32(DDR_CFG_BASE + 0x320, 0x00000001);
    // SWCTL.sw_done:0:1:=0x1
    write32(DDR_CFG_BASE + 0x36c, 0x00000000);
    // POISONCFG.rd_poison_intr_clr:24:1:=0x0
    // POISONCFG.rd_poison_intr_en:20:1:=0x0
    // POISONCFG.rd_poison_slverr_en:16:1:=0x0
    // POISONCFG.wr_poison_intr_clr:8:1:=0x0
    // POISONCFG.wr_poison_intr_en:4:1:=0x0
    // POISONCFG.wr_poison_slverr_en:0:1:=0x0
    write32(DDR_CFG_BASE + 0x400, 0x00000011);
    // PCCFG.dch_density_ratio:12:2:=0x0
    // PCCFG.bl_exp_mode:8:1:=0x0
    // PCCFG.pagematch_limit:4:1:=0x1
    // PCCFG.go2critical_en:0:1:=0x1
    write32(DDR_CFG_BASE + 0x404, 0x00006000);
    // PCFGR_0.rdwr_ordered_en:16:1:=0x0
    // PCFGR_0.rd_port_pagematch_en:14:1:=0x1
    // PCFGR_0.rd_port_urgent_en:13:1:=0x1
    // PCFGR_0.rd_port_aging_en:12:1:=0x0
    // PCFGR_0.read_reorder_bypass_en:11:1:=0x0
    // PCFGR_0.rd_port_priority:0:10:=0x0
    write32(DDR_CFG_BASE + 0x408, 0x00006000);
    // PCFGW_0.wr_port_pagematch_en:14:1:=0x1
    // PCFGW_0.wr_port_urgent_en:13:1:=0x1
    // PCFGW_0.wr_port_aging_en:12:1:=0x0
    // PCFGW_0.wr_port_priority:0:10:=0x0
    write32(PORT_CTRL_0_EN, 0x00000001);
    write32(DDR_CFG_BASE + 0x494, 0x00000007);
    // PCFGQOS0_0.rqos_map_region2:24:8:=0x0
    // PCFGQOS0_0.rqos_map_region1:20:4:=0x0
    // PCFGQOS0_0.rqos_map_region0:16:4:=0x0
    // PCFGQOS0_0.rqos_map_level2:8:8:=0x0
    // PCFGQOS0_0.rqos_map_level1:0:8:=0x7
    write32(DDR_CFG_BASE + 0x498, 0x0000006a);
    // PCFGQOS1_0.rqos_map_timeoutr:16:16:=0x0
    // PCFGQOS1_0.rqos_map_timeoutb:0:16:=0x6a
    write32(DDR_CFG_BASE + 0x49c, 0x00000e07);
    // PCFGWQOS0_0.wqos_map_region2:24:8:=0x0
    // PCFGWQOS0_0.wqos_map_region1:20:4:=0x0
    // PCFGWQOS0_0.wqos_map_region0:16:4:=0x0
    // PCFGWQOS0_0.wqos_map_level2:8:8:=0xe
    // PCFGWQOS0_0.wqos_map_level1:0:8:=0x7
    write32(DDR_CFG_BASE + 0x4a0, 0x01a801a8);
    // PCFGWQOS1_0.wqos_map_timeout2:16:16:=0x1a8
    // PCFGWQOS1_0.wqos_map_timeout1:0:16:=0x1a8
    write32(DDR_CFG_BASE + 0x4b4, 0x00006000);
    // PCFGR_1.rdwr_ordered_en:16:1:=0x0
    // PCFGR_1.rd_port_pagematch_en:14:1:=0x1
    // PCFGR_1.rd_port_urgent_en:13:1:=0x1
    // PCFGR_1.rd_port_aging_en:12:1:=0x0
    // PCFGR_1.read_reorder_bypass_en:11:1:=0x0
    // PCFGR_1.rd_port_priority:0:10:=0x0
    write32(DDR_CFG_BASE + 0x4b8, 0x00006000);
    // PCFGW_1.wr_port_pagematch_en:14:1:=0x1
    // PCFGW_1.wr_port_urgent_en:13:1:=0x1
    // PCFGW_1.wr_port_aging_en:12:1:=0x0
    // PCFGW_1.wr_port_priority:0:10:=0x0
    write32(PORT_CTRL_1_EN, 0x00000001);
    write32(DDR_CFG_BASE + 0x544, 0x00000007);
    // PCFGQOS0_1.rqos_map_region2:24:8:=0x0
    // PCFGQOS0_1.rqos_map_region1:20:4:=0x0
    // PCFGQOS0_1.rqos_map_region0:16:4:=0x0
    // PCFGQOS0_1.rqos_map_level2:8:8:=0x0
    // PCFGQOS0_1.rqos_map_level1:0:8:=0x7
    write32(DDR_CFG_BASE + 0x548, 0x0000006a);
    // PCFGQOS1_1.rqos_map_timeoutr:16:16:=0x0
    // PCFGQOS1_1.rqos_map_timeoutb:0:16:=0x6a
    write32(DDR_CFG_BASE + 0x54c, 0x00000e07);
    // PCFGWQOS0_1.wqos_map_region2:24:8:=0x0
    // PCFGWQOS0_1.wqos_map_region1:20:4:=0x0
    // PCFGWQOS0_1.wqos_map_region0:16:4:=0x0
    // PCFGWQOS0_1.wqos_map_level2:8:8:=0xe
    // PCFGWQOS0_1.wqos_map_level1:0:8:=0x7
    write32(DDR_CFG_BASE + 0x550, 0x01a801a8);
    // PCFGWQOS1_1.wqos_map_timeout2:16:16:=0x1a8
    // PCFGWQOS1_1.wqos_map_timeout1:0:16:=0x1a8
    write32(DDR_CFG_BASE + 0x564, 0x00006000);
    // PCFGR_2.rdwr_ordered_en:16:1:=0x0
    // PCFGR_2.rd_port_pagematch_en:14:1:=0x1
    // PCFGR_2.rd_port_urgent_en:13:1:=0x1
    // PCFGR_2.rd_port_aging_en:12:1:=0x0
    // PCFGR_2.read_reorder_bypass_en:11:1:=0x0
    // PCFGR_2.rd_port_priority:0:10:=0x0
    write32(DDR_CFG_BASE + 0x568, 0x00006000);
    // PCFGW_2.wr_port_pagematch_en:14:1:=0x1
    // PCFGW_2.wr_port_urgent_en:13:1:=0x1
    // PCFGW_2.wr_port_aging_en:12:1:=0x0
    // PCFGW_2.wr_port_priority:0:10:=0x0
    write32(PORT_CTRL_2_EN, 0x00000001);
    write32(DDR_CFG_BASE + 0x5f4, 0x00000007);
    // PCFGQOS0_2.rqos_map_region2:24:8:=0x0
    // PCFGQOS0_2.rqos_map_region1:20:4:=0x0
    // PCFGQOS0_2.rqos_map_region0:16:4:=0x0
    // PCFGQOS0_2.rqos_map_level2:8:8:=0x0
    // PCFGQOS0_2.rqos_map_level1:0:8:=0x7
    write32(DDR_CFG_BASE + 0x5f8, 0x0000006a);
    // PCFGQOS1_2.rqos_map_timeoutr:16:16:=0x0
    // PCFGQOS1_2.rqos_map_timeoutb:0:16:=0x6a
    write32(DDR_CFG_BASE + 0x5fc, 0x00000e07);
    // PCFGWQOS0_2.wqos_map_region2:24:8:=0x0
    // PCFGWQOS0_2.wqos_map_region1:20:4:=0x0
    // PCFGWQOS0_2.wqos_map_region0:16:4:=0x0
    // PCFGWQOS0_2.wqos_map_level2:8:8:=0xe
    // PCFGWQOS0_2.wqos_map_level1:0:8:=0x7
    write32(DDR_CFG_BASE + 0x600, 0x01a801a8);
    // PCFGWQOS1_2.wqos_map_timeout2:16:16:=0x1a8
    // PCFGWQOS1_2.wqos_map_timeout1:0:16:=0x1a8
}

pub fn high_patch() {
    // enable auto PD/SR
    write32(DDR_CFG_BASE + 0x0030, 0x00000002);
    // enable auto ctrl_upd
    write32(DDR_CFG_BASE + 0x01a0, 0x00400018);
    // enable clock gating
    write32(CLOCK_GATING_CONTROL, 0x00000000);

    // change XPI to multi DDR burst
    // write32(DDR_CFG_BASE + 0x000c, 0x63786370);
    // cv180x only
    // write32(DDR_CFG_BASE + 0x000c, 0x63746371);
    // write32(CLOCK_GATING_ENABLE, 0x08000000);
}

pub fn low_patch() {
    // disable auto PD/SR
    write32(DDR_CFG_BASE + 0x0030, 0x00000000);
    // disable auto ctrl_upd
    write32(DDR_CFG_BASE + 0x01a0, 0xC0400018);
    // disable clock gating
    write32(CLOCK_GATING_CONTROL, 0x00000fff);

    // change XPI to single DDR burst
    // write32(DDR_CFG_BASE + 0x000c, 0x63746371);
    // cv180x only
    // write32(DDR_CFG_BASE + 0x0044, 0x14000000);
}

pub fn update_by_dram_size(size: u32) {
    let v = read32(DDR_CFG_BASE + 0x0);
    let s1 = (v >> 12) & 0b11;
    let s2 = (v >> 30) & 0b11;
    println!("   DRAM cap shift vals: x16 {s1}, dev {s2}");
    // DRAM cap in megabytes per cap
    let dram_cap_in_mbyte = size;
    // change sys cap to x16 cap
    let dram_cap_in_mbyte = dram_cap_in_mbyte >> (1 - s1);
    // change x16 cap to device cap
    let dram_cap_in_mbyte = dram_cap_in_mbyte >> (2 - s2);
    println!("   DRAM cap in MB per dev: {dram_cap_in_mbyte}");
    match dram_cap_in_mbyte {
        5 => {
            write32(DDR_CFG_BASE + 0x64, 0x00510019);
            write32(DDR_CFG_BASE + 0x100, 0x0B011610);
            write32(DDR_CFG_BASE + 0x120, 0x00000502);

            write32(DDR_CFG_BASE + 0x200, 0x00001F1F);
            write32(DDR_CFG_BASE + 0x204, 0x003F0606);
            write32(DDR_CFG_BASE + 0x208, 0x00000000);
            write32(DDR_CFG_BASE + 0x20c, 0x1F1F0000);
            write32(DDR_CFG_BASE + 0x210, 0x00001F1F);
            write32(DDR_CFG_BASE + 0x214, 0x040F0404);
            write32(DDR_CFG_BASE + 0x218, 0x04040404);
            write32(DDR_CFG_BASE + 0x21c, 0x00000404);
            write32(DDR_CFG_BASE + 0x220, 0x00003F3F);
            write32(DDR_CFG_BASE + 0x224, 0x04040404);
            write32(DDR_CFG_BASE + 0x228, 0x04040404);
            write32(DDR_CFG_BASE + 0x22c, 0x001F1F04);
        }
        6 => {
            write32(DDR_CFG_BASE + 0x64, 0x0071002A);
            write32(DDR_CFG_BASE + 0x120, 0x00000903);
        }
        7 => {
            write32(DDR_CFG_BASE + 0x64, 0x00710034);
            write32(DDR_CFG_BASE + 0x120, 0x00000903);
        }
        8 => {
            write32(DDR_CFG_BASE + 0x64, 0x0071004B);
            write32(DDR_CFG_BASE + 0x120, 0x00000904);
        }
        9 => {
            write32(DDR_CFG_BASE + 0x64, 0x0071007A);
            write32(DDR_CFG_BASE + 0x120, 0x00000905);
        }
        10 => {
            write32(DDR_CFG_BASE + 0x64, 0x007100A4);
            write32(DDR_CFG_BASE + 0x120, 0x00000907);
        }
        _ => {
            // not supposed to happen, but you never know...
            println!("  WARNING: unsupported DRAM cap: {dram_cap_in_mbyte} MB per dev");
        }
    }
    // toggle refresh_update_level
    write32(REFRESH_CONTROL3, 0x00000002);
    write32(REFRESH_CONTROL3, 0x00000000);
}
