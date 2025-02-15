pub const SEC_SUBSYS_BASE: usize = 0x0200_0000;

pub const TOP_BASE: usize = 0x0300_0000;
pub const TOP_MISC: usize = TOP_BASE;
// TRM alpha p62
pub const CLK_GEN_PLL_CTRL_BASE: usize = TOP_BASE + 0x2000;

pub const EFUSE: usize = TOP_BASE + 0x0005_0000;

pub const RTC_SYS_BASE: usize = 0x0500_0000;

// NOTE: The vendor code has PHYD_BASE and PHYD_BASE_ADDR.
// Those are not the same, looks like there was some confusion.
// PHYD_BASE is really PHYD_APB. They also have that. We reduced it.
// What is PHYD anyway?
// TODO: What is the difference between PHY_BASE and PHY_BASE_ADDR?
// plat/cv181x/include/ddr/ddr_sys.h
pub const DDR_SYS_BASE: usize = 0x0800_0000;
pub const PI_BASE: usize = DDR_SYS_BASE; // same as PHYD_BASE_ADDR ... unused?
pub const PHYD_BASE_ADDR: usize = DDR_SYS_BASE; // ?? used in phy_init
pub const PHY_BASE: usize = DDR_SYS_BASE + 0x2000;
pub const PHY_VERSION: usize = DDR_SYS_BASE + 0x3000;
pub const DDR_CFG_BASE: usize = DDR_SYS_BASE + 0x4000;
pub const PHYD_APB: usize = DDR_SYS_BASE + 0x6000;
pub const AXI_MON_BASE: usize = DDR_SYS_BASE + 0x8000;
pub const DDR_TOP_BASE: usize = DDR_SYS_BASE + 0xa000;
pub const DDR_BIST_BASE: usize = DDR_SYS_BASE + 0x0001_0000;

pub const TPU_SRAM_BASE: usize = 0x0c00_0000;

pub const AXI_SRAM_BASE: usize = 0x0e00_0000;

pub const DRAM_BASE: usize = 0x8000_0000;

// mask ROM: 64k (CV1800B) or 128k (SG200x)
pub const MASK_ROM_BASE: usize = 0x0440_0000;
// The mask ROM provides us with helper functions.
// plat/cv180x/include/riscv/rom_api_refer.h
// plat/cv181x/include/riscv/rom_api_refer.h
#[cfg(soc = "CV1800B")]
pub const MASK_ROM_FN_BASE: usize = MASK_ROM_BASE;
// On later SoCs, the mask ROM functions are off
#[cfg(not(soc = "CV1800B"))]
pub const MASK_ROM_FN_BASE: usize = MASK_ROM_BASE + 0x0001_8000;
