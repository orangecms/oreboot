// Rockchip often has multiple blocks per peripheral, some called "S" ("Secure")
// and others "NS" ("Non-Secure"). See the GRF section in the RK3288 manual:
// https://www.rockchip.fr/Rockchip%20RK3288%20TRM%20V1.0%20Part%201-System%20and%20System%20Control.pdf
// > The general register file will be used to do static set by software, which
// > is composed of many registers for system control. The GRF is divided into
// > two sections, one is GRF for non-secure system, the other is SGRF for
// > secure system.

pub const PMU_SGRF_BASE: usize = 0xFDC0_0000;
pub const PMU_GRF_BASE: usize = 0xFDC2_0000;
pub const DDR_GRF_BASE: usize = 0xFDC4_0000;
pub const SYS_GRF_BASE: usize = 0xFDC6_0000;
pub const SRAM_BASE: usize = 0xFDCC_0000;
// always on system
pub const PMU_CRU_BASE: usize = 0xFDD0_0000;
// "general secure system except always on system"
pub const CRU_S_BASE: usize = 0xFDD1_0000;
// SGRF: ("Secure") General Register File
// https://www.kernel.org/doc/Documentation/devicetree/bindings/soc/rockchip/grf.txt
pub const SYS_SGRF_BASE: usize = 0xFDD1_8000;
pub const TIMER_BASE: usize = 0xFDD1_C000;
// general system except always on system
pub const CRU_NS_BASE: usize = 0xFDD2_0000;
pub const I2C0_BASE: usize = 0xFDD4_0000;
pub const UART0_BASE: usize = 0xFDD5_0000;

pub const HDCP_KEY: usize = 0xFE0D_0000;
pub const RESX_BASE: usize = 0xFE10_0000;
pub const UPCTL2_BASE: usize = 0xFE25_0000;
pub const OTP_NS_BASE: usize = 0xFE38_C000;
pub const UART1_BASE: usize = 0xFE65_0000;
pub const UART2_BASE: usize = 0xFE66_0000;
pub const OTP_S_BASE: usize = 0xFE3A_0000;

pub const DDR_PHY_BASE: usize = 0xFE80_0000;
pub const OTP_PHY_BASE: usize = 0xFE88_0000;

pub const MASK_ROM_BASE: usize = 0xFFFF_0000;
