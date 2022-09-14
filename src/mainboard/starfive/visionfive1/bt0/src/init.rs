use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

const UART3_BASE: u32 = 0x1244_0000;

const REG_THR: u32 = UART3_BASE + 0x00; /* Transmitter holding reg. */
const REG_RDR: u32 = UART3_BASE + 0x00; /* Receiver data reg.       */
const REG_BRDL: u32 = UART3_BASE + 0x00; /* Baud rate divisor (LSB)  */
const REG_BRDH: u32 = UART3_BASE + 0x01; /* Baud rate divisor (MSB)  */
const REG_IER: u32 = UART3_BASE + 0x01; /* Interrupt enable reg.    */
const REG_IIR: u32 = UART3_BASE + 0x02; /* Interrupt ID reg.        */
const REG_FCR: u32 = UART3_BASE + 0x02; /* FIFO control reg.        */
const REG_LCR: u32 = UART3_BASE + 0x03; /* Line control reg.        */
const REG_MDC: u32 = UART3_BASE + 0x04; /* Modem control reg.       */
const REG_LSR: u32 = UART3_BASE + 0x05; /* Line status reg.         */
const REG_MSR: u32 = UART3_BASE + 0x06; /* Modem status reg.        */
const REG_DLF: u32 = UART3_BASE + 0xC0; /* Divisor Latch Fraction   */

fn write_32(reg: u32, val: u32) {
    unsafe {
        write_volatile(reg as *mut u32, val);
    }
}

fn read_32(reg: u32) -> u32 {
    unsafe { read_volatile(reg as *mut u32) }
}

/* constants for line control register */

const LCR_CS5: u8 = 0x00; /* 5 bits data size */
const LCR_CS6: u8 = 0x01; /* 6 bits data size */
const LCR_CS7: u8 = 0x02; /* 7 bits data size */
const LCR_CS8: u8 = 0x03; /* 8 bits data size */
const LCR_2_STB: u8 = 0x04; /* 2 stop bits */
const LCR_1_STB: u8 = 0x00; /* 1 stop bit */
const LCR_PEN: u8 = 0x08; /* parity enable */
const LCR_PDIS: u8 = 0x00; /* parity disable */
const LCR_EPS: u8 = 0x10; /* even parity select */
const LCR_SP: u8 = 0x20; /* stick parity select */
const LCR_SBRK: u8 = 0x40; /* break control bit */
const LCR_DLAB: u8 = 0x80; /* divisor latch access enable */

/* constants for line status register */

const LSR_RXRDY: u8 = 0x01; /* receiver data available */
const LSR_OE: u8 = 0x02; /* overrun error */
const LSR_PE: u8 = 0x04; /* parity error */
const LSR_FE: u8 = 0x08; /* framing error */
const LSR_BI: u8 = 0x10; /* break interrupt */
const LSR_EOB_MASK: u8 = 0x1E; /* Error or Break mask */
const LSR_THRE: u8 = 0x20; /* transmit holding register empty */
const LSR_TEMT: u8 = 0x40; /* transmitter empty */

/* equates for FIFO control register */

const FCR_FIFO: u8 = 0x01; /* enable XMIT and RCVR FIFO */
const FCR_RCVRCLR: u8 = 0x02; /* clear RCVR FIFO */
const FCR_XMITCLR: u8 = 0x04; /* clear XMIT FIFO */

const FCR_MODE0: u8 = 0x00; /* set receiver in mode 0 */
const FCR_MODE1: u8 = 0x08; /* set receiver in mode 1 */

/* RCVR FIFO interrupt levels: trigger interrupt with this bytes in FIFO */
const FCR_FIFO_1: u8 = 0x00; /* 1 byte in RCVR FIFO */
const FCR_FIFO_4: u8 = 0x40; /* 4 bytes in RCVR FIFO */
const FCR_FIFO_8: u8 = 0x80; /* 8 bytes in RCVR FIFO */
const FCR_FIFO_14: u8 = 0xC0; /* 14 bytes in RCVR FIFO */

const UART_CLK: u32 = 100_000_000;
const UART_BAUDRATE_32MCLK_115200: u32 = 115200;

pub fn uart_write(c: char) {
    unsafe {
        /*
        loop {
            let lsr = read_32(REG_LSR) as u8 & LSR_THRE;
            if lsr != 0 {
                break;
            }
        }
        */
        write_volatile(UART3_BASE as *mut u32, c as u32);
    }
}

fn write8(reg: u32, val: u8) {
    unsafe {
        write_volatile(reg as *mut u8, val);
    }
}

pub fn uart_init() {
    let divisor = (UART_CLK / UART_BAUDRATE_32MCLK_115200) >> 4;

    let lcr_cache = read_32(REG_LCR);
    write8(REG_LCR, LCR_DLAB | lcr_cache as u8);
    write8(REG_BRDL, divisor as u8);
    write8(REG_BRDH, (divisor >> 8) as u8);

    /* restore the DLAB to access the baud rate divisor registers */
    write8(REG_LCR, lcr_cache as u8);
    /* 8 data bits, 1 stop bit, no parity, clear DLAB */
    write8(REG_LCR, (LCR_CS8 | LCR_1_STB | LCR_PDIS) as u8);

    write8(REG_MDC, 0); /*disable flow control*/

    /*
     * Program FIFO: enabled, mode 0 (set for compatibility with quark),
     * generate the interrupt at 8th byte
     * Clear TX and RX FIFO
     */
    write8(
        REG_FCR,
        (FCR_FIFO | FCR_MODE1 | /*FCR_FIFO_1*/FCR_FIFO_8 | FCR_RCVRCLR | FCR_XMITCLR) as u8,
    );

    write8(REG_IER, 0); // disable the serial interrupt
}

pub const CLKGEN_BASE: u32 = 0x1180_0000;
pub const CLK_CPUNDBUS_ROOT_CTRL: u32 = CLKGEN_BASE + 0x0;
pub const CLK_DLA_ROOT_CTRL: u32 = CLKGEN_BASE + 0x4;
pub const CLK_DSP_ROOT_CTRL: u32 = CLKGEN_BASE + 0x8;
pub const CLK_GMACUSB_ROOT_CTRL: u32 = CLKGEN_BASE + 0xC;
pub const CLK_PERH0_ROOT_CTRL: u32 = CLKGEN_BASE + 0x10;

pub fn clk_cpundbus_root_pll0_out() {
    let v = read_32(CLK_CPUNDBUS_ROOT_CTRL) & !(0x3 << 24);
    write_32(CLK_CPUNDBUS_ROOT_CTRL, v | 1 << 24);
}

pub fn clk_dla_root_pll1_out() {
    let v = read_32(CLK_DLA_ROOT_CTRL) & !(0x3 << 24);
    write_32(CLK_DLA_ROOT_CTRL, v | 1 << 24);
}

pub fn clk_dsp_root_pll2_out() {
    let v = read_32(CLK_DSP_ROOT_CTRL) & !(0x3 << 24);
    write_32(CLK_DSP_ROOT_CTRL, v | 3 << 24);
}

pub fn clk_perh0_root_pll0_out() {
    let v = read_32(CLK_PERH0_ROOT_CTRL) & !(0x1 << 24);
    write_32(CLK_PERH0_ROOT_CTRL, v | 1 << 24);
}

fn init_coreclk() {
    // TODO: make base a parameter.
    clk_cpundbus_root_pll0_out();
    clk_dla_root_pll1_out();
    clk_dsp_root_pll2_out();
    clk_perh0_root_pll0_out();

    // not enabled in original.
    // slow down nne bus can fix nne50 & vp6 ram scan issue,
    // as well as vin_subsys reg scan issue.
    //	clk_nne_bus_cpu_axi_;
}

pub fn clock_init() {
    // Update the peripheral clock dividers of UART, SPI and I2C to safe
    // values as we can't put them in reset before changing frequency.
    /*
    let hfclk = 1_000_000_000; // 1GHz
    let clks = [];
    for clk in clks.iter_mut() {
        if false {
            clk.set_clock_rate(hfclk);
        }
    }
    */

    init_coreclk();

    // These take like 16 cycles to actually propagate. We can't go sending
    // stuff before they come out of reset. So wait.
    // TODO: Add a register to read the current reset states, or DDR Control
    // device?
    for _ in 0..=255 { /* nop */ }
    // self.init_pll_ge();
    //        self.dev_reset
    //            .set(reset_mask(false, false, false, false, false));

    unsafe { asm!("fence") };
}

pub const SYSCON_SYSMAIN_CTRL_BASE: u32 = 0x00_1185_0000;
pub const SYSCON_SYSMAIN_CTRL28: u32 = SYSCON_SYSMAIN_CTRL_BASE + 0x70;
pub fn syscon_gmac_phy_intf_sel(v: u32) {
    let nv = read_32(SYSCON_SYSMAIN_CTRL28) & !(0x7);
    write_32(SYSCON_SYSMAIN_CTRL28, nv | (v & 0x7));
}

pub const SYSCON_IOPAD_CTRL_BASE: u32 = 0x00_1185_8000;
pub const SYSCON_IOPAD_CTRL32: u32 = SYSCON_IOPAD_CTRL_BASE + 0x80;
pub const SYSCON_IOPAD_CTRL33: u32 = SYSCON_IOPAD_CTRL_BASE + 0x84;
pub const SYSCON_IOPAD_CTRL34: u32 = SYSCON_IOPAD_CTRL_BASE + 0x88;
pub const SYSCON_IOPAD_CTRL35: u32 = SYSCON_IOPAD_CTRL_BASE + 0x8c;
pub const SYSCON_IOPAD_CTRL38: u32 = SYSCON_IOPAD_CTRL_BASE + 0x98;
pub const SYSCON_IOPAD_CTRL39: u32 = SYSCON_IOPAD_CTRL_BASE + 0x9C;
pub const SYSCON_IOPAD_CTRL50: u32 = SYSCON_IOPAD_CTRL_BASE + 0xC8;
pub const SYSCON_IOPAD_CTRL89: u32 = SYSCON_IOPAD_CTRL_BASE + 0x164;
pub const SYSCON_IOPAD_CTRL90: u32 = SYSCON_IOPAD_CTRL_BASE + 0x168;
pub const SYSCON_IOPAD_CTRL91: u32 = SYSCON_IOPAD_CTRL_BASE + 0x16C;
pub const SYSCON_IOPAD_CTRL92: u32 = SYSCON_IOPAD_CTRL_BASE + 0x170;
pub const SYSCON_IOPAD_CTRL93: u32 = SYSCON_IOPAD_CTRL_BASE + 0x174;
pub const SYSCON_IOPAD_CTRL94: u32 = SYSCON_IOPAD_CTRL_BASE + 0x178;
pub const SYSCON_IOPAD_CTRL95: u32 = SYSCON_IOPAD_CTRL_BASE + 0x17C;
pub const SYSCON_IOPAD_CTRL96: u32 = SYSCON_IOPAD_CTRL_BASE + 0x180;
pub const SYSCON_IOPAD_CTRL97: u32 = SYSCON_IOPAD_CTRL_BASE + 0x184;
pub const SYSCON_IOPAD_CTRL98: u32 = SYSCON_IOPAD_CTRL_BASE + 0x188;
pub const SYSCON_IOPAD_CTRL99: u32 = SYSCON_IOPAD_CTRL_BASE + 0x18C;
pub const SYSCON_IOPAD_CTRL100: u32 = SYSCON_IOPAD_CTRL_BASE + 0x190;
pub const SYSCON_IOPAD_CTRL101: u32 = SYSCON_IOPAD_CTRL_BASE + 0x194;
pub const SYSCON_IOPAD_CTRL102: u32 = SYSCON_IOPAD_CTRL_BASE + 0x198;
pub const SYSCON_IOPAD_CTRL103: u32 = SYSCON_IOPAD_CTRL_BASE + 0x19C;
pub const SYSCON_IOPAD_CTRL104: u32 = SYSCON_IOPAD_CTRL_BASE + 0x1A0;

pub fn syscon_io_padshare_sel(v: u32) {
    let nv = read_32(SYSCON_IOPAD_CTRL104) & !(0x7);
    write_32(SYSCON_IOPAD_CTRL104, nv | v & 0x7);
}

pub fn syscon_func_0(v: u32) {
    // NOTE: for whatever reason, it appears that writing only works after
    // reading i.e., if you remove the `read_32`, it breaks the code
    // let's hope the compiler does not remove it in optimization
    read_32(SYSCON_IOPAD_CTRL32);
    write_32(SYSCON_IOPAD_CTRL32, v);
}

pub fn syscon_func_1(v: u32) {
    read_32(SYSCON_IOPAD_CTRL33);
    write_32(SYSCON_IOPAD_CTRL33, v);
}

pub fn syscon_func_2(v: u32) {
    read_32(SYSCON_IOPAD_CTRL34);
    write_32(SYSCON_IOPAD_CTRL34, v);
}

pub fn syscon_func_3(v: u32) {
    read_32(SYSCON_IOPAD_CTRL35);
    write_32(SYSCON_IOPAD_CTRL35, v);
}

pub fn syscon_func_6(v: u32) {
    read_32(SYSCON_IOPAD_CTRL38);
    write_32(SYSCON_IOPAD_CTRL38, v);
}

pub fn syscon_func_7(v: u32) {
    read_32(SYSCON_IOPAD_CTRL39);
    write_32(SYSCON_IOPAD_CTRL39, v);
}

pub fn syscon_func_18(v: u32) {
    read_32(SYSCON_IOPAD_CTRL50);
    write_32(SYSCON_IOPAD_CTRL50, v);
}

pub fn iopad_init() {
    syscon_func_0(0x00c00000);
    syscon_func_1(0x00c000c0);
    syscon_func_2(0x00c000c0);
    syscon_func_3(0x00c000c0);
    syscon_func_6(0x00c00000);
    syscon_func_7(0x00c300c3);
    unsafe { asm!("fence") };
}

pub const RSTGEN_BASE: u32 = 0x1184_0000;
#[allow(clippy::identity_op)]
pub const RSTGEN_SOFT_ASSERT0: u32 = RSTGEN_BASE + 0x0;
pub const RSTGEN_SOFT_ASSERT1: u32 = RSTGEN_BASE + 0x4;
pub const RSTGEN_SOFT_ASSERT2: u32 = RSTGEN_BASE + 0x8;
pub const RSTGEN_SOFT_ASSERT3: u32 = RSTGEN_BASE + 0xC;

pub const RSTGEN_SOFT_STATUS0: u32 = RSTGEN_BASE + 0x10;
pub const RSTGEN_SOFT_STATUS1: u32 = RSTGEN_BASE + 0x14;
pub const RSTGEN_SOFT_STATUS2: u32 = RSTGEN_BASE + 0x18;
pub const RSTGEN_SOFT_STATUS3: u32 = RSTGEN_BASE + 0x1C;

pub fn clear_rstgen_rstn_usbnoc_axi_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 6);
    v |= 0 << 6;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 6;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn clear_rstgen_rstn_hifi4noc_axi_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 2);
    v |= 0 << 2;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 2;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub const CLK_X2C_AXI_CTRL: u32 = CLKGEN_BASE + 0x15C;
pub const CLK_MSI_APB_CTRL: u32 = CLKGEN_BASE + 0x2D8;
pub fn enable_clk_x2c_axi_() {
    let mut v = read_32(CLK_X2C_AXI_CTRL);
    v &= !(0x1 << 31);
    v |= 1 << 31;
    write_32(CLK_X2C_AXI_CTRL, v);
}

pub fn enable_clk_msi_apb_() {
    let mut v = read_32(CLK_MSI_APB_CTRL);
    v &= !(0x1 << 31);
    v |= 1 << 31;
    write_32(CLK_MSI_APB_CTRL, v);
}

pub fn clear_rstgen_rstn_x2c_axi_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 9);
    v |= 0 << 9;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 9;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn assert_rstgen_rstn_x2c_axi_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 9);
    v |= 1 << 9;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 9;
        v &= 0x1;
        if !v != 0x0 {
            break;
        }
    }
}

pub fn clear_rstgen_rstn_msi_apb_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT3);
    v &= !(0x1 << 14);
    v |= 0 << 14;
    write_32(RSTGEN_SOFT_ASSERT3, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS3) >> 14;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn clear_rstgen_rstn_dspx2c_axi_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 14);
    v |= 0 << 14;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 14;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn clear_rstgen_rstn_dma1p_axi_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 8);
    v |= 0 << 8;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 8;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn rstgen_init() {
    clear_rstgen_rstn_usbnoc_axi_();
    clear_rstgen_rstn_hifi4noc_axi_();

    enable_clk_x2c_axi_();
    clear_rstgen_rstn_x2c_axi_();

    clear_rstgen_rstn_dspx2c_axi_();
    clear_rstgen_rstn_dma1p_axi_();

    enable_clk_msi_apb_();
    clear_rstgen_rstn_msi_apb_();

    assert_rstgen_rstn_x2c_axi_();
    clear_rstgen_rstn_x2c_axi_();
    unsafe { asm!("fence") };
}

pub fn syscon_func_57(v: u32) {
    read_32(SYSCON_IOPAD_CTRL89);
    write_32(SYSCON_IOPAD_CTRL89, v);
}

pub fn syscon_func_58(v: u32) {
    read_32(SYSCON_IOPAD_CTRL90);
    write_32(SYSCON_IOPAD_CTRL90, v);
}

pub fn syscon_func_59(v: u32) {
    read_32(SYSCON_IOPAD_CTRL91);
    write_32(SYSCON_IOPAD_CTRL91, v);
}

pub fn syscon_func_60(v: u32) {
    read_32(SYSCON_IOPAD_CTRL92);
    write_32(SYSCON_IOPAD_CTRL92, v);
}

pub fn syscon_func_61(v: u32) {
    read_32(SYSCON_IOPAD_CTRL93);
    write_32(SYSCON_IOPAD_CTRL93, v);
}

pub fn syscon_func_62(v: u32) {
    read_32(SYSCON_IOPAD_CTRL94);
    write_32(SYSCON_IOPAD_CTRL94, v);
}

pub fn syscon_func_63(v: u32) {
    read_32(SYSCON_IOPAD_CTRL95);
    write_32(SYSCON_IOPAD_CTRL95, v);
}

pub fn syscon_func_64(v: u32) {
    read_32(SYSCON_IOPAD_CTRL96);
    write_32(SYSCON_IOPAD_CTRL96, v);
}

pub fn syscon_func_65(v: u32) {
    read_32(SYSCON_IOPAD_CTRL97);
    write_32(SYSCON_IOPAD_CTRL97, v);
}

pub fn syscon_func_66(v: u32) {
    read_32(SYSCON_IOPAD_CTRL98);
    write_32(SYSCON_IOPAD_CTRL98, v);
}
pub fn syscon_func_67(v: u32) {
    read_32(SYSCON_IOPAD_CTRL99);
    write_32(SYSCON_IOPAD_CTRL99, v);
}

pub fn syscon_func_68(v: u32) {
    read_32(SYSCON_IOPAD_CTRL100);
    write_32(SYSCON_IOPAD_CTRL100, v);
}

pub fn syscon_func_69(v: u32) {
    read_32(SYSCON_IOPAD_CTRL101);
    write_32(SYSCON_IOPAD_CTRL101, v);
}

pub fn syscon_func_70(v: u32) {
    read_32(SYSCON_IOPAD_CTRL102);
    write_32(SYSCON_IOPAD_CTRL102, v);
}

pub fn clear_rstgen_rstn_gmac_ahb_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 28);
    v |= 0 << 28;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 28;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn assert_rstgen_rstn_gmac_ahb_() {
    let mut v = read_32(RSTGEN_SOFT_ASSERT1);
    v &= !(0x1 << 28);
    v |= 1 << 28;
    write_32(RSTGEN_SOFT_ASSERT1, v);
    loop {
        let mut v = read_32(RSTGEN_SOFT_STATUS1) >> 28;
        v &= 0x1;
        if !v != 0x0 {
            break;
        }
    }
}

pub const CLK_GMAC_AHB_CTRL: u32 = CLKGEN_BASE + 0x1E0;
pub const CLK_GMAC_PTP_REFCLK_CTRL: u32 = CLKGEN_BASE + 0x1E8;
pub const CLK_GMAC_GTXCLK_CTRL: u32 = CLKGEN_BASE + 0x1EC;

pub fn enable_clk_gmac_ahb_() {
    let v = read_32(CLK_GMAC_AHB_CTRL) & !(0x1 << 31);
    write_32(CLK_GMAC_AHB_CTRL, v | 1 << 31);
}

pub fn enable_clk_gmac_ptp_refclk_() {
    let v = read_32(CLK_GMAC_PTP_REFCLK_CTRL) & !(0x1 << 31);
    write_32(CLK_GMAC_PTP_REFCLK_CTRL, v | 1 << 31);
}

pub fn enable_clk_gmac_gtxclk_() {
    let v = read_32(CLK_GMAC_GTXCLK_CTRL) & !(0x1 << 31);
    write_32(CLK_GMAC_GTXCLK_CTRL, v | 1 << 31);
}

pub fn syscon_init() {
    /*phy must use gpio to hardware reset*/
    enable_clk_gmac_ahb_();
    enable_clk_gmac_ptp_refclk_();
    enable_clk_gmac_gtxclk_();
    assert_rstgen_rstn_gmac_ahb_();
    // GMAC_PHY_RXCLK ...?
    syscon_func_57(0x00030080);
    syscon_func_58(0x00030080);
    syscon_func_59(0x00030003);
    syscon_func_60(0x00030003);
    syscon_func_61(0x00030003);
    syscon_func_62(0x00030003);
    syscon_func_63(0x00800003);
    syscon_func_64(0x00800080);
    syscon_func_65(0x00800080);
    syscon_func_66(0x00800080);
    syscon_func_67(0x00800080);
    syscon_func_68(0x00800080);
    syscon_func_69(0x00800080);
    // GMAC_MDC ?
    syscon_func_70(0x00800080);

    clear_rstgen_rstn_gmac_ahb_();
    syscon_gmac_phy_intf_sel(0x1); //rgmii
}
