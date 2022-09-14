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

fn serial_out(reg: u32, val: u32) {
    unsafe {
        write_volatile(reg as *mut u32, val);
    }
}

fn serial_in(reg: u32) -> u32 {
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
            let lsr = serial_in(REG_LSR) as u8 & LSR_THRE;
            if lsr != 0 {
                break;
            }
        }
        */
        write_volatile(UART3_BASE as *mut u32, c as u32);
    }
}

fn write_u8(reg: u32, val: u8) {
    unsafe {
        write_volatile(reg as *mut u8, val);
    }
}

pub fn uart_init() {
    // move UART to other header
    syscon_cfg_104_io_padshare_sel(6);
    let divisor = (UART_CLK / UART_BAUDRATE_32MCLK_115200) >> 4;

    let lcr_cache = serial_in(REG_LCR);
    write_u8(REG_LCR, LCR_DLAB | lcr_cache as u8);
    write_u8(REG_BRDL, divisor as u8);
    write_u8(REG_BRDH, (divisor >> 8) as u8);

    /* restore the DLAB to access the baud rate divisor registers */
    write_u8(REG_LCR, lcr_cache as u8);
    /* 8 data bits, 1 stop bit, no parity, clear DLAB */
    write_u8(REG_LCR, (LCR_CS8 | LCR_1_STB | LCR_PDIS) as u8);

    write_u8(REG_MDC, 0); /*disable flow control*/

    /*
     * Program FIFO: enabled, mode 0 (set for compatibility with quark),
     * generate the interrupt at 8th byte
     * Clear TX and RX FIFO
     */
    write_u8(
        REG_FCR,
        (FCR_FIFO | FCR_MODE1 | /*FCR_FIFO_1*/FCR_FIFO_8 | FCR_RCVRCLR | FCR_XMITCLR) as u8,
    );

    write_u8(REG_IER, 0); // disable the serial interrupt
}

pub const CLKGEN_BASE_ADDR: u32 = 0x1180_0000;
const clk_cpundbus_root_ctrl: u32 = CLKGEN_BASE_ADDR + 0x0;
pub const clk_dla_root_ctrl: u32 = CLKGEN_BASE_ADDR + 0x4;
pub const clk_dsp_root_ctrl: u32 = CLKGEN_BASE_ADDR + 0x8;
pub const clk_gmacusb_root_ctrl: u32 = CLKGEN_BASE_ADDR + 0xC;
pub const clk_perh0_root_ctrl: u32 = CLKGEN_BASE_ADDR + 0x10;

pub fn _SWITCH_CLOCK_clk_cpundbus_root_SOURCE_clk_pll0_out_() {
    let mut v = serial_in(clk_cpundbus_root_ctrl);
    v &= !(0x3 << 24);
    v |= (0x1 & 0x3) << 24;
    serial_out(clk_cpundbus_root_ctrl, v);
}

pub fn _SWITCH_CLOCK_clk_dla_root_SOURCE_clk_pll1_out_() {
    let mut v = serial_in(clk_dla_root_ctrl);
    v &= !(0x3 << 24);
    v |= (0x1 & 0x3) << 24;
    serial_out(clk_dla_root_ctrl, v);
}

pub fn _SWITCH_CLOCK_clk_dsp_root_SOURCE_clk_pll2_out_() {
    let mut v = serial_in(clk_dsp_root_ctrl);
    v &= !(0x3 << 24);
    v |= 3 << 24;
    serial_out(clk_dsp_root_ctrl, v);
}

pub fn _SWITCH_CLOCK_clk_perh0_root_SOURCE_clk_pll0_out_() {
    let mut v = serial_in(clk_perh0_root_ctrl);
    v &= !(0x1 << 24);
    v |= 1 << 24;
    serial_out(clk_perh0_root_ctrl, v);
}

fn init_coreclk() {
    // TODO: make base a parameter.
    _SWITCH_CLOCK_clk_cpundbus_root_SOURCE_clk_pll0_out_();
    _SWITCH_CLOCK_clk_dla_root_SOURCE_clk_pll1_out_();
    _SWITCH_CLOCK_clk_dsp_root_SOURCE_clk_pll2_out_();
    _SWITCH_CLOCK_clk_perh0_root_SOURCE_clk_pll0_out_();

    // not enabled in original.
    // slow down nne bus can fix nne50 & vp6 ram scan issue,
    // as well as vin_subsys reg scan issue.
    //	_SWITCH_CLOCK_clk_nne_bus_SOURCE_clk_cpu_axi_;
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

pub const SYSCON_IOPAD_CTRL_BASE: u32 = 0x00_1185_8000;
pub const SYSCON_IOPAD_CTRL32: u32 = SYSCON_IOPAD_CTRL_BASE + 0x80;
pub const SYSCON_IOPAD_CTRL33: u32 = SYSCON_IOPAD_CTRL_BASE + 0x84;
pub const SYSCON_IOPAD_CTRL34: u32 = SYSCON_IOPAD_CTRL_BASE + 0x88;
pub const SYSCON_IOPAD_CTRL35: u32 = SYSCON_IOPAD_CTRL_BASE + 0x8c;
pub const SYSCON_IOPAD_CTRL38: u32 = SYSCON_IOPAD_CTRL_BASE + 0x98;
pub const SYSCON_IOPAD_CTRL39: u32 = SYSCON_IOPAD_CTRL_BASE + 0x9C;
pub const SYSCON_IOPAD_CTRL50: u32 = SYSCON_IOPAD_CTRL_BASE + 0xC8;

pub fn syscon_cfg_50_func_18(v: u32) {
    serial_in(SYSCON_IOPAD_CTRL50);
    serial_out(SYSCON_IOPAD_CTRL50, v);
}

pub fn syscon_cfg_104_io_padshare_sel(v: u32) {
    let nv = serial_in(syscon_cfg_iopad_ctrl104) & !(0x7);
    serial_out(syscon_cfg_iopad_ctrl104, nv | v & 0x7);
}

pub fn syscon_cfg_32_func_0(v: u32) {
    // NOTE: for whatever reason, it appears that writing only works after
    // reading i.e., if you remove the `serial_in`, it breaks the code
    // let's hope the compiler does not remove it in optimization
    serial_in(SYSCON_IOPAD_CTRL32);
    serial_out(SYSCON_IOPAD_CTRL32, v);
}

pub fn syscon_cfg_33_func_1(v: u32) {
    serial_in(SYSCON_IOPAD_CTRL33);
    serial_out(SYSCON_IOPAD_CTRL33, v);
}

pub fn syscon_cfg_34_func_2(v: u32) {
    serial_in(SYSCON_IOPAD_CTRL34);
    serial_out(SYSCON_IOPAD_CTRL34, v);
}

pub fn syscon_cfg_35_func_3(v: u32) {
    serial_in(SYSCON_IOPAD_CTRL35);
    serial_out(SYSCON_IOPAD_CTRL35, v);
}

pub fn syscon_cfg_38_func_6(v: u32) {
    serial_in(SYSCON_IOPAD_CTRL38);
    serial_out(SYSCON_IOPAD_CTRL38, v);
}

pub fn syscon_cfg_39_func_7(v: u32) {
    serial_in(SYSCON_IOPAD_CTRL39);
    serial_out(SYSCON_IOPAD_CTRL39, v);
}

pub fn iopad_init() {
    syscon_cfg_32_func_0(0x00c00000);
    syscon_cfg_33_func_1(0x00c000c0);
    syscon_cfg_34_func_2(0x00c000c0);
    syscon_cfg_35_func_3(0x00c000c0);
    syscon_cfg_38_func_6(0x00c00000);
    syscon_cfg_39_func_7(0x00c300c3);
    unsafe { asm!("fence") };
}

pub const RSTGEN_BASE_ADDR: u32 = 0x1184_0000;
#[allow(clippy::identity_op)]
pub const rstgen_Software_RESET_assert0: u32 = RSTGEN_BASE_ADDR + 0x0;
pub const rstgen_Software_RESET_assert1: u32 = RSTGEN_BASE_ADDR + 0x4;
pub const rstgen_Software_RESET_assert2: u32 = RSTGEN_BASE_ADDR + 0x8;
pub const rstgen_Software_RESET_assert3: u32 = RSTGEN_BASE_ADDR + 0xC;

pub const rstgen_Software_RESET_status0: u32 = RSTGEN_BASE_ADDR + 0x10;
pub const rstgen_Software_RESET_status1: u32 = RSTGEN_BASE_ADDR + 0x14;
pub const rstgen_Software_RESET_status2: u32 = RSTGEN_BASE_ADDR + 0x18;
pub const rstgen_Software_RESET_status3: u32 = RSTGEN_BASE_ADDR + 0x1C;
pub fn _CLEAR_RESET_rstgen_rstn_usbnoc_axi_() {
    let mut v = serial_in(rstgen_Software_RESET_assert1);
    v &= !(0x1 << 6);
    v |= 0 << 6;
    serial_out(rstgen_Software_RESET_assert1, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status1) >> 6;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn _CLEAR_RESET_rstgen_rstn_hifi4noc_axi_() {
    let mut v = serial_in(rstgen_Software_RESET_assert1);
    v &= !(0x1 << 2);
    v |= 0 << 2;
    serial_out(rstgen_Software_RESET_assert1, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status1) >> 2;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub const clk_x2c_axi_ctrl: u32 = CLKGEN_BASE_ADDR + 0x15C;
pub const clk_msi_apb_ctrl: u32 = CLKGEN_BASE_ADDR + 0x2D8;
pub fn _ENABLE_CLOCK_clk_x2c_axi_() {
    let mut v = serial_in(clk_x2c_axi_ctrl);
    v &= !(0x1 << 31);
    v |= 1 << 31;
    serial_out(clk_x2c_axi_ctrl, v);
}

pub fn _ENABLE_CLOCK_clk_msi_apb_() {
    let mut v = serial_in(clk_msi_apb_ctrl);
    v &= !(0x1 << 31);
    v |= 1 << 31;
    serial_out(clk_msi_apb_ctrl, v);
}

pub fn _CLEAR_RESET_rstgen_rstn_x2c_axi_() {
    let mut v = serial_in(rstgen_Software_RESET_assert1);
    v &= !(0x1 << 9);
    v |= 0 << 9;
    serial_out(rstgen_Software_RESET_assert1, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status1) >> 9;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn _ASSERT_RESET_rstgen_rstn_x2c_axi_() {
    let mut v = serial_in(rstgen_Software_RESET_assert1);
    v &= !(0x1 << 9);
    v |= 1 << 9;
    serial_out(rstgen_Software_RESET_assert1, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status1) >> 9;
        v &= 0x1;
        if !v != 0x0 {
            break;
        }
    }
}

pub fn _CLEAR_RESET_rstgen_rstn_msi_apb_() {
    let mut v = serial_in(rstgen_Software_RESET_assert3);
    v &= !(0x1 << 14);
    v |= 0 << 14;
    serial_out(rstgen_Software_RESET_assert3, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status3) >> 14;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn _CLEAR_RESET_rstgen_rstn_dspx2c_axi_() {
    let mut v = serial_in(rstgen_Software_RESET_assert1);
    v &= !(0x1 << 14);
    v |= 0 << 14;
    serial_out(rstgen_Software_RESET_assert1, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status1) >> 14;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn _CLEAR_RESET_rstgen_rstn_dma1p_axi_() {
    let mut v = serial_in(rstgen_Software_RESET_assert1);
    v &= !(0x1 << 8);
    v |= 0 << 8;
    serial_out(rstgen_Software_RESET_assert1, v);
    loop {
        let mut v = serial_in(rstgen_Software_RESET_status1) >> 8;
        v &= 0x1;
        if !v != 0x1 {
            break;
        }
    }
}

pub fn rstgen_init() {
    _CLEAR_RESET_rstgen_rstn_usbnoc_axi_();
    _CLEAR_RESET_rstgen_rstn_hifi4noc_axi_();

    _ENABLE_CLOCK_clk_x2c_axi_();
    _CLEAR_RESET_rstgen_rstn_x2c_axi_();

    //_CLEAR_RESET_rstgen_rstn_dspx2c_axi_();
    //_CLEAR_RESET_rstgen_rstn_dma1p_axi_();

    _ENABLE_CLOCK_clk_msi_apb_();
    _CLEAR_RESET_rstgen_rstn_msi_apb_();

    _ASSERT_RESET_rstgen_rstn_x2c_axi_();
    _CLEAR_RESET_rstgen_rstn_x2c_axi_();
    unsafe { asm!("fence") };
}

pub const syscon_cfg_iopad_ctrl89: u32 = SYSCON_IOPAD_CTRL_BASE + 0x164;
pub const syscon_cfg_iopad_ctrl90: u32 = SYSCON_IOPAD_CTRL_BASE + 0x168;
pub const syscon_cfg_iopad_ctrl91: u32 = SYSCON_IOPAD_CTRL_BASE + 0x16C;
pub const syscon_cfg_iopad_ctrl92: u32 = SYSCON_IOPAD_CTRL_BASE + 0x170;
pub const syscon_cfg_iopad_ctrl93: u32 = SYSCON_IOPAD_CTRL_BASE + 0x174;
pub const syscon_cfg_iopad_ctrl94: u32 = SYSCON_IOPAD_CTRL_BASE + 0x178;
pub const syscon_cfg_iopad_ctrl95: u32 = SYSCON_IOPAD_CTRL_BASE + 0x17C;
pub const syscon_cfg_iopad_ctrl96: u32 = SYSCON_IOPAD_CTRL_BASE + 0x180;
pub const syscon_cfg_iopad_ctrl97: u32 = SYSCON_IOPAD_CTRL_BASE + 0x184;
pub const syscon_cfg_iopad_ctrl98: u32 = SYSCON_IOPAD_CTRL_BASE + 0x188;
pub const syscon_cfg_iopad_ctrl99: u32 = SYSCON_IOPAD_CTRL_BASE + 0x18C;
pub const syscon_cfg_iopad_ctrl100: u32 = SYSCON_IOPAD_CTRL_BASE + 0x190;
pub const syscon_cfg_iopad_ctrl101: u32 = SYSCON_IOPAD_CTRL_BASE + 0x194;
pub const syscon_cfg_iopad_ctrl102: u32 = SYSCON_IOPAD_CTRL_BASE + 0x198;
pub const syscon_cfg_iopad_ctrl103: u32 = SYSCON_IOPAD_CTRL_BASE + 0x19C;
pub const syscon_cfg_iopad_ctrl104: u32 = SYSCON_IOPAD_CTRL_BASE + 0x1A0;

pub fn syscon_cfg_89_func_57(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl89);
    serial_out(syscon_cfg_iopad_ctrl89, v);
}

pub fn syscon_cfg_90_func_58(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl90);
    serial_out(syscon_cfg_iopad_ctrl90, v);
}

pub fn syscon_cfg_91_func_59(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl91);
    serial_out(syscon_cfg_iopad_ctrl91, v);
}

pub fn syscon_cfg_92_func_60(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl92);
    serial_out(syscon_cfg_iopad_ctrl92, v);
}

pub fn syscon_cfg_93_func_61(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl93);
    serial_out(syscon_cfg_iopad_ctrl93, v);
}

pub fn syscon_cfg_94_func_62(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl94);
    serial_out(syscon_cfg_iopad_ctrl94, v);
}

pub fn syscon_cfg_95_func_63(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl95);
    serial_out(syscon_cfg_iopad_ctrl95, v);
}

pub fn syscon_cfg_96_func_64(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl96);
    serial_out(syscon_cfg_iopad_ctrl96, v);
}

pub fn syscon_cfg_97_func_65(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl97);
    serial_out(syscon_cfg_iopad_ctrl97, v);
}

pub fn syscon_cfg_98_func_66(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl98);
    serial_out(syscon_cfg_iopad_ctrl98, v);
}
pub fn syscon_cfg_99_func_67(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl99);
    serial_out(syscon_cfg_iopad_ctrl99, v);
}

pub fn syscon_cfg_100_func_68(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl100);
    serial_out(syscon_cfg_iopad_ctrl100, v);
}

pub fn syscon_cfg_101_func_69(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl101);
    serial_out(syscon_cfg_iopad_ctrl101, v);
}

pub fn syscon_cfg_102_func_70(v: u32) {
    serial_in(syscon_cfg_iopad_ctrl102);
    serial_out(syscon_cfg_iopad_ctrl102, v);
}
pub fn syscon_init() {
    /*phy must use gpio to hardware reset*/
    //   _ENABLE_CLOCK_clk_gmac_ahb_();
    //   _ENABLE_CLOCK_clk_gmac_ptp_refclk_();
    //   _ENABLE_CLOCK_clk_gmac_gtxclk_();
    //   _ASSERT_RESET_rstgen_rstn_gmac_ahb_();

    syscon_cfg_89_func_57(0x00030080);
    syscon_cfg_90_func_58(0x00030080);

    syscon_cfg_91_func_59(0x00030003);
    syscon_cfg_92_func_60(0x00030003);
    syscon_cfg_93_func_61(0x00030003);
    syscon_cfg_94_func_62(0x00030003);

    syscon_cfg_95_func_63(0x00800003);

    syscon_cfg_96_func_64(0x00800080);
    syscon_cfg_97_func_65(0x00800080);
    syscon_cfg_98_func_66(0x00800080);
    syscon_cfg_99_func_67(0x00800080);
    syscon_cfg_100_func_68(0x00800080);
    syscon_cfg_101_func_69(0x00800080);
    syscon_cfg_102_func_70(0x00800080);

    //   _CLEAR_RESET_rstgen_rstn_gmac_ahb_();
    //   syscon_cfg_28_gmac_phy_intf_sel(0x1); //rgmii
}

pub fn init() {
    clock_init();
    // for illegal instruction exception
    crate::init::syscon_cfg_50_func_18(0x00c000c0);
    rstgen_init();
    iopad_init();
    uart_init();
    syscon_init();
}
