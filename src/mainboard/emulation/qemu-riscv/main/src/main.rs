#![doc = include_str!("../README.md")]
#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
global_asm!(include_str!("bootblock.S"));
global_asm!(include_str!("init.S"));

use oreboot_arch::riscv64::riscv::register::mhartid;

static PLATFORM: &str = "QEMU RISC-V";
static VERSION: &str = env!("CARGO_PKG_VERSION");

#[macro_use]
extern crate log;

mod mem_map;
mod sbi_platform;
mod uart;
mod util;

static mut SERIAL: Option<uart::QEMUSerial> = None;

fn init_logger(s: uart::QEMUSerial) {
    unsafe {
        SERIAL.replace(s);
        if let Some(m) = SERIAL.as_mut() {
            log::init(m);
        }
    }
}

#[no_mangle]
pub extern "C" fn _start(dtb_address: usize) -> ! {
    let s = uart::QEMUSerial::new();
    init_logger(s);
    println!("oreboot 🦀 main");

    use oreboot_arch::riscv64::sbi as ore_sbi;
    let sbi = sbi_platform::init();
    ore_sbi::runtime::init();
    ore_sbi::info::print_info(PLATFORM, VERSION);

    if false {
        util::dump_block(mem_map::PAYLOAD_ADDR, 0x80, 0x20);
    }

    let hart_id = mhartid::read();
    let (reset_type, reset_reason) = ore_sbi::execute::execute_supervisor(
        sbi,
        mem_map::PAYLOAD_ADDR,
        hart_id,
        dtb_address,
        Some(mem_map::CLINT_BASE),
    );
    println!("[oreboot] reset; reason: {reset_reason}, type: {reset_type}");

    loop {
        oreboot_arch::riscv64::riscv::asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
