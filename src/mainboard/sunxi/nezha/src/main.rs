#![no_std]
#![no_main]
#![feature(default_alloc_error_handler)]

use core::fmt::Write;
use core::panic::PanicInfo;
use core::ptr::read_volatile;
use core::{arch::global_asm, ptr::write_volatile};
use oreboot_arch::riscv64 as arch;
use oreboot_drivers::{
    uart::sunxi::{Sunxi, UART0},
    wrappers::{DoD, Memory, SectionReader},
    Driver,
};
use oreboot_soc::sunxi::d1::{ccu::CCU, gpio::GPIO};
use payloads::payload;
use sbi::sbi_init;

global_asm!(include_str!("../start.S"));

const MEM: usize = 0x4000_0000;
const CACHED_MEM: usize = 0x8000_0000;

/// copy forward to dest, so the loop goes backwards to not run into overlaps
unsafe fn bcopy(w: &mut print::WriteTo<DoD>, dest: usize, from: usize, size: usize) {
    let qsize = size / 4;
    writeln!(w, " size {:08x}  qsize {:08x}\r", size, qsize).unwrap();
    let range = (0..qsize).rev();
    for offs in range {
        let faddr = from + offs * 4;
        let v = read_volatile(faddr as *mut u32);
        let daddr = dest + offs * 4;
        if offs % 0x1_0000 == 0 {
            writeln!(w, " v {:08x}  f {:x}  d {:x}\r", v, faddr, daddr).unwrap();
        }
        write_volatile(daddr as *mut u32, v);
    }
}

// hart = hardware thread (something like core)
#[no_mangle]
pub extern "C" fn _start() -> ! {
    // clock
    let mut ccu = CCU::new();
    ccu.init().unwrap();
    let mut gpio = GPIO::new();
    gpio.init().unwrap();
    let mut uart0 = Sunxi::new(UART0 as usize, 115200);
    uart0.init().unwrap();
    uart0.pwrite(b"UART0 initialized\r\n", 0).unwrap();

    let mut uarts = [&mut uart0 as &mut dyn Driver];
    let console = &mut DoD::new(&mut uarts[..]);
    console.init().unwrap();
    console.pwrite(b"Welcome to oreboot\r\n", 0).unwrap();

    let w = &mut print::WriteTo::new(console);
    writeln!(w, "## Loading payload\r").unwrap();

    // see ../fixed-dtfs.dts
    // TODO: adjust when DRAM driver is implemented / booting from SPI
    let payload_offset = 0x2_0000;
    let payload_size = 0x1000;
    let linuxboot_offset = payload_offset + payload_size;
    let linuxboot_size = 0xee_0000;
    let dtb_offset = linuxboot_offset + linuxboot_size;
    let dtb_size = 0xe000;

    // TODO; This payload structure should be loaded from boot medium rather
    // than hardcoded.
    let segs = &[
        payload::Segment {
            typ: payload::stype::PAYLOAD_SEGMENT_ENTRY,
            base: CACHED_MEM,
            data: &mut SectionReader::new(&Memory {}, MEM + payload_offset, payload_size),
        },
        payload::Segment {
            typ: payload::stype::PAYLOAD_SEGMENT_ENTRY,
            base: CACHED_MEM,
            data: &mut SectionReader::new(&Memory {}, MEM + linuxboot_offset, linuxboot_size),
        },
        payload::Segment {
            typ: payload::stype::PAYLOAD_SEGMENT_ENTRY,
            base: CACHED_MEM,
            data: &mut SectionReader::new(&Memory {}, MEM + dtb_offset, dtb_size),
        },
    ];
    // TODO: Get this from configuration
    let use_sbi = true;
    if use_sbi {
        let lb_addr = MEM + 0x0020_0000;
        let dtb_addr = lb_addr + 0x0100_0000;
        /*
        unsafe {
            // backwards, so DTB first
            writeln!(
                w,
                "from 0x{:x} to 0x{:x}\r",
                MEM + dtb_offset,
                pbase + linuxboot_size
            )
            .unwrap();
            bcopy(w, pbase + linuxboot_size, MEM + dtb_offset, dtb_size);
            writeln!(w, "from 0x{:x} to 0x{:x}\r", MEM + linuxboot_offset, pbase).unwrap();
            bcopy(w, pbase, MEM + linuxboot_offset, linuxboot_size);
        }
        */
        writeln!(w, "Handing over to SBI, will continue at 0x{:x}\r", lb_addr).unwrap();
        sbi_init(lb_addr, dtb_addr);
    } else {
        // FIXME: This is not copied as of now and relies on the DTFS setting up
        // the positions correctly, whoops.
        let entry = MEM + payload_offset;
        let payload: payload::Payload = payload::Payload {
            typ: payload::ftype::CBFS_TYPE_RAW,
            compression: payload::ctype::CBFS_COMPRESS_NONE,
            offset: 0,
            entry,
            dtb: 0,
            // TODO: These two length fields are not used.
            rom_len: 0,
            mem_len: 0,
            segs,
        };
        // payload.load();
        writeln!(w, "Running payload entry 0x{:x}\r", entry).unwrap();
        payload.run();
        writeln!(w, "Unexpected return from payload\r").unwrap();
    }
    arch::halt()
}

/// This function is called on panic.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Assume that uart0.init() has already been called before the panic.
    let mut uart0 = Sunxi::new(UART0 as usize, 115200);
    let w = &mut print::WriteTo::new(&mut uart0);
    // Printing in the panic handler is best-effort because we really don't want to invoke the panic
    // handler from inside itself.
    let _ = writeln!(w, "PANIC: {}\r", info);
    arch::halt()
}
