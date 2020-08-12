#![feature(llvm_asm)]
#![feature(lang_items, start)]
#![no_std]
#![no_main]
#![feature(global_asm)]

use arch::ioport::IOPort;
use core::fmt::Write;
use core::panic::PanicInfo;
use model::Driver;
use payloads::payload;
use print;
use uart::i8250::I8250;
mod mainboard;
use mainboard::MainBoard;

extern crate heapless; // v0.4.x
use heapless::consts::*;
use heapless::Vec;

use core::ptr;
// Until we are done hacking on this, use our private copy.
// Plan to copy it back later.
global_asm!(include_str!("bootblock.S"));
fn peek(a: u32) -> u32 {
    let y = a as *const u32;
    unsafe { ptr::read_volatile(y) }
}

//global_asm!(include_str!("init.S"));

#[no_mangle]
pub extern "C" fn _start(fdt_address: usize) -> ! {
    let m = &mut MainBoard::new();
    m.init().unwrap();
    let io = &mut IOPort;
    let post = &mut IOPort;
    let uart0 = &mut I8250::new(0x3f8, 0, io);
    uart0.init().unwrap();

    uart0.pwrite(b"Welcome to oreboot\r\n", 0).unwrap();
    let mut p: [u8; 1] = [0xf0; 1];
    post.pwrite(&p, 0x80).unwrap();
    let w = &mut print::WriteTo::new(uart0);
    p[0] = p[0] + 1;
    loop {
        let io = &mut IOPort;
        let uart0 = &mut I8250::new(0x3f8, 0, io);
        let mut line: Vec<u8, U8> = Vec::new();
        loop {
            let mut c: [u8; 1] = [12;1];
            uart0.pread(&mut c, 1).unwrap();
            if c[0] == 12 || c[0] == 4 {
                break;
            }
            line.push(c[0]).unwrap();
        }
        write!(w, "Read this line: {:?}", line).unwrap();
        break;
    }
    
    for a in 0x7c0000..0x7f0000 {
        let v = peek(a);
        if v == 0x7f454c46 {
            write!(w, "found sig at {}\r\n", v).unwrap();
        }
        if v == 0x464c457f {
            write!(w, "found back sig at {}\r\n", v).unwrap();
        }
    }
    let payload = &mut payload::StreamPayload { typ: payload::ftype::CBFS_TYPE_SELF, compression: payload::ctype::CBFS_COMPRESS_NONE, offset: 0, entry: 0x1000020 as usize, rom_len: 0 as usize, mem_len: 0 as usize, dtb: 0, rom: 0x76c00000 };
    post.pwrite(&p, 0x80).unwrap();
    p[0] = p[0] + 1;
    write!(w, "loading payload with fdt_address {}\r\n", fdt_address).unwrap();
    post.pwrite(&p, 0x80).unwrap();
    p[0] = p[0] + 1;
    payload.load(w);
    post.pwrite(&p, 0x80).unwrap();
    p[0] = p[0] + 1;
    write!(w, "Running payload\r\n").unwrap();
    post.pwrite(&p, 0x80).unwrap();
    p[0] = p[0] + 1;
    payload.run(w);
    post.pwrite(&p, 0x80).unwrap();
    p[0] = p[0] + 1;

    write!(w, "Unexpected return from payload\r\n").unwrap();
    post.pwrite(&p, 0x80).unwrap();
    p[0] = p[0] + 1;
    arch::halt()
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Assume that uart0.init() has already been called before the panic.
    let io = &mut IOPort;
    let uart0 = &mut I8250::new(0x3f8, 0, io);
    let w = &mut print::WriteTo::new(uart0);
    // Printing in the panic handler is best-effort because we really don't want to invoke the panic
    // handler from inside itself.
    let _ = write!(w, "PANIC: {}\r\n", info);
    arch::halt()
}
