use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const LINKERSCRIPT_FILENAME: &str = "link-milkv-meles-bt0.ld";

// SRAM size is really 1.5M, but who needs that?
const LINKERSCRIPT: &[u8] = b"
OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY {
    SRAM : ORIGIN = 0xFFE0000000, LENGTH = 128k
}
SECTIONS {
    .head : {
        *(.head.text)
    } > SRAM
    .text : {
        KEEP(*(.text.entry))
        *(.text .text.*)
        . = ALIGN(8);
    } > SRAM
    .bss : {
        _sbss = .;
        *(.bss .bss.*);
        _ebss = .;
    } > SRAM

    # https://docs.rust-embedded.org/embedonomicon/main.html
    .rodata : {
        *(.rodata .rodata.*);
    } > SRAM
    .data : {
        _sdata = .;
        *(.data .data.*);
        _edata = .;
    } > SRAM
    _sidata = LOADADDR(.data);

    /DISCARD/ : {
        *(.eh_frame)
        *(.debug_*)
        *(.comment*)
    }
}";

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join(LINKERSCRIPT_FILENAME))
        .unwrap()
        .write_all(LINKERSCRIPT)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
