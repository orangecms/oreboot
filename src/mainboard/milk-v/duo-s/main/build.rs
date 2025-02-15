use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

// Same for all SoCs: CV1800B (Milk-V Duo), SG200x (Milk-V Duo 256M, Duo S)
const MEM_BASE: usize = 0x80000000;
const MEM_SIZE: usize = 2 * 1024 * 1024;

const LINKERSCRIPT_FILENAME: &str = "link.x";
const LINKERSCRIPT_TEMPLATE: &str = r#"
OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY {
    MEM : ORIGIN = $MEM_BASE$, LENGTH = $MEM_SIZE$
}
SECTIONS {
    .head : {
        *(.head.text)
    } > MEM
    .text : {
        KEEP(*(.text.entry))
        *(.text .text.*)
        . = ALIGN(8);
    } > MEM
    .bss : {
        _sbss = .;
        *(.bss .bss.*);
        _ebss = .;
        . = ALIGN(8);
    } > MEM

    # https://docs.rust-embedded.org/embedonomicon/main.html
    .rodata : {
        *(.rodata .rodata.*);
    } > MEM
    .data : {
        _sdata = .;
        *(.data .data.*);
        _edata = .;
    } > MEM
    _sidata = LOADADDR(.data);

    /DISCARD/ : {
        *(.eh_frame)
        *(.debug_*)
        *(.comment*)
    }
}"#;

fn main() {
    // Until someone figures out how we can simplify this, do step by step
    // conversion and string replacement to inject memory base and size.
    let mem_base = format!("0x{MEM_BASE:08x}");
    let ms = MEM_SIZE / 1024;
    let mem_size = format!("{ms}K");
    let ls = LINKERSCRIPT_TEMPLATE.replace("$MEM_BASE$", mem_base.as_str());
    let ls = ls.replace("$MEM_SIZE$", mem_size.as_str());
    let ls = ls.as_bytes();

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join(LINKERSCRIPT_FILENAME))
        .unwrap()
        .write_all(ls)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
