use core::arch::asm;

pub fn init() {
    dump_csrs();
    init_csrs();
    dump_csrs();
}

fn dump_csrs() {
    let mut v: usize;
    unsafe {
        println!("==== platform CSRs ====");
        asm!("csrr {}, 0x7c0", out(reg) v);
        println!("   MXSTATUS  {v:08x}");
        asm!("csrr {}, 0x7c1", out(reg) v);
        println!("   MHCR      {v:08x}");
        asm!("csrr {}, 0x7c2", out(reg) v);
        println!("   MCOR      {v:08x}");
        asm!("csrr {}, 0x7c5", out(reg) v);
        println!("   MHINT     {v:08x}");
        println!("see C906 manual p581 ff");
        println!("=======================");
    }
}

fn init_csrs() {
    println!("Set up extension CSRs");
    if false {
        unsafe {
            asm!("csrs 0x7c0, {}", in(reg) 0x00018000);
        }
    }
    unsafe {
        // MXSTATUS: T-Head ISA extension enable, MAEE, MM, UCME, CLINTEE
        // NOTE: Linux relies on detecting errata via mvendorid, marchid and
        // mipmid. If that detection fails, and we enable MAEE, Linux won't come
        // up. When D-cache is enabled, and the detection fails, we run into
        // cache coherency issues. Welcome to the minefield! :)
        // NOTE: We already set part of this in bt0, but it seems to get lost?
        asm!("csrs 0x7c0, {}", in(reg) 0x00638000);
        // MCOR: invalidate ICACHE/DCACHE/BTB/BHT
        asm!("csrw 0x7c2, {}", in(reg) 0x00070013);
        // MHCR
        asm!("csrw 0x7c1, {}", in(reg) 0x000011ff);
        // MHINT
        asm!("csrw 0x7c5, {}", in(reg) 0x0016e30c);
    }
}

// XuanTie (T-Head) CPU model register
const MCPUID: u32 = 0xfc0;

// The machine mode processor model register (MCPUID) stores the processor
// model information. Its reset value is determined by the product itself and
// complies with the Pingtouge product definition specifications to facilitate
// software identification. By continuously reading the MCPUID register, up to
// 7 different return values can be obtained to represent C906 product
// information, as shown in Figure ??.
pub fn print_cpuid() {
    let mut id: u32;
    for i in 0..7 {
        unsafe { asm!("csrr {}, 0xfc0", out(reg) id) };
        println!("MCPUID {i}: {id:08x}");
    }
}
