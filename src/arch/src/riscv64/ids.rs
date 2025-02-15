use riscv::register::{marchid, mhartid, mimpid, mvendorid};

use log::println;

fn vendorid_to_name<'a>(vendorid: usize) -> &'a str {
    match vendorid {
        0x0489 => "SiFive",
        0x05b7 => "T-Head",
        _ => "unknown",
    }
}

// FIXME: This really depends on the vendor first!
fn impid_to_name<'a>(impid: usize) -> &'a str {
    match impid {
        0x0421_0427 => "21G1.02.00 / llama.02.00-general",
        _ => "unknown",
    }
}

/// Print RISC-V core information:
/// - hart ID
/// - vendor
/// - arch
/// - implementation
pub fn print_ids() {
    let hart_id = mhartid::read();
    println!("RISC-V hart ID {hart_id}");
    let aid = marchid::read().map(|r| r.bits()).unwrap_or(0);
    println!("RISC-V arch {aid:08x}");
    let vid = mvendorid::read().map(|r| r.bits()).unwrap_or(0);
    let vendor_name = vendorid_to_name(vid);
    println!("RISC-V core vendor: {vendor_name} (0x{vid:04x})");
    let iid = mimpid::read().map(|r| r.bits()).unwrap_or(0);
    let imp_name = impid_to_name(iid);
    println!("RISC-V implementation: {imp_name} (0x{iid:08x})");
}
