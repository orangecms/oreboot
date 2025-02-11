use core::ptr::write_volatile;

pub fn write32(reg: usize, val: u32) {
    unsafe {
        write_volatile(reg as *mut u32, val);
    }
}

pub fn write64(reg: usize, val: u64) {
    write32(reg, val as u32);
    write32(reg + 4, (val >> 32) as u32);
}

pub fn dump(addr: usize, size: usize) {
    let s = unsafe { core::slice::from_raw_parts(addr as *const u8, size) };
    for w in s.iter() {
        print!("{:02x}", w);
    }
    println!();
}

pub fn dump_block(addr: usize, size: usize, step_size: usize) {
    println!("[SBI] dump {size} bytes @{addr:x}");
    for b in (addr..addr + size).step_by(step_size) {
        dump(b, step_size);
    }
}
