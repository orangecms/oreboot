/* SPDX-License-Identifier: GPL-2.0-only */

// use crate::bios::setup_bios_tables;
// use crate::consts::*;
// use crate::err::Error;
// use crate::utils::round_up_4k;
#[allow(unused_imports)]
#[allow(non_camel_case_types)]
use core::fmt;
use core::fmt::Write;
use core::intrinsics::{copy, transmute};
use core::mem::{size_of, zeroed};
use model::{Driver, Result};
use print;
use wrappers::{Memory, SectionReader};
pub type EntryPoint = unsafe extern "C" fn(r0: usize, dtb: usize);
//use std::mem::{size_of, transmute, zeroed};

pub const E820_RAM: u32 = 1;
pub const E820_RESERVED: u32 = 2;
pub const E820_ACPI: u32 = 3;
pub const E820_NVS: u32 = 4;
pub const E820_UNUSABLE: u32 = 5;

// TODO define these somewhere.
const PAGE_SIZE: usize = 4096;

#[repr(C, packed)]
pub struct E820Entry {
    addr: u64,
    size: u64,
    r#type: u32,
}

pub const E820_MAX: usize = 128;
#[repr(C, packed)]
pub struct BootParams {
    screen_info: [u8; 0x040 - 0x000],     // 0x000
    apm_bios_info: [u8; 0x054 - 0x040],   // 0x040
    _pad2: [u8; 4],                       // 0x054
    tboot_addr: u64,                      // 0x058
    ist_info: [u8; 0x070 - 0x060],        // 0x060
    acpi_rsdp_addr: [u8; 0x078 - 0x070],  // 0x070
    _pad3: [u8; 8],                       // 0x078
    hd0_info: [u8; 0x090 - 0x080],        // 0x080 /* obsolete! */
    hd1_info: [u8; 0x0a0 - 0x090],        // 0x090 /* obsolete! */
    sys_desc_table: [u8; 0x0b0 - 0x0a0],  // 0x0a0 /* obsolete! */
    olpc_ofw_header: [u8; 0x0c0 - 0x0b0], // 0x0b0
    ext_ramdisk_image: u32,               // 0x0c0
    ext_ramdisk_size: u32,                // 0x0c4
    ext_cmd_line_ptr: u32,                // 0x0c8
    _pad4: [u8; 116],                     // 0x0cc
    edid_info: [u8; 0x1c0 - 0x140],       // 0x140
    efi_info: [u8; 0x1e0 - 0x1c0],        // 0x1c0
    alt_mem_k: u32,                       // 0x1e0
    scratch: u32,                         // 0x1e4 /* obsolete! */
    e820_entries: u8,                     // 0x1e8
    eddbuf_entries: u8,                   // 0x1e9
    edd_mbr_sig_buf_entries: u8,          // 0x1ea
    kbd_status: u8,                       // 0x1eb
    secure_boot: u8,                      // 0x1ec
    _pad5: [u8; 2],                       // 0x1ed
    sentinel: u8,                         // 0x1ef
    _pad6: [u8; 1],                       // 0x1f0
    hdr: SetupHeader,                     // 0x1f1
    _pad7: [u8; 0x290 - 0x1f1 - size_of::<SetupHeader>()],
    edd_mbr_sig_buffer: [u32; 16],     // 0x290
    e820_table: [E820Entry; E820_MAX], // 0x2d0
    _pad8: [u8; 48],                   // 0xcd0
    eddbuf: [u8; 0xeec - 0xd00],       // 0xd00
    _pad9: [u8; 276],                  // 0xeec
}

#[repr(C, packed)]
pub struct SetupHeader {
    setup_sects: u8,
    root_flags: u16,
    syssize: u32,
    ram_size: u16,
    vid_mode: u16,
    root_dev: u16,
    boot_flag: u16,
    jump: u16,
    header: u32,
    version: u16,
    realmode_swtch: u32,
    start_sys: u16,
    kernel_version: u16,
    type_of_loader: u8,
    loadflags: u8,
    setup_move_size: u16,
    code32_start: u32,
    ramdisk_image: u32,
    ramdisk_size: u32,
    bootsect_kludge: u32,
    heap_end_ptr: u16,
    ext_loader_ver: u8,
    ext_loader_type: u8,
    cmd_line_ptr: u32,
    initrd_addr_max: u32,
    kernel_alignment: u32,
    relocatable_kernel: u8,
    min_alignment: u8,
    xloadflags: u16,
    cmdline_size: u32,
    hardware_subarch: u32,
    hardware_subarch_data: u64,
    payload_offset: u32,
    payload_length: u32,
    setup_data: u64,
    pref_address: u64,
    init_size: u32,
    handover_offset: u32,
    kernel_info_offset: u32,
}

const HDRS: u32 = 0x53726448;
const MAGIC_AA55: u16 = 0xaa55;

const HEADER_OFFSET: usize = 0x01f1;
//const ENTRY_64: usize = 0x200;

const LOW_MEM_64K: usize = 64 * 1024;
const LOW_MEM_1M: usize = 1 * 1048576;

const XLF_KERNEL_64: u16 = 1 << 0;
const XLF_CAN_BE_LOADED_ABOVE_4G: u16 = 1 << 1;

// The implementation of load_linux64 is inspired by
// https://github.com/akaros/akaros/blob/master/user/vmm/memory.c and
// https://github.com/machyve/xhyve/blob/master/src/firmware/kexec.c
pub struct BzImage {
    pub low_mem_size: u64,
    pub high_mem_start: u64,
    pub high_mem_size: u64,
    pub rom_base: usize,
    pub rom_size: usize,
    pub entry: usize,
}

impl BootParams {
    pub fn new() -> Self {
        // We don't want unsafe,
        // but rust cannot derive Default for T[n] where n > 32
        unsafe { zeroed() }
    }
}

impl BzImage {
    pub fn load(&mut self, w: &mut print::WriteTo) -> Result<usize> {
        // The raw pointer shit is too painful.
        let rom = SectionReader::new(&Memory {}, self.rom_base, self.rom_size);
        let mut header: SetupHeader = {
            let mut buff = [0u8; size_of::<SetupHeader>()];
            rom.pread(&mut buff, HEADER_OFFSET).unwrap();
            unsafe { transmute(buff) }
        };

        // first we make sure that the kernel is not too old

        // from https://www.kernel.org/doc/Documentation/x86/boot.txt:
        // For backwards compatibility, if the setup_sects field contains 0, the
        // real value is 4.
        if header.setup_sects == 0 {
            header.setup_sects = 4;
        }
        // check magic numbers
        if header.boot_flag != MAGIC_AA55 {
            return Err("magic number missing: header.boot_flag != 0xaa55");
        }
        if header.header != HDRS {
            return Err("magic number missing: header.header != 0x53726448");
        }
        // only accept version >= 2.12
        if header.version < 0x020c {
            let version = header.version;
            write!(w, "kernel version too old: 0x{:04x}", version).unwrap();
            return Err("kernel version too old");
        }
        if header.xloadflags | XLF_KERNEL_64 == 0 {
            return Err("kernel has no 64-bit entry point");
        }
        if header.xloadflags | XLF_CAN_BE_LOADED_ABOVE_4G == 0 {
            return Err("kernel cannot be loaded above 4GiB");
        }
        if header.relocatable_kernel == 0 {
            return Err("kernel is not relocatable");
        }

        // calculate offsets
        // bootparam offset
        // can we just stick with 90000 for now?
        //let bp_offset = self.mem_size - PAGE_SIZE;
        //let cmd_line_offset = bp_offset - PAGE_SIZE;

        let mut bp = BootParams::new();
        bp.hdr = header;

        // load kernel
        let mut kernel_offset = (bp.hdr.setup_sects as usize + 1) * 512;

        // Copy from driver into segment.
        let mut buf = [0u8; 512];
        let mut load: usize = 0x1000000;
        loop {
            let size = match rom.pread(&mut buf, kernel_offset) {
                Ok(x) => x,
                _x => break,
            };
            write!(w, "Copy to {:x} for {:x}\n", load, size).unwrap();
            unsafe { copy(buf.as_ptr(), load as *mut u8, size) };
            kernel_offset += size;
            load += size;
        }

        //kernel_file.seek(SeekFrom::Start(kernel_offset))?;
        //kernel_file.read_exact(&mut high_mem[0..(kn_meta.len() - kernel_offset) as usize])?;

        // for now we assume a built-in command line; where else would we get it?
        // // command line
        // if cmd_line.len() > bp.hdr.cmdline_size as usize {
        //     let cmdline_size = bp.hdr.cmdline_size;
        //     return Err(format_args!(
        //         "length of command line exceeds bp.hdr.cmdline_size = {}\n{}",
        //         cmdline_size, cmd_line
        //     ))?;
        // }
        // &high_mem[cmd_line_offset..(cmd_line_offset + cmd_line.len())]
        //     .clone_from_slice(cmd_line.as_bytes());
        // let cmd_line_base = high_mem.start + cmd_line_offset;
        // bp.hdr.cmd_line_ptr = (cmd_line_base & 0xffffffff) as u32;
        // bp.ext_cmd_line_ptr = (cmd_line_base >> 32) as u32;

        // load ramdisk ... someday
        // if let Some(rd_path) = rd_path {
        //     let rd_meta = metadata(&rd_path)?;
        //     let rd_size = rd_meta.len() as usize;
        //     if rd_size > low_mem.size - LOW_MEM_1M {
        //         return Err(format_args!(
        //             "size of ramdisk file {} is too large, limit: {} MiB.",
        //             &rd_path,
        //             low_mem.size / MiB - 1,
        //         ))?;
        //     }
        //     let mut rd_file = File::open(&rd_path)?;
        //     let rd_base = low_mem.size - round_up_4k(rd_size);
        //     rd_file.read_exact(&mut low_mem[rd_base..(rd_base + rd_size)])?;
        //     bp.hdr.ramdisk_image = (rd_base & 0xffffffff) as u32;
        //     bp.ext_ramdisk_image = (rd_base >> 32) as u32;
        //     bp.hdr.ramdisk_size = (rd_size & 0xffffffff) as u32;
        //     bp.ext_ramdisk_size = (rd_size >> 32) as u32;
        //     bp.hdr.root_dev = 0x100;
        // }

        // setup e820 tables

        // The first page is always reserved.
        let entry_first_page = E820Entry { addr: 0, size: PAGE_SIZE as u64, r#type: E820_RESERVED };
        // a tiny bit of low memory for trampoline
        let entry_low = E820Entry { addr: PAGE_SIZE as u64, size: (LOW_MEM_64K - PAGE_SIZE) as u64, r#type: E820_RAM };
        // memory from 64K to LOW_MEM_1M is reserved
        let entry_reserved = E820Entry { addr: LOW_MEM_64K as u64, size: (LOW_MEM_1M - LOW_MEM_64K) as u64, r#type: E820_RESERVED };
        // LOW_MEM_1M to low_mem_size for ramdisk and multiboot
        let entry_low_main = E820Entry { addr: LOW_MEM_1M as u64, size: (self.low_mem_size - LOW_MEM_1M as u64), r#type: E820_RAM };
        // main memory above 4GB
        let entry_main = E820Entry { addr: self.high_mem_start as u64, size: self.high_mem_size as u64, r#type: E820_RAM };
        bp.e820_table[0] = entry_first_page;
        bp.e820_table[1] = entry_low;
        bp.e820_table[2] = entry_reserved;
        bp.e820_table[3] = entry_low_main;
        bp.e820_table[4] = entry_main;
        bp.e820_entries = 5;
        unsafe { copy(&bp, 0x90000 as *mut BootParams, size_of::<BootParams>());}

        Ok(0)
    }
    /// Run the payload. This might not return.
    pub fn run(&self, w: &mut print::WriteTo) {
        // Jump to the payload.
        // See: linux/Documentation/arm/Booting
        unsafe {
            let f = transmute::<usize, EntryPoint>(self.entry);
            write!(w, "on to {:#x}", self.entry).unwrap();
            f(1, 0);
        }
        // TODO: error when payload returns.
    }
}
