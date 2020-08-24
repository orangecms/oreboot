/* SPDX-License-Identifier: GPL-2.0-only */

use crate::bios::setup_bios_tables;
use crate::consts::msr::*;
use crate::consts::x86::*;
use crate::consts::*;
use crate::err::Error;
use crate::hv::ffi::*;
use crate::hv::vmx::*;
use crate::hv::*;
use crate::mach::MachVMBlock;
use crate::utils::round_up_4k;
use crate::{GuestThread, VirtualMachine};
use crossbeam_channel::unbounded as channel;
#[allow(unused_imports)]
use log::*;
use std::fs::{metadata, File};
use std::io::{Read, Seek, SeekFrom};
use std::mem::{size_of, transmute, zeroed};
use std::sync::Arc;

pub const E820_RAM: u32 = 1;
pub const E820_RESERVED: u32 = 2;
pub const E820_ACPI: u32 = 3;
pub const E820_NVS: u32 = 4;
pub const E820_UNUSABLE: u32 = 5;

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

impl BootParams {
    pub fn new() -> Self {
        // We don't want unsafe,
        // but rust cannot derive Default for T[n] where n > 32
        unsafe { zeroed() }
    }
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

const HEADER_OFFSET: u64 = 0x01f1;
const ENTRY_64: usize = 0x200;

const LOW_MEM_64K: usize = 64 * KiB;
const LOW_MEM_1M: usize = 1 * MiB;

const XLF_KERNEL_64: u16 = 1 << 0;
const XLF_CAN_BE_LOADED_ABOVE_4G: u16 = 1 << 1;

// The implementation of load_linux64 is inspired by
// https://github.com/akaros/akaros/blob/master/user/vmm/memory.c and
// https://github.com/machyve/xhyve/blob/master/src/firmware/kexec.c

pub fn load_linux64(
    vm: &Arc<VirtualMachine>,
    kernel_path: String,
    rd_path: Option<String>,
    mut cmd_line: String,
    mem_size: usize,
) -> Result<Vec<GuestThread>, Error> {
    let kn_meta = metadata(&kernel_path)?;
    let mut kernel_file = File::open(&kernel_path)?;
    let mut header: SetupHeader = {
        let mut buff = [0u8; size_of::<SetupHeader>()];
        kernel_file.seek(SeekFrom::Start(HEADER_OFFSET))?;
        kernel_file.read_exact(&mut buff)?;
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
        return Err("magic number missing: header.boot_flag != 0xaa55".to_string())?;
    }
    if header.header != HDRS {
        return Err("magic number missing: header.header != 0x53726448".to_string())?;
    }
    // only accept version >= 2.12
    if header.version < 0x020c {
        let version = header.version;
        return Err(format!("kernel version too old: 0x{:04x}", version))?;
    }
    if header.xloadflags | XLF_KERNEL_64 == 0 {
        return Err("kernel has no 64-bit entry point".to_string())?;
    }
    if header.xloadflags | XLF_CAN_BE_LOADED_ABOVE_4G == 0 {
        return Err("kernel cannot be loaded above 4GiB".to_string())?;
    }
    if header.relocatable_kernel == 0 {
        return Err("kernel is not relocatable".to_string())?;
    }

    let num_gth = vm.cores;
    // setup low memory
    let mut low_mem; // = vm.low_mem.as_ref().unwrap().write().unwrap();
    if let Some(mem) = vm.low_mem.as_ref() {
        low_mem = mem.write().unwrap();
        // let mut mem = low_mem.write().unwrap();
        setup_bios_tables(0xe0000, &mut low_mem, num_gth);
    } else {
        return Err("Linux requires low memory".to_string())?;
    }

    // command line
    for virtio_dev in vm.virtio_mmio_devices.iter() {
        let mmio_dev = virtio_dev.lock().unwrap();
        let virtio_para = format!(
            " virtio_mmio.device=1K@0x{:x}:{}",
            mmio_dev.addr, mmio_dev.dev.irq
        );
        cmd_line.push_str(&virtio_para);
    }

    // calculate offsets
    let bp_offset = mem_size - PAGE_SIZE;
    let cmd_line_offset = bp_offset - PAGE_SIZE;
    let gdt_offset = cmd_line_offset - PAGE_SIZE;
    let pml4_offset = gdt_offset - PAGE_SIZE;
    let first_pdpt_offset = pml4_offset - PAGE_SIZE;

    let mut high_mem = MachVMBlock::new_aligned(mem_size, header.kernel_alignment as usize)?;

    let mut bp = BootParams::new();
    bp.hdr = header;

    // load kernel
    let kernel_offset = (bp.hdr.setup_sects as u64 + 1) * 512;
    kernel_file.seek(SeekFrom::Start(kernel_offset))?;
    kernel_file.read_exact(&mut high_mem[0..(kn_meta.len() - kernel_offset) as usize])?;

    // command line
    if cmd_line.len() > bp.hdr.cmdline_size as usize {
        let cmdline_size = bp.hdr.cmdline_size;
        return Err(format!(
            "length of command line exceeds bp.hdr.cmdline_size = {}\n{}",
            cmdline_size, cmd_line
        ))?;
    }
    &high_mem[cmd_line_offset..(cmd_line_offset + cmd_line.len())]
        .clone_from_slice(cmd_line.as_bytes());
    let cmd_line_base = high_mem.start + cmd_line_offset;
    bp.hdr.cmd_line_ptr = (cmd_line_base & 0xffffffff) as u32;
    bp.ext_cmd_line_ptr = (cmd_line_base >> 32) as u32;

    // load ramdisk
    if let Some(rd_path) = rd_path {
        let rd_meta = metadata(&rd_path)?;
        let rd_size = rd_meta.len() as usize;
        if rd_size > low_mem.size - LOW_MEM_1M {
            return Err(format!(
                "size of ramdisk file {} is too large, limit: {} MiB.",
                &rd_path,
                low_mem.size / MiB - 1,
            ))?;
        }
        let mut rd_file = File::open(&rd_path)?;
        let rd_base = low_mem.size - round_up_4k(rd_size);
        rd_file.read_exact(&mut low_mem[rd_base..(rd_base + rd_size)])?;
        bp.hdr.ramdisk_image = (rd_base & 0xffffffff) as u32;
        bp.ext_ramdisk_image = (rd_base >> 32) as u32;
        bp.hdr.ramdisk_size = (rd_size & 0xffffffff) as u32;
        bp.ext_ramdisk_size = (rd_size >> 32) as u32;
        bp.hdr.root_dev = 0x100;
    }

    // setup e820 tables

    // The first page is always reserved.
    let entry_first_page = E820Entry {
        addr: 0,
        size: PAGE_SIZE as u64,
        r#type: E820_RESERVED,
    };
    // a tiny bit of low memory for trampoline
    let entry_low = E820Entry {
        addr: PAGE_SIZE as u64,
        size: (LOW_MEM_64K - PAGE_SIZE) as u64,
        r#type: E820_RAM,
    };
    // memory from 64K to LOW_MEM_1M is reserved
    let entry_reserved = E820Entry {
        addr: LOW_MEM_64K as u64,
        size: (LOW_MEM_1M - LOW_MEM_64K) as u64,
        r#type: E820_RESERVED,
    };
    // LOW_MEM_1M to low_mem_size for ramdisk and multiboot
    let entry_low_main = E820Entry {
        addr: LOW_MEM_1M as u64,
        size: (low_mem.size - LOW_MEM_1M) as u64,
        r#type: E820_RAM,
    };
    // main memory above 4GB
    let entry_main = E820Entry {
        addr: high_mem.start as u64,
        size: high_mem.size as u64,
        r#type: E820_RAM,
    };
    bp.e820_table[0] = entry_first_page;
    bp.e820_table[1] = entry_low;
    bp.e820_table[2] = entry_reserved;
    bp.e820_table[3] = entry_low_main;
    bp.e820_table[4] = entry_main;
    bp.e820_entries = 5;
    high_mem.write(bp, bp_offset, 0);

    // setup gdt
    let gdt_entries: [u64; 4] = [0, 0, 0x00af9a000000ffff, 0x00cf92000000ffff];
    high_mem.write(gdt_entries, gdt_offset, 0);

    // identity paging
    let pml4e_base = high_mem.start + pml4_offset;
    let first_pdpt_base = high_mem.start + first_pdpt_offset;
    let pml4e: u64 = PG_P | PG_RW | first_pdpt_base as u64;
    high_mem.write(pml4e, pml4_offset, 0);
    for i in 0..512 {
        let pdpte: u64 = (i << 30) | PG_P | PG_RW | PG_PS;
        high_mem.write(pdpte, first_pdpt_offset, i as usize);
    }

    // setup initial register values and VMCS fields
    let init_regs = vec![
        (X86Reg::CR3, pml4e_base as u64),
        (X86Reg::GDT_BASE, (high_mem.start + gdt_offset) as u64),
        (X86Reg::GDT_LIMIT, 0x1f),
        (X86Reg::RFLAGS, FL_RSVD_1),
        (X86Reg::RSI, (high_mem.start + bp_offset) as u64),
        (X86Reg::RIP, (high_mem.start + ENTRY_64) as u64),
    ]
    .into_iter()
    .collect();

    let ctrl_pin = gen_exec_ctrl(vmx_read_capability(VMXCap::Pin)?, 0, 0);
    let ctrl_cpu = gen_exec_ctrl(
        vmx_read_capability(VMXCap::CPU)?,
        CPU_BASED_CR8_LOAD | CPU_BASED_CR8_STORE,
        0,
    );
    let ctrl_cpu2 = gen_exec_ctrl(
        vmx_read_capability(VMXCap::CPU2)?,
        CPU_BASED2_RDTSCP | CPU_BASED2_INVPCID,
        0,
    );
    let ctrl_entry = gen_exec_ctrl(vmx_read_capability(VMXCap::Entry)?, VMENTRY_GUEST_IA32E, 0);
    let cr0 = X86_CR0_NE | X86_CR0_PE | X86_CR0_PG;
    let cr4 = X86_CR4_VMXE | X86_CR4_PAE;
    let efer = EFER_LMA | EFER_LME;
    let init_vmcs = vec![
        (VMCS_GUEST_CS_AR, 0xa09b),
        (VMCS_GUEST_CS_LIMIT, 0xffffffff),
        (VMCS_GUEST_DS_AR, 0xc093),
        (VMCS_GUEST_DS_LIMIT, 0xffffffff),
        (VMCS_GUEST_ES_AR, 0xc093),
        (VMCS_GUEST_ES_LIMIT, 0xffffffff),
        (VMCS_GUEST_FS_AR, 0x93),
        (VMCS_GUEST_GS_AR, 0x93),
        (VMCS_GUEST_SS_AR, 0xc093),
        (VMCS_GUEST_SS_LIMIT, 0xffffffff),
        (VMCS_GUEST_LDTR_AR, 0x82),
        (VMCS_GUEST_TR_AR, 0x8b),
        (VMCS_CTRL_PIN_BASED, ctrl_pin),
        (VMCS_CTRL_CPU_BASED, ctrl_cpu),
        (VMCS_CTRL_CPU_BASED2, ctrl_cpu2),
        (VMCS_CTRL_VMENTRY_CONTROLS, ctrl_entry),
        (VMCS_CTRL_EXC_BITMAP, 0xffffffff & !(1 << 14) & !(1 << 3)), // currently we track all exceptions except #BP and #PF.
        (VMCS_GUEST_CR0, cr0),
        (VMCS_CTRL_CR0_MASK, X86_CR0_PE | X86_CR0_PG),
        (VMCS_CTRL_CR0_SHADOW, cr0),
        (VMCS_GUEST_CR4, cr4),
        (VMCS_CTRL_CR4_MASK, X86_CR4_VMXE),
        (VMCS_CTRL_CR4_SHADOW, 0),
        (VMCS_GUEST_IA32_EFER, efer),
    ]
    .into_iter()
    .collect();

    vm.mem_space.write().unwrap().map(
        high_mem.start,
        high_mem.start,
        high_mem.size,
        HV_MEMORY_READ | HV_MEMORY_WRITE | HV_MEMORY_EXEC,
    )?;

    vm.high_mem.write().unwrap().push(high_mem);
    let mut guest_threads = Vec::with_capacity(num_gth as usize);
    let mut vector_senders = Vec::with_capacity(num_gth as usize);

    for i in 0..num_gth {
        let (sender, receiver) = channel();
        vector_senders.push(sender);
        let mut gth = GuestThread::new(vm, i);
        gth.vector_receiver = Some(receiver);
        guest_threads.push(gth);
    }
    *vm.vector_senders.lock().unwrap() = Some(vector_senders);

    // guest thread 0 is the BSP, bootstrap processor
    guest_threads[0].init_regs = init_regs;
    guest_threads[0].init_vmcs = init_vmcs;

    Ok(guest_threads)
}
