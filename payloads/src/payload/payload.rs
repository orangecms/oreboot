use core::fmt::Write;
use core::intrinsics::{copy, transmute};
use model::{Driver, EOF};
use print;
use wrappers::{Memory, SectionReader};
pub type EntryPoint = unsafe extern "C" fn(r0: usize, dtb: usize);

/// compression types
#[derive(PartialEq, Debug)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum ctype {
    CBFS_COMPRESS_NONE = 0,
    CBFS_COMPRESS_LZMA = 1,
    CBFS_COMPRESS_LZ4 = 2,
}

/// cbfs file attrs
#[derive(PartialEq, Debug)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum fattr {
    CBFS_FILE_ATTR_TAG_UNUSED = 0,
    CBFS_FILE_ATTR_TAG_UNUSED2 = 0xffffffff,
    CBFS_FILE_ATTR_TAG_COMPRESSION = 0x42435a4c,
    CBFS_FILE_ATTR_TAG_HASH = 0x68736148,
    CBFS_FILE_ATTR_TAG_POSITION = 0x42435350,  // PSCB
    CBFS_FILE_ATTR_TAG_ALIGNMENT = 0x42434c41, // ALCB
}

/// cbfs architecture types,
#[derive(PartialEq, Debug)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum atype {
    CBFS_ARCHITECTURE_UNKNOWN = 0xFFFFFFFF,
    CBFS_ARCHITECTURE_X86 = 0x00000001,
    CBFS_ARCHITECTURE_ARM = 0x00000010,
}

/// cbfs header types,
#[derive(PartialEq, Debug)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum htype {
    CBFS_HEADER_MAGIC = 0x4F524243,
    CBFS_HEADER_VERSION1 = 0x31313131,
    CBFS_HEADER_VERSION2 = 0x31313132,
}

/// cbfs file types
#[derive(PartialEq, Debug)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum ftype {
    CBFS_TYPE_DELETED = 0x00000000,
    CBFS_TYPE_DELETED2 = 0xffffffff,
    CBFS_TYPE_STAGE = 0x10,
    CBFS_TYPE_SELF = 0x20,
    CBFS_TYPE_FIT = 0x21,
    CBFS_TYPE_OPTIONROM = 0x30,
    CBFS_TYPE_BOOTSPLASH = 0x40,
    CBFS_TYPE_RAW = 0x50,
    CBFS_TYPE_VSA = 0x51,
    CBFS_TYPE_MBI = 0x52,
    CBFS_TYPE_MICROCODE = 0x53,
    CBFS_TYPE_FSP = 0x60,
    CBFS_TYPE_MRC = 0x61,
    CBFS_TYPE_MMA = 0x62,
    CBFS_TYPE_EFI = 0x63,
    CBFS_TYPE_STRUCT = 0x70,
    CBFS_COMPONENT_CMOS_DEFAULT = 0xaa,
    CBFS_TYPE_SPD = 0xab,
    CBFS_TYPE_MRC_CACHE = 0xac,
    CBFS_COMPONENT_CMOS_LAYOUT = 0x01aa,
}

/// Payload segments types
#[derive(PartialEq, Debug)]
#[allow(non_camel_case_types)]
#[repr(u32)]
pub enum stype {
    PAYLOAD_SEGMENT_CODE = 0x434F4445,
    PAYLOAD_SEGMENT_DATA = 0x44415441,
    PAYLOAD_SEGMENT_BSS = 0x42535320,
    PAYLOAD_SEGMENT_PARAMS = 0x50415241,
    PAYLOAD_SEGMENT_ENTRY = 0x454E5452,
    PAYLOAD_SEGMENT_DTB = 0x44544220,
    PAYLOAD_SEGMENT_BAD = 0xFFFFFFFF,
    CBFS_SEGMENT_CODE = 0x45444F43,
    CBFS_SEGMENT_DATA = 0x41544144,
    CBFS_SEGMENT_BSS = 0x20535342,
    CBFS_SEGMENT_PARAMS = 0x41524150,
    CBFS_SEGMENT_ENTRY = 0x52544E45,
}

// TODO:
// Maybe do what they suggest here? https://enodev.fr/posts/rusticity-convert-an-integer-to-an-enum.html
// I give up.
impl From<u32> for stype {
    fn from(s: u32) -> Self {
        match s {
            0x434F4445 => stype::PAYLOAD_SEGMENT_CODE,
            0x44415441 => stype::PAYLOAD_SEGMENT_DATA,
            0x42535320 => stype::PAYLOAD_SEGMENT_BSS,
            0x50415241 => stype::PAYLOAD_SEGMENT_PARAMS,
            0x454E5452 => stype::PAYLOAD_SEGMENT_ENTRY,
            0x44544220 => stype::PAYLOAD_SEGMENT_DTB,
            0x45444F43 => stype::CBFS_SEGMENT_CODE,
            0x41544144 => stype::CBFS_SEGMENT_DATA,
            0x20535342 => stype::CBFS_SEGMENT_BSS,
            0x41524150 => stype::CBFS_SEGMENT_PARAMS,
            0x52544E45 => stype::CBFS_SEGMENT_ENTRY,
            _ => stype::PAYLOAD_SEGMENT_BAD,
        }
    }
}
/// A payload. oreboot will only have payloads for anything past the romstage.
/// N.B. This struct is NOT designed to be deserialized.
// #[derive(Debug)]
pub struct Payload<'a> {
    /// Type of payload
    pub typ: ftype,
    /// Compression type
    pub compression: ctype,
    /// Offset in ROM
    pub offset: usize,
    /// Physical load address
    pub entry: usize,
    /// the dtb
    pub dtb: usize,
    /// Length in ROM
    pub rom_len: usize,
    /// Length in memory (i.e. once uncompressed)
    pub mem_len: usize,
    /// Segments
    pub segs: &'a [Segment<'a>],
}

/// A payload. oreboot will only have payloads for anything past the romstage.
/// N.B. This struct is NOT designed to be deserialized.
// #[derive(Debug)]
pub struct StreamPayload {
    /// base of rom
    pub rom: usize,
    /// Type of payload
    pub typ: ftype,
    /// Compression type
    pub compression: ctype,
    /// Offset in ROM
    pub offset: usize,
    /// Physical load address
    pub entry: usize,
    /// the dtb
    pub dtb: usize,
    /// Length in ROM
    pub rom_len: usize,
    /// Length in memory (i.e. once uncompressed)
    pub mem_len: usize,
}

// #[derive(Debug)]
pub struct Segment<'a> {
    /// Type
    pub typ: stype,
    /// Load address in memory
    pub base: usize,
    /// The data
    pub data: &'a mut dyn Driver,
}

#[derive(Debug)]
struct CBFSSeg {
    typ: u32,
    comp: u32,
    off: u32,
    load: u64,
    len: u32,
    memlen: u32,
}

// Stream payloads copy segments one at a time to memory.
// This avoids the problem in coreboot (and in Rust) where we have
// to pre-declare a fixed-size array and hope it's big enough.
// TOOD: remove all uses of non-streaming payloads.
impl StreamPayload {
    /// Load the payload in memory. Returns the entrypoint.
    pub fn load(&mut self, w: &mut print::WriteTo) {
        // TODO: how many segments are there?
        // The coreboot convention: ENTRY marks the last segment.
        // we need to ensure we create them that way too.
        let mut hdr: usize = 0;
        write!(w, "loading ...\n").unwrap();
        loop {
            write!(w, "decode header at {:x?}\n", hdr).unwrap();
            let v = &mut [0u8; 28];
            let rom = SectionReader::new(&Memory {}, self.rom + hdr, 28);
            hdr += 28;
            write!(w, "decode header now at {:x?}\n", hdr).unwrap();
            rom.pread(v, 0).unwrap();
            for i in 0..28 {
                write!(w, "{} {:x}\r\n", i, v[i]).unwrap();
            }
            // Serialize in the cbfs struct.
            // This code works on be and le.
            // https://commandcenter.blogspot.com/2012/04/byte-order-fallacy.html
            // I've probably used every way to do this that exists in the last 40 years.
            // In the end, this looks ugly but is clearer in most ways than the others.
            // This is the only place in here that might use reflection. Doing it this way
            // gives me a 40484 byte binary, which is a bit better than below.
            // Most interestingly, the code turns into 32- and 64- bit loads and stores:
            // we first noticed gcc and llvm doing this optimization 5 years ago, and rust
            // does it too.
            // The only sad part is that rustfmt doesn't line things up nicely. Oh well.
            // Oh, yeah, it also works correctly on 64-bit systems and the postcard stuff failed badly.
            let seg = CBFSSeg {
                // The type is little endian (currently)
                typ: ((v[0 + 3] as u32) << 24) | ((v[0 + 2] as u32) << 16) | ((v[0 + 1] as u32) << 8) | ((v[0 + 0] as u32) << 0),
                // Other fields are big-endian (currently)
                comp: ((v[4 + 0] as u32) << 24) | ((v[4 + 1] as u32) << 16) | ((v[4 + 2] as u32) << 8) | ((v[4 + 3] as u32) << 0),
                off: ((v[8 + 0] as u32) << 24) | ((v[8 + 1] as u32) << 16) | ((v[8 + 2] as u32) << 8) | ((v[8 + 3] as u32) << 0),
                load: ((v[12 + 0] as u64) << 56) | ((v[12 + 1] as u64) << 48) | ((v[12 + 2] as u64) << 40) | ((v[12 + 3] as u64) << 32) | ((v[12 + 4] as u64) << 24) | ((v[12 + 5] as u64) << 16) | ((v[12 + 6] as u64) << 8) | ((v[12 + 7] as u64) << 0),
                len: ((v[20 + 0] as u32) << 24) | ((v[20 + 1] as u32) << 16) | ((v[20 + 2] as u32) << 8) | ((v[20 + 3] as u32) << 0),
                memlen: ((v[24 + 0] as u32) << 24) | ((v[24 + 1] as u32) << 16) | ((v[24 + 2] as u32) << 8) | ((v[24 + 3] as u32) << 0),
            };

            let typ: stype = core::convert::From::from(seg.typ);
            // Better minds than mine can figure this shit out. Or when I learn more.
            // Size with this: 42068
            // let typ: stype = core::convert::From::from(seg.typ);
            match typ {
                stype::CBFS_SEGMENT_ENTRY | stype::CBFS_SEGMENT_CODE | stype::CBFS_SEGMENT_DATA | stype::CBFS_SEGMENT_BSS | stype::CBFS_SEGMENT_PARAMS => {
                    write!(w, "cbfs seg {:x?}\n", seg).unwrap();
                }
                stype::PAYLOAD_SEGMENT_BAD => {
                    panic!("Panic'ing on PAYLOAD_SEGMENT_BAD: seg now {:x?} {:x?} typ {:x?}", self.rom, seg, typ);
                }
                _ => {
                    write!(w, "Seg is unchanged: {:x?}\n", seg).unwrap();
                }
            }

            let mut load = seg.load as usize;

            // Copy from driver into segment.
            let mut buf = [0u8; 512];
            match typ {
                // in cbfs, this is always the LAST segment.
                // We should continue the convention.
                stype::CBFS_SEGMENT_ENTRY => {
                    write!(w, "ENTRY {:x?}\n", load).unwrap();
                    self.entry = load;
                    return;
                }
                stype::PAYLOAD_SEGMENT_DTB => self.dtb = load,
                stype::CBFS_SEGMENT_DATA | stype::CBFS_SEGMENT_CODE => {
                    write!(w, "set up from at {:x}\n", self.rom + seg.off as usize).unwrap();
                    if seg.comp != 0 {
                        panic!("We don't do uncompress!!!!");
                    }
                    let data = SectionReader::new(&Memory {}, self.rom + seg.off as usize, seg.len as usize);
                    let mut i: usize = 0;
                    loop {
                        let size = match data.pread(&mut buf, i) {
                            Ok(x) => x,
                            EOF => break,
                            _ => panic!("driver error"),
                        };
                        write!(w, "Copy to {:x} for {:x}\n", load, size).unwrap();
                        unsafe { copy(buf.as_ptr(), load as *mut u8, size) };
                        i += size;
                        load += size;
                    }
                }
                _ => panic!("fix payload loader {} {:x}", self.dtb, seg.typ),
            }
        }
    }

    /// Run the payload. This might not return.
    pub fn run(&self, w: &mut print::WriteTo) {
        // Jump to the payload.
        // See: linux/Documentation/arm/Booting
        unsafe {
            let f = transmute::<usize, EntryPoint>(self.entry);
            write!(w, "on to {:#x}", self.entry).unwrap();
            f(1, self.dtb);
        }
        // TODO: error when payload returns.
    }
}

// to be deprecated
impl<'a> Payload<'a> {
    /// Load the payload in memory. Returns the entrypoint.
    pub fn load(&mut self) {
        // Copy the segments into RAM.
        for s in self.segs {
            // Copy from driver into segment.
            let mut buf = [0u8; 512];
            let mut off = 0;
            if s.typ == stype::PAYLOAD_SEGMENT_ENTRY {
                self.entry = s.base;
            }
            if self.dtb == 0 && s.typ == stype::PAYLOAD_SEGMENT_DATA {
                self.dtb = s.base
            }
            loop {
                let size = match s.data.pread(&mut buf, off) {
                    Ok(x) => x,
                    EOF => break,
                    _ => panic!("driver error"),
                };
                // TODO: This incurs a second copy. There's probably a better way.
                unsafe { copy(buf.as_ptr(), (s.base + off) as *mut u8, size) };
                off += size;
            }
        }
    }

    /// Run the payload. This might not return.
    pub fn run(&self) {
        // Jump to the payload.
        // See: linux/Documentation/arm/Booting
        unsafe {
            let f = transmute::<usize, EntryPoint>(self.entry);
            f(1, self.dtb);
        }
        // TODO: error when payload returns.
    }
}
