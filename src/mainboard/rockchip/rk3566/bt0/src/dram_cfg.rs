// U-Boot
// arch/arm/include/asm/arch-rockchip/sdram_common.h
// arch/arm/include/asm/arch-rockchip/sdram_rv1126.h

// U-Boot: `sdram_cap_info`
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ChannelParams {
    pub rank: u32,
    pub column: u32,
    pub bank_num: u32,
    pub chan_bus_width: u32,
    pub die_bus_width: u32,
    pub row_3_4: u32,
    pub cs0_row: u32,
    pub cs1_row: u32,
    pub cs0_high16bit_row: u32,
    pub cs1_high16bit_row: u32,
    pub ddr_config: u32,
    pub unk1: u32,
    pub unk2: u32,
}

// U-Boot: `sdram_msch_timings`
#[repr(C)]
pub struct MschNocTimings {
    pub ddrtiminga0: u32,
    pub ddrtimingb0: u32,
    pub ddrtimingc0: u32,
    pub ddr4_timing: u32,
    pub devtodev: u32,
    pub ddr_mode: u32,
    pub agingx: u32,
    pub unk3: u32,
    pub unk4: u32,
    pub unk5: u32,
    pub unk6: u32,
    pub unk7: u32,
}

// U-Boot: `sdram_base_params`
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DramParams {
    pub dram_freq: u32,
    pub dram_type: u32,
    pub num_channels: u32,
    pub stride: u32,
    pub odt: u32,
}

#[repr(C)]
pub struct RegVal {
    pub offset: u32,
    pub value: u32,
}

// NOTE: Originally 34 entries, but we removed the end marker and empty ones.
// Other configs may have more, but this reduces our code size and is simpler.
pub type CtrlCfg = [RegVal; 31];

// NOTE: Originally 8 entries, but we removed the end marker and empty ones.
// Other configs may have more, but this reduces our code size and is simpler.
pub type PhyCfg = [RegVal; 4];

#[repr(C)]
pub struct Config {
    pub chan_params: ChannelParams,
    pub timings: MschNocTimings,
    pub dram_params: DramParams,
    pub ctrl_cfg: CtrlCfg,
    pub phy_cfg: PhyCfg,
}

#[derive(Clone, Copy)]
pub struct Params {
    pub chan_params: ChannelParams,
    pub dram_params: DramParams,
}

// ?
pub const LPDDR4_CFG_324: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x2f0d060a,
        ddrtimingb0: 0x6020804,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000144,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000000,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x13002e,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x02013e,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x210000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000202,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x240012,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x100000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0xc070507,
        },
        RegVal {
            offset: 0x000104,
            value: 0x05040b,
        },
        RegVal {
            offset: 0x000108,
            value: 0x4070c0d,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x505000,
        },
        RegVal {
            offset: 0x000110,
            value: 0x3040204,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4050303,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010004,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000301,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000303,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0x100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x00002f,
        },
        RegVal {
            offset: 0x000180,
            value: 0xa200a2,
        },
        RegVal {
            offset: 0x000184,
            value: 0x900000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7040000,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0x905092c,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0xe000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0x8000000,
        },
    ],
};

// 3 ?
pub const LPDDR4_CFG_396: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x3110080d,
        ddrtimingb0: 0x8020804,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x00018c,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000000,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x180038,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x020184,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x280000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000202,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x240012,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x100000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0xd080609,
        },
        RegVal {
            offset: 0x000104,
            value: 0x05040d,
        },
        RegVal {
            offset: 0x000108,
            value: 0x4070c0d,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x505000,
        },
        RegVal {
            offset: 0x000110,
            value: 0x4040205,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4050303,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010004,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000301,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000303,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0x100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x000039,
        },
        RegVal {
            offset: 0x000180,
            value: 0xc600c6,
        },
        RegVal {
            offset: 0x000184,
            value: 0xa00000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7040000,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0x905092c,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0xe000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0x8000000,
        },
    ],
};

// 3 ?
pub const LPDDR4_CFG_528: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x34140b11,
        ddrtimingb0: 0xb030804,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000210,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000000,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x20004a,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x020205,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x350000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000203,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x240012,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x100000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0xe0b090c,
        },
        RegVal {
            offset: 0x000104,
            value: 0x050412,
        },
        RegVal {
            offset: 0x000108,
            value: 0x4070c0d,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x505000,
        },
        RegVal {
            offset: 0x000110,
            value: 0x5040306,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4050404,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010004,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000301,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000404,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0x100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x00004c,
        },
        RegVal {
            offset: 0x000180,
            value: 0x1080108,
        },
        RegVal {
            offset: 0x000184,
            value: 0xe00000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7040000,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0x905092c,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0xe000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0x8000000,
        },
    ],
};

// 3 ?
pub const LPDDR4_CFG_630: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x36170d14,
        ddrtimingb0: 0xd030805,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000276,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x260059,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x020269,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x3f0000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000204,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x240012,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0xf0d0a0e,
        },
        RegVal {
            offset: 0x000104,
            value: 0x050415,
        },
        RegVal {
            offset: 0x000108,
            value: 0x4070d0d,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x505000,
        },
        RegVal {
            offset: 0x000110,
            value: 0x6040407,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4050505,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010004,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000301,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000404,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0x100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x00005b,
        },
        RegVal {
            offset: 0x000180,
            value: 0x13b013b,
        },
        RegVal {
            offset: 0x000184,
            value: 0x1000000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7040000,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xa040b28,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0xe000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0x8000000,
        },
    ],
};

// 3
pub const LPDDR4_CFG_780: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x391b1019,
        ddrtimingb0: 0x10040805,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x00030c,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x2f006e,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x0202fb,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x4e0000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000204,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x240012,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0x10100d11,
        },
        RegVal {
            offset: 0x000104,
            value: 0x050419,
        },
        RegVal {
            offset: 0x000108,
            value: 0x4070c0d,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x606000,
        },
        RegVal {
            offset: 0x000110,
            value: 0x8040409,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4050606,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010004,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000301,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000505,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0x100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x000071,
        },
        RegVal {
            offset: 0x000180,
            value: 0x1860186,
        },
        RegVal {
            offset: 0x000184,
            value: 0x1400000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7040000,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xa040b28,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0xe000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0x8000000,
        },
    ],
};

pub const LPDDR4_CFG_920: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x3e20121d,
        ddrtimingb0: 0x12050a07,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000398,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x380081,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x020384,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x5b0000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000205,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x34001b,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0x12130f14,
        },
        RegVal {
            offset: 0x000104,
            value: 0x06041e,
        },
        RegVal {
            offset: 0x000108,
            value: 0x50a0e0f,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x707000,
        },
        RegVal {
            offset: 0x000110,
            value: 0x904050a,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4060707,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010005,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000401,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000606,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0xa100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x000085,
        },
        RegVal {
            offset: 0x000180,
            value: 0x1cc01cc,
        },
        RegVal {
            offset: 0x000184,
            value: 0x1700000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7070001,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xb050d3c,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0x14000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0xa000000,
        },
    ],
};

pub const LPDDR4_CFG_1056: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x41241522,
        ddrtimingb0: 0x15050b07,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000420,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x400094,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x030409,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x690000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000206,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x34001b,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0x14161217,
        },
        RegVal {
            offset: 0x000104,
            value: 0x0f0422,
        },
        RegVal {
            offset: 0x000108,
            value: 0x50a0e0f,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x808000,
        },
        RegVal {
            offset: 0x000110,
            value: 0xa04060c,
        },
        RegVal {
            offset: 0x000114,
            value: 0xf0f0808,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010005,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000401,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000606,
        },
        RegVal {
            offset: 0x000130,
            value: 0x0f0000,
        },
        RegVal {
            offset: 0x000134,
            value: 0xa100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x000098,
        },
        RegVal {
            offset: 0x000180,
            value: 0x2100210,
        },
        RegVal {
            offset: 0x000184,
            value: 0x1b00000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7070001,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xb050d3c,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0x14000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0xa000000,
        },
    ],
};

pub const LPDDR4_CFG_1184: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x45281825,
        ddrtimingb0: 0x18060c09,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x0004a0,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x4800a6,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x030486,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x750000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000206,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x440024,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0x16181419,
        },
        RegVal {
            offset: 0x000104,
            value: 0x050526,
        },
        RegVal {
            offset: 0x000108,
            value: 0x60c1011,
        },
        RegVal {
            offset: 0x00010c,
            value: 0x909000,
        },
        RegVal {
            offset: 0x000110,
            value: 0xb04060d,
        },
        RegVal {
            offset: 0x000114,
            value: 0x2050909,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010006,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000501,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000707,
        },
        RegVal {
            offset: 0x000130,
            value: 0x020000,
        },
        RegVal {
            offset: 0x000134,
            value: 0xb100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x0000ab,
        },
        RegVal {
            offset: 0x000180,
            value: 0x2500250,
        },
        RegVal {
            offset: 0x000184,
            value: 0x1e00000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7090002,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xc060f48,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0x18000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0xc000000,
        },
    ],
};

pub const LPDDR4_CFG_1332: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x482c1b2a,
        ddrtimingb0: 0x1b070d09,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000534,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x5100bb,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x030516,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x840000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000207,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x440024,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0x171b161c,
        },
        RegVal {
            offset: 0x000104,
            value: 0x07052a,
        },
        RegVal {
            offset: 0x000108,
            value: 0x60c1012,
        },
        RegVal {
            offset: 0x00010c,
            value: 0xa0a000,
        },
        RegVal {
            offset: 0x000110,
            value: 0xc04070e,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4070a0a,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010006,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000501,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000707,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0xb100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x0000c0,
        },
        RegVal {
            offset: 0x000180,
            value: 0x29a029a,
        },
        RegVal {
            offset: 0x000184,
            value: 0x2200000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x7090002,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xc060f48,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0x18000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0xc000000,
        },
    ],
};

pub const LPDDR4_CFG_1560: Config = Config {
    chan_params: ChannelParams {
        rank: 0x000001,
        column: 0x00000b,
        bank_num: 0x000003,
        chan_bus_width: 0x000001,
        die_bus_width: 0x000001,
        row_3_4: 0x000000,
        cs0_row: 0x000011,
        cs1_row: 0x000011,
        cs0_high16bit_row: 0x000000,
        cs1_high16bit_row: 0x000000,
        ddr_config: 0x000000,
        unk1: 0x000000,
        unk2: 0x000000,
    },
    timings: MschNocTimings {
        ddrtiminga0: 0x4f342131,
        ddrtimingb0: 0x1f080f0a,
        ddrtimingc0: 0x000c04,
        ddr4_timing: 0x000000,
        devtodev: 0x001111,
        ddr_mode: 0x000054,
        agingx: 0x0000ff,
        unk3: 0x000000,
        unk4: 0x000000,
        unk5: 0x000000,
        unk6: 0x000000,
        unk7: 0x000000,
    },
    dram_params: DramParams {
        dram_freq: 0x000618,
        dram_type: 0x000007,
        num_channels: 0x000001,
        stride: 0x000000,
        odt: 0x000001,
    },
    ctrl_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x83081020,
        },
        RegVal {
            offset: 0x000064,
            value: 0x5f00db,
        },
        RegVal {
            offset: 0x0000d0,
            value: 0x0305f5,
        },
        RegVal {
            offset: 0x0000d4,
            value: 0x9a0000,
        },
        RegVal {
            offset: 0x0000d8,
            value: 0x000208,
        },
        RegVal {
            offset: 0x0000dc,
            value: 0x54002d,
        },
        RegVal {
            offset: 0x0000e0,
            value: 0x310000,
        },
        RegVal {
            offset: 0x0000e8,
            value: 0x110000,
        },
        RegVal {
            offset: 0x0000ec,
            value: 0x000000,
        },
        RegVal {
            offset: 0x0000f4,
            value: 0x0f022f,
        },
        RegVal {
            offset: 0x000100,
            value: 0x1a201a21,
        },
        RegVal {
            offset: 0x000104,
            value: 0x080632,
        },
        RegVal {
            offset: 0x000108,
            value: 0x70e1114,
        },
        RegVal {
            offset: 0x00010c,
            value: 0xb0b000,
        },
        RegVal {
            offset: 0x000110,
            value: 0xf040811,
        },
        RegVal {
            offset: 0x000114,
            value: 0x4080c0c,
        },
        RegVal {
            offset: 0x000118,
            value: 0x1010007,
        },
        RegVal {
            offset: 0x00011c,
            value: 0x000601,
        },
        RegVal {
            offset: 0x000120,
            value: 0x000909,
        },
        RegVal {
            offset: 0x000130,
            value: 0x040000,
        },
        RegVal {
            offset: 0x000134,
            value: 0xc100002,
        },
        RegVal {
            offset: 0x000138,
            value: 0x0000e1,
        },
        RegVal {
            offset: 0x000180,
            value: 0x30c030c,
        },
        RegVal {
            offset: 0x000184,
            value: 0x2700000,
        },
        RegVal {
            offset: 0x000190,
            value: 0x70b0003,
        },
        RegVal {
            offset: 0x000198,
            value: 0xa000101,
        },
        RegVal {
            offset: 0x0001a0,
            value: 0xc0400003,
        },
        RegVal {
            offset: 0x000240,
            value: 0xd071154,
        },
        RegVal {
            offset: 0x000244,
            value: 0x000101,
        },
        RegVal {
            offset: 0x000250,
            value: 0x001f00,
        },
        RegVal {
            offset: 0x000490,
            value: 0x000001,
        },
    ],
    phy_cfg: [
        RegVal {
            offset: 0x000000,
            value: 0x001fd7,
        },
        RegVal {
            offset: 0x000008,
            value: 0x000000,
        },
        RegVal {
            offset: 0x00000c,
            value: 0x1c000000,
        },
        RegVal {
            offset: 0x000010,
            value: 0xe000000,
        },
    ],
};
