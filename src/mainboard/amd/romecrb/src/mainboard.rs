/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2020 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

use clock::ClockNode;
use core::ptr;
use model::*;
use wrappers::Memory;

const FCH_UART_LEGACY_DECODE: u32 = 0xfedc0020;
const FCH_LEGACY_3F8_SH: u16 = 1 << 3;

fn poke16(a: u32, v: u16) -> () {
    let y = a as *mut u16;
    unsafe {
        ptr::write_volatile(y, v);
    }
}

// WIP: mainboard driver. I mean the concept is a WIP.
pub struct MainBoard {}

impl MainBoard {
    pub fn new() -> MainBoard {
        MainBoard {}
    }
}

impl Driver for MainBoard {
    fn init(&mut self) -> Result<()> {
        let cbfs = [
            0x4cu8, 0x42u8, 0x49u8, 0x4fu8, 0x18u8, 0x00u8, 0x00u8, 0x00u8, 0xcfu8, 0x17u8, 0x00u8, 0x00u8, 0x88u8, 0x02u8, 0x00u8, 00u8, 0xe5u8, 0x53u8, 0x00u8, 0x00u8, 0x16u8, 0x00u8, 0x00u8, 0x00u8, 0x01u8, 0x00u8, 0x00u8, 0x00u8, 0x80u8, 0x00u8,
            0x00u8, 00u8,
        ];
        // Knowledge from coreboot to get minimal serial working.
        // GPIO defaults are fine.
        // clock default is fine.
        // The only thing we need is to set up the legacy decode.
        poke16(FCH_UART_LEGACY_DECODE, FCH_LEGACY_3F8_SH);
        let ram_cbfs = &mut Memory {};
        ram_cbfs.pwrite(&cbfs, 0x10000).unwrap();
        Ok(())
    }

    fn pread(&self, _data: &mut [u8], _offset: usize) -> Result<usize> {
        return Ok(0);
    }

    fn pwrite(&mut self, _data: &[u8], _offset: usize) -> Result<usize> {
        Ok(_data.len())
    }

    fn shutdown(&mut self) {}
}

impl ClockNode for MainBoard {
    // This uses hfclk as the input rate.
    fn set_clock_rate(&mut self, _rate: u32) {}
}
