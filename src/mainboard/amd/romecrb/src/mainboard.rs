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
use x86_64::instructions::{rdmsr, wrmsr};

const FCH_UART_LEGACY_DECODE: u32 = 0xfedc0020;
const FCH_LEGACY_3F8_SH: u16 = 1 << 3;

fn poke16(a: u32, v: u16) -> () {
    let y = a as *mut u16;
    unsafe {
        ptr::write_volatile(y, v);
    }
}

fn poke32(a: u32, v: u32) -> () {
    let y = a as *mut u32;
    unsafe {
        ptr::write_volatile(y, v);
    }
}

fn peek32(a: u32) -> u32 {
    let y = a as *mut u32;
    unsafe {
        return ptr::read_volatile(y);
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
        // Knowledge from coreboot to get minimal serial working.
        // GPIO defaults are fine.
        // clock default is NOT fine.
        // Need to set it to 8 mhz.
        // this should fuck up uart output but we'll see.
        //uart_ctrl = sm_pci_read32(SMB_UART_CONFIG);
        //uart_ctrl |= 1 << (SMB_UART_1_8M_SHIFT + idx);
        //sm_pci_write32(SMB_UART_CONFIG, uart_ctrl);
        // FED8000 is the basic MMIO space.
        // fed800fc is the uart control reg.
        // bit 28 is the bit which sets it between 48m and 1.8m
        // we want 1.8m. They made oddball 48m default. Stupid.
        let mut uc = peek32(0xfed800fc);
        uc = uc | (1 << 28);
        poke32(0xfed800fc, uc);
        // Set up the legacy decode.
        poke16(FCH_UART_LEGACY_DECODE, FCH_LEGACY_3F8_SH);
        unsafe {
            let v = rdmsr(0x1b) | 0x900;
            wrmsr(0x1b, v);
            let v = rdmsr(0x1b) | 0xd00;
            wrmsr(0x1b, v);
        }
        // IOAPIC
        //     wmem fed80300 e3070b77
        //    wmem fed00010 3
        poke32(0xfed80300, 0xe3070b77);
        poke32(0xfed00010, 3);
        let i = peek32(0xfed00010);
        poke32(0xfed00010, i | 8);
        // THis is likely not needed but.
        //poke32(0xfed00108, 0x5b03d997);

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
