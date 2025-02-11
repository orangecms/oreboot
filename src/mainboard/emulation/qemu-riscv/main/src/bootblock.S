/*
 * Early initialization code for RISC-V
 *
 * Copyright 2013 Google Inc.
 * Copyright 2016 Jonathan Neuschäfer <j.neuschaefer@gmx.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 */

.section ".bootblock.boot", "ax", %progbits

.globl _boot
_boot:
	# The previous boot stage passes these variables:
	#   a0: hartid
	#   a1: ROM FDT

	# TODO: Support SMP.
	# Spin all harts other than 0.
	csrr t0, mhartid
spin:
	bgt t0, x0, spin

	# Set the stack pointer
	# 0xDEADBEEF is used to check stack overflow
1:
  auipc t0,     %pcrel_hi(_stack_ptr)
  lwu sp, %pcrel_lo(1b)(t0)

	li t0, 0xDEADBEEF
	sw t0, 0(sp)

	# Pass device tree as a0 to Rust code.
	add a0, a1, x0
	call _start

.section ".bootblock.stack", "aw", %progbits
_stack:
.space 65536
