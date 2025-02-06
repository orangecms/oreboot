OREBOOT=$(abspath $(CURDIR)/../../../../../)
TARGET     = riscv64imac-unknown-none-elf
RELEASE_ELF= $(OREBOOT)/target/$(TARGET)/release/milk_v-duo_s-bt0
RELEASE_BIN= $(OREBOOT)/target/$(TARGET)/release/milk_v-duo_s-bt0.bin

SG_BOOT ?= sg_boot
SERIAL  ?= /dev/ttyUSB0
SOC     ?= SG200X

cibuild: mainboard
# TODO
nop:
	echo nope...

ciclippy: nop
citest: nop
checkformat: nop

mainboard: build

build:
	# TODO: cargo make
	cargo build --release \
		--config \
		target.riscv64imac-unknown-none-elf.rustflags=[\'--cfg=soc=\"$(SOC)\"\'] \
		--config \
		rustc-cfg=\'soc=\"$(SOC)\"\' \
		--config \
		build.rustflags=[\'--cfg=soc=\"$(SOC)\"\']
	cargo objcopy -- -O binary $(RELEASE_BIN)

run: build
	$(SG_BOOT) run $(RELEASE_BIN)

objdump: build
	cargo objdump -- -D

hexdump: build
	xxd $(RELEASE_BIN)
