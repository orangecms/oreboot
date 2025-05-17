OREBOOT=$(abspath $(CURDIR)/../../../../../)
TARGET     = riscv64imac-unknown-none-elf
RELEASE_ELF= $(OREBOOT)/target/$(TARGET)/release/milk_v-duo_s-main
RELEASE_BIN= $(OREBOOT)/target/$(TARGET)/release/milk_v-duo_s-main.bin

SG_BOOT ?= sg_boot
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
	cargo build --release
	cargo objcopy -- -O binary $(RELEASE_BIN)

load: build
	$(SG_BOOT) load $(RELEASE_BIN)

objdump: build
	cargo objdump -- -D

hexdump: build
	xxd $(RELEASE_BIN)
