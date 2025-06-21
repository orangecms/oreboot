OREBOOT=$(abspath $(CURDIR)/../../../../../)
TARGET     = riscv64imac-unknown-none-elf
SERIAL    := /dev/ttyUSB0
RELEASE_ELF= $(OREBOOT)/target/$(TARGET)/release/spacemit-k1x-bt0
RELEASE_BIN= $(OREBOOT)/target/$(TARGET)/release/spacemit-k1x-bt0.bin
FINAL_BIN= $(OREBOOT)/target/$(TARGET)/release/spacemit-k1x-bt0_h.bin

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
	cd $(OREBOOT) && ./tools/spacemit/build.py -c tools/spacemit/oreboot.json -o $(FINAL_BIN)

run: build
	fastboot stage $(FINAL_BIN)
	fastboot continue

objdump: build
	cargo objdump -- -D

hexdump: build
	xxd $(RELEASE_BIN)

flowchart:
	mmcd -i flow.chart -w 1920 -H 1080 -o flow.png
