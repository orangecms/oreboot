ORE_DIR := ../../../../..
BASE_DIR := $(ORE_DIR)/target/armv7a-none-eabi/release
ELF_FILE := $(BASE_DIR)/oreboot-xiongmai-xm8536d-bt0
BIN_FILE := $(BASE_DIR)/oreboot-xiongmai-xm8536d-bt0.bin

OBJDUMP ?= arm-linux-gnueabi-objdump

all:
	cargo build --release #-vv
	cargo objcopy -- --strip-all -O binary $(BIN_FILE)
	truncate -s 128K $(BIN_FILE)
	ls -l $(BIN_FILE)

clean:
	cargo clean

objdump: all
	cargo objdump -- -D

objdumpgcc: all
	$(OBJDUMP) -D $(ELF_FILE)

hexdump: all
	xxd $(BIN_FILE)
