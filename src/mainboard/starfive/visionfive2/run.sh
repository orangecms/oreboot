#!/bin/sh

PORT=/dev/ttyUSB0
DTB=jh7110-starfive-visionfive-2-v1.3b.dtb
LINUX_DIR=~/firmware/RISC-V/StarFive/VisionFive2/linux
LINUX_DTB=$LINUX_DIR/arch/riscv/boot/dts/starfive/$DTB

make DRAM_SIZE=8G PORT=$PORT DTB=$LINUX_DTB run && tio $PORT
