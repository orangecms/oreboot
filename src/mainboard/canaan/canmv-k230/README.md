# Canaan Kendryte K230(D)

There are two variants of this SoC:

- K230D, with 128MB built-in DRAM
- K230, with external DRAM

## Running oreboot

You will need <https://github.com/platform-system-interface/kendryte_boot>.

Test payload: <https://github.com/platform-system-interface/sbitest>.
In `sbitest/`, run `make` to obtain `sbitest_k230.bin`.

Either have `kendryte_boot` in your `$PATH`, or set the make variable
`KENDRYTE_BOOT`. Pass your payload via the make variable `PAYLOAD`:

```sh
make \
  KENDRYTE_BOOT=/path/to/kendryte_boot/target/release/kendryte_boot \
  PAYLOAD=/path/to/sbitest/sbitest_k230.bin \
  run
```

## Technical Reference Manual

<https://kendryte-download.canaan-creative.com/developer/k230/HDK/K230%E7%A1%AC%E4%BB%B6%E6%96%87%E6%A1%A3/K230_Technical_Reference_Manual_V0.3.1_20241118.pdf>
