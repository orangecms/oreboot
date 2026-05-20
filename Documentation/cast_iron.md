# Cast Iron

## Boot Chain

The boot flow is commonly:

1. mask ROM (hardwired)
2. bt0 (init DRAM)
3. main (in DRAM)
4. payload (in DRAM)

In theory, bt0 can already initiate loading code to other processing units.

## Build System

1. build/have payload, know its size, put at end of image, aligned
2. build main, passing supervisor flag and start/size of payload, put before
   payload, aligned
3. build bt0, passing start/size of main, put at start of image / as required by
   platform
4. build DTFS, put between bt0 and main, aligned

Q: where do we put other components? E.g., peripheral and coprocessor firmware
