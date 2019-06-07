# RISC-V Generic BSP

This demo is intended to be generic and simple enough to run on different RISC-V
multilibs and platforms. It has been tested on Spike, QEMU, Piccolo and Sail.
To configure the DEMO to run with a specific libs and/or platform, you need to
use the env variables when executing make.

# Build

Running make will build the default BSP configuration which is
spike-rv32imac-ilp32
```
$ make
```

