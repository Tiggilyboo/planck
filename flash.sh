#!/bin/bash
set -x

cargo b --release
elf2uf2-rs -vds ./target/thumbv6m-none-eabi/release/planck

# uart info
sudo screen /dev/ttyACM0

# show key presses
#showkey -ak
