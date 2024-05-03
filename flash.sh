#!/bin/bash
set -x

cargo b --release
elf2uf2-rs -vds ./target/thumbv6m-none-eabi/release/planck
