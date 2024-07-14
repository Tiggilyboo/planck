#!/bin/bash

rm planck.h
python pico-sdk/lib/btstack/tool/compile_gatt.py planck.gatt planck.h

rm -rf build
mkdir build
cd build
cmake ..
make
cd ..
