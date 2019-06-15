#!/bin/sh

# build and make sure everything ready for copying to SD card
cd build && make -j `nproc` && make optee-os optee-client optee-examples

echo "Build Script finished!"
