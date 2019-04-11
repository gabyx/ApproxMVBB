#!/bin/bash
cores=$(grep -c ^processor /proc/cpuinfo)
echo "Building with ${cores} cores!"
make -j${cores} "$@" 
exit