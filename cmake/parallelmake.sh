#!/bin/bash
cores=$(grep -c ^processor /proc/cpuinfo)
make -j$cores "$@" 
exit