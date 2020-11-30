#!/bin/sh

# This script uses dirrect paths so it wont work on another computer besides Ryans PC
# RUN with SUDO or else it wont setup the Imperx Camera Correctly

# Compile PTU Server Code
cd ..
pushd .
cd hardware/PTU/C++
rm -rf build
mkdir build
cd build
cmake ..
make -j4
echo "Running PTU Server"
./PanTiltController


