#!/bin/sh

# IMPERX SETUP
# Increase Maximum Filebuffer Memory size for Imperx
cd ..
pushd .
cd hardware/imperx/C++/IpxCameraSDK-1.2.0.13/bin/Linux64_x64
./manage_usbfs_memory_size.sh

# compile and run imperx server
cd ../../../../../../
cd hardware/imperx/C++/IpxCameraSDK-1.2.0.13/samples
rm -rf build
mkdir build
cd build
cmake ..
make -j4
cd QtCommonStreamUI
./QtCommonStreamUI
