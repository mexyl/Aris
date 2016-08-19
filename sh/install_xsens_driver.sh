#!/bin/bash

cd /usr/aris2/sh/xsens_driver && make
modprobe usbserial
insmod xsens_mt.ko
cd ..
echo "imu driver installed successfully"

