#!/bin/sh
#
# Makes the module then insmod's it, and sets permissions

make clean && make && sudo insmod gdeq046t82.ko && sudo chmod 666 /sys/bus/spi/devices/spi0.0/update_display

