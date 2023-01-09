#!/usr/bin/bash

mount -o rw,remount /system
export PASSIVE="0"
chmod 700 ./launch_chffrplus.sh
sed -i -e 's/\r$//' ./launch_chffrplus.sh
chmod 700 ./unix.sh
sed -i -e 's/\r$//' ./unix.sh
./unix.sh
./launch_chffrplus.sh

