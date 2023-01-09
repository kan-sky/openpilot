#!/usr/bin/bash

mount -o remount,rw /system
cp -rf /data/openpilot/installer/fonts/NanumGothic* /system/fonts/
cp -rf /data/openpilot/installer/fonts/opensans_* /data/openpilot/selfdrive/assets/fonts/
cp -rf /data/openpilot/installer/fonts/fonts.xml /system/etc/fonts.xml
chmod 644 /system/etc/fonts.xml
chmod 644 /system/fonts/NanumGothic*
