#!/usr/bin/env bash

if [ ! -f "./boot_finish" ]; then
  mount -o rw,remount /system
  chmod 755 ./restart.sh
  chmod 755 ./selfdrive/apilot.py

  if [ ! -f "/data/params/d/DongleId" ]; then
    echo -n "UnregisteredDevice" > /data/params/d/DongleId
  fi
  rm -f /data/params/d/Offroad_UnregisteredHardware
  touch ./boot_finish
else
  chmod 644 ./boot_finish
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function ensure_python_package {
  # Never blocks boot. A 2026-09-17 real-world case proved why: at this point
  # in boot, network/DNS may not be up yet even on a device that was fully set
  # up over WiFi - a fresh reinstall hit "Temporary failure in name
  # resolution" on every package here and, because failures used to hang boot
  # forever (`while true; do sleep 1; done`), the car couldn't start at all.
  # None of this function's packages gate core driving (steering/longitudinal)
  # - they're all peripheral carrot features (settings UI, navi, cluster
  # display, not-yet-ported extras) - so a transient install failure should
  # degrade that one feature, never the ability to drive. Installs straight
  # into the running Python's own environment (confirmed writable on real
  # devices - carrot features have been `pip install`ed directly before).
  local import_name="$1"
  local package_name="$2"
  local used_now="${3:-0}"

  if python3 -c "import ${import_name}" > /dev/null 2>&1; then
    echo "${package_name} already installed."
    return 0
  fi

  echo "${package_name} not found, installing."
  if python3 -m pip install --disable-pip-version-check --no-input --timeout 15 --retries 2 \
       --upgrade "$package_name" && \
     python3 -c "import ${import_name}" > /dev/null 2>&1; then
    echo "${package_name} installed."
    return 0
  fi

  # Keep the actual import error visible when installation could not repair it.
  python3 -c "import ${import_name}" >&2
  if [ "$used_now" = "1" ]; then
    echo "WARNING: ${package_name} is unavailable - a feature that depends on it today will be degraded this boot."
  else
    echo "${package_name} is unavailable; continuing without it (not used by anything yet)."
  fi
  return 0
}

function bootstrap_runtime_dependencies {
  # Full carrot-wip package set, bundled up front so a future carrot feature
  # port doesn't need its own dependency-install pass. Only the 5 packages
  # something in 196 actually imports today (used_now=1, just louder logging
  # on failure - see ensure_python_package) predate this port; the other 8 are
  # pre-staged for carrot features not yet ported here and nothing imports
  # them yet, so install failures for those are true no-ops either way.
  ensure_python_package serial pyserial 1
  ensure_python_package aiohttp aiohttp 1
  ensure_python_package psutil psutil 1
  ensure_python_package qrcode qrcode 1
  ensure_python_package shapely shapely 1

  ensure_python_package msgpack msgpack 0
  ensure_python_package av av 0
  ensure_python_package aiortc "aiortc==1.14.0" 0
  ensure_python_package crcmod crcmod-plus 0
  ensure_python_package jsonrpc json-rpc 0
  ensure_python_package brotli brotli 0
  ensure_python_package usb pyusb 0

  # carrot-wip pins this for its Xiaoge lane/BSD inference feature (not yet
  # ported to 196).
  ensure_python_package cv2 "opencv-python-headless==4.13.0.92" 0
}

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/openpilot/common/hardware/comma/agnos.py"
    MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/openpilot/common/hardware/comma/updater $AGNOS_PY $MANIFEST
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD"

  # submodule package symlinks for PYTHONPATH imports on device.
  # on PC these come from editable installs via pyproject.toml / uv.
  ln -sfn msgq_repo/msgq msgq
  ln -sfn opendbc_repo/opendbc opendbc
  ln -sfn rednose_repo/rednose rednose
  ln -sfn teleoprtc_repo/teleoprtc teleoprtc
  ln -sfn tinygrad_repo/tinygrad tinygrad

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # SCons imports some of these dependency modules while building Params, so
  # bootstrap them before the first SCons invocation. Runs unconditionally
  # (including on prebuilt images) - each package's own already-installed
  # check makes that a fast no-op when nothing's missing. Never blocks boot -
  # see ensure_python_package.
  bootstrap_runtime_dependencies

  # start manager
  cd openpilot/system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
