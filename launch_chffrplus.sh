#!/usr/bin/env bash

if [ ! -f "./boot_finish" ]; then
  mount -o rw,remount /system
  chmod 755 ./restart.sh
  chmod 755 ./selfdrive/apilot.py
  chmod 755 ./selfdrive/debug/clear_dtc.py
  chmod 755 ./selfdrive/debug/debug_console_carrot.py
  chmod 755 ./selfdrive/debug/debug_console.py

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
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
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

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1500 > /tmp/launch_log
  if python -c "import flask" > /dev/null 2>&1; then
    echo "Flask already installed."
  else
    echo "Flask installing."
    pip install flask
  fi
  if python -c "import shapely" > /dev/null 2>&1; then
    echo "shapely already installed."
  else
    echo "shapely installing."
    pip install shapely
  fi
  if python -c "import kaitaistruct" > /dev/null 2>&1; then
    echo "kaitaistruct already installed."
  else
    echo "kaitaistruct installing."
    pip install kaitaistruct
  fi

  # events language init
  #LANG=$(cat ${PARAMS_ROOT}/d/LanguageSetting)
  LANG=$(cat /data/params/d/LanguageSetting)
  EVENTSTAT=$(git status)


  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
