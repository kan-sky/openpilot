#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.6"
fi

export STAGING_ROOT="/data/safe_staging"

# tinygrad's diskcache (sqlite, used to compile/cache selfdrive model kernels)
# defaults to $XDG_CACHE_HOME/tinygrad, falling back to ~/.cache/tinygrad when
# unset (see tinygrad_repo/tinygrad/helpers.py). Two independent fresh-install
# reports (2026-09-19) hit "sqlite3.OperationalError: database or disk is
# full" from diskcache_put during the very first model compile, even though
# /data itself had tens of GB free - pointing at ~/.cache landing on a much
# smaller/more constrained mount than /data. Pin it explicitly to /data, the
# one partition confirmed spacious in both reports, instead of trusting
# whatever $HOME resolves to.
export XDG_CACHE_HOME="/data/.cache"
