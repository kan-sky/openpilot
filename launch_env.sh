#!/usr/bin/env bash
export PYTHONPATH="/data/pylibs:$PYTHONPATH"

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
  export AGNOS_VERSION="20.3"
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

# 기기에 /data/skip_fw_query 파일이 있으면 시동 시 VIN/펌웨어 조회(OBD multiplexing)를 건너뛴다.
# OBD 포트 쪽 배선이 없는 DIY 하네스에서 조회 중 버스 1 stuff error 폭주로 interruptRateCan2 폴트가 걸리는 것 방지.
# 차량 인식은 CAN 핑거프린트로 한다.
if [ -f /data/skip_fw_query ]; then
  export SKIP_FW_QUERY=1
fi
