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
  export AGNOS_VERSION="19.7"
fi

export STAGING_ROOT="/data/safe_staging"

# zoo private S3 archive. Effective mode (comma | privacy) is computed from
# the toggles and diffed against ZooAppliedMode, so a switch drops the old
# mode's dongle identity at the next boot instead of diffing raw params.
# Toggle: /data/params/d/ZooS3Enabled ("1"/"0")
# Link:   /data/params/d/ZooS3CommaLinkEnabled ("1"/"0"); "0" = privacy mode
ZOO_PARAMS_DIR="${ZOO_PARAMS_DIR:-/data/params/d}"
ZOO_S3_ENABLED="0"
[ -f "$ZOO_PARAMS_DIR/ZooS3Enabled" ] && [ "$(cat "$ZOO_PARAMS_DIR/ZooS3Enabled" 2>/dev/null)" = "1" ] && ZOO_S3_ENABLED="1"
ZOO_COMMA_LINK="1"
[ -f "$ZOO_PARAMS_DIR/ZooS3CommaLinkEnabled" ] && [ "$(cat "$ZOO_PARAMS_DIR/ZooS3CommaLinkEnabled" 2>/dev/null)" = "0" ] && ZOO_COMMA_LINK="0"

ZOO_NOW="comma"
[ "$ZOO_S3_ENABLED" = "1" ] && [ "$ZOO_COMMA_LINK" = "0" ] && ZOO_NOW="privacy"
ZOO_PREV="$(cat "$ZOO_PARAMS_DIR/ZooAppliedMode" 2>/dev/null || true)"
if [ "$ZOO_PREV" != "$ZOO_NOW" ]; then
  # the first run after install only records the state; an empty previous
  # mode must not drop a healthy comma identity
  if [ -n "$ZOO_PREV" ]; then
    rm -f "$ZOO_PARAMS_DIR/DongleId"
    # a stale nag from a previous comma life must not follow into privacy mode
    rm -f "$ZOO_PARAMS_DIR/Offroad_UnregisteredHardware"
  fi
  printf '%s' "$ZOO_NOW" > "$ZOO_PARAMS_DIR/ZooAppliedMode"
fi
if [ "$ZOO_NOW" = "privacy" ]; then
  export ZOO_PRIVACY_MODE=1
fi
