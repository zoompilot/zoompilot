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

# optional per-device overrides: KEY=VALUE lines, fixed allowlist below.
# the file must be owned by the launcher user (comma on devices), with mode
# 600, 640, or 644, or it is ignored with a note in the launch log.
# it lives outside /data/openpilot, so updates do not remove it.
env_file=/data/openpilot.env
if [ -f "$env_file" ]; then
  if [ -O "$env_file" ]; then
    case "$(stat -c %a "$env_file" 2>/dev/null)" in
      600|640|644)
        while IFS='=' read -r key value; do
          value="${value%$'\r'}"
          case "$key" in
            API_HOST|ATHENA_HOST) [ -n "$value" ] && export "$key=$value" ;;
          esac
        done < "$env_file"
        ;;
      *)
        echo "ignoring $env_file: want mode 600, 640, or 644"
        ;;
    esac
  else
    echo "ignoring $env_file: not owned by $(id -un)"
  fi
fi
unset env_file key value
