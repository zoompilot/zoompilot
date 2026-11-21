#!/bin/bash
#
# Copyright (c) 2026-, Zeph Leggett.
# This file is part of jetlink and is licensed under the MIT License.
#
# Run cameras and modeld on the comma, with private Params and messaging.
# --record also runs recording and driver monitoring; --resources samples /proc.
# Leave jetlinkd up: modeld borrows the endpoints from it, as it does on a
# drive. Never run this in a moving vehicle.
#
#   jetlink_repo/scripts/comma/jetlink_live_bench.sh [seconds] [--record] [--resources]
#
# OPENPILOT is the checkout to run, by default the one this repo is a submodule
# of. OUTPUT may specify a new directory for logs, frame CSV and summary JSON.
set -eu
here="$(cd "$(dirname "$0")" && pwd)"
cd "${OPENPILOT:-$here/../../..}"
export PYTHONPATH=$PWD
seconds="${1:-180}"
if [ "$#" -gt 0 ]; then shift; fi
exec /usr/local/venv/bin/python3 "$here/jetlink_bench.py" \
  --seconds "$seconds" --output "${OUTPUT:-/data/tmp/jetlink-bench-$(date +%Y%m%d-%H%M%S)}" "$@"
