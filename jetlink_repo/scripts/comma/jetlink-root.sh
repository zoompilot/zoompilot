#!/usr/bin/env bash
#
# Copyright (c) 2026-, Zeph Leggett.
# This file is part of jetlink and is licensed under the MIT License.
#
# Forwards to jetlink/comma/jetlink-root.sh, for a running owner from an older
# jetlink that still names this path while its checkout updates. Remove after v0.9.x.
exec bash "$(dirname "${BASH_SOURCE[0]}")/../../jetlink/comma/jetlink-root.sh" "$@"
