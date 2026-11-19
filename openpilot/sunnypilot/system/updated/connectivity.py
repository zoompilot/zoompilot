"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
# Overrides for updated.py's connectivity limits, imported right after upstream defines its
# own. Unreachable for any device: both connectivity alerts (the offroad prompt and the
# no-engage cutoff) stay permanently off.
HOURS_NO_CONNECTIVITY_MAX = 1000000
ROUTES_NO_CONNECTIVITY_MAX = 1000000
HOURS_NO_CONNECTIVITY_PROMPT = 1000000
ROUTES_NO_CONNECTIVITY_PROMPT = 1000000
