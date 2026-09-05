"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed-bin torque values out of an rlog, across the 2026-09 schema move.

torqued_ext publishes the bins on its own message, liveTorqueParametersSP (customReserved19
on the wire), sent just before every lateralTorqueParameters at the same cadence. Logs from
before that carried them as fields @14 to @18 of lateralTorqueParameters itself, which the
current log.capnp no longer declares. Cap'n Proto keeps unknown fields on the wire, so a
legacy message is copied out and re-read through the old layout (legacy_speed_bin.capnp).

Usage in a LogReader loop:

  bins = SpeedBinTracker()
  for ev in LogReader(path):
    w = ev.which()
    if bins.feed(w, ev):
      continue                      # the fork message: remembered for the next upstream one
    if w == 'lateralTorqueParameters':
      sp = bins.bins_for(ev.lateralTorqueParameters)   # a reader with speedBin* fields, or None
"""
from pathlib import Path

import capnp

from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import LIVE_TORQUE_PARAMETERS_SP_SERVICE

SPEED_BIN_SERVICE = LIVE_TORQUE_PARAMETERS_SP_SERVICE
_LEGACY_SCHEMA = Path(__file__).with_name("legacy_speed_bin.capnp")
_legacy = None


def _legacy_struct():
  global _legacy
  if _legacy is None:
    _legacy = capnp.load(str(_LEGACY_SCHEMA)).LegacyLateralTorqueParameters
  return _legacy


def legacy_speed_bins(ltp):
  """The pre-move speed-bin fields of a lateralTorqueParameters reader, re-read through the
  old layout, or None when the message carries no bins (a new log, or a car without them)."""
  with _legacy_struct().from_bytes(ltp.as_builder().to_bytes()) as legacy:
    return legacy if len(legacy.speedBinCenters) else None


class SpeedBinTracker:
  """Pairs each upstream lateralTorqueParameters with the bins that go with it."""

  def __init__(self):
    self.sp_last = None
    self.seen_sp = False

  def feed(self, which, ev) -> bool:
    """Remembers a fork message; True when ev was one."""
    if which != SPEED_BIN_SERVICE:
      return False
    self.sp_last = getattr(ev, which)
    self.seen_sp = True
    return True

  def bins_for(self, ltp):
    """The fork message published beside this upstream one (new logs), else the fields
    re-read off the legacy layout (old logs), else None."""
    if self.seen_sp:
      return self.sp_last
    return legacy_speed_bins(ltp)
