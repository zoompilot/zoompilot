#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.mazda.values import DBC

# Six tracks arrive on 0x361-0x366 at 10 Hz. Exclude 0x365 and 0x366 until their relative
# velocity encoding is known; radard requires a valid vRel.
RADAR_TRACK_ADDRS = list(range(0x361, 0x367))
RADAR_USABLE_ADDRS = set(range(0x361, 0x365))  # only tracks with reliable RELV
RADAR_TRIGGER_MSG = RADAR_TRACK_ADDRS[-1]  # 0x366, last in the burst
SENTINEL_DIST = 4095 * 0.0625   # 255.9375 m, raw 4095
SENTINEL_ANG = 2046 * 0.015625  # 31.96875 deg, raw 2046
SENTINEL_RELV = -16 * 0.0625    # -1.0 m/s, raw -16


# Track IDs preserve the existing DBC message names for 0x361-0x366.
def _create_radar_can_parser(car_fingerprint):
  messages = [(addr, 10) for addr in RADAR_TRACK_ADDRS]
  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 0)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.track_id = 0
    self.updated_messages = set()
    self.trigger_msg = RADAR_TRIGGER_MSG
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for addr in RADAR_TRACK_ADDRS:
      msg = self.rcp.vl[addr]

      dist = msg['DIST_OBJ']
      ang = msg['ANG_OBJ']
      relv = msg['RELV_OBJ']

      # Each sentinel is individually valid, so require the complete empty-slot signature.
      slot_empty = dist == SENTINEL_DIST and ang == SENTINEL_ANG and relv == SENTINEL_RELV
      if slot_empty or addr not in RADAR_USABLE_ADDRS:
        if addr in self.pts:
          del self.pts[addr]
        continue

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      azimuth = math.radians(ang)
      self.pts[addr].dRel = math.cos(azimuth) * dist
      self.pts[addr].yRel = -math.sin(azimuth) * dist
      self.pts[addr].vRel = relv

    ret.points = list(self.pts.values())
    return ret
