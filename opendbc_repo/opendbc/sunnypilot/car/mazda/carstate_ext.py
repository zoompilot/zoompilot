"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV


def decode_speed_sign(sign) -> float:
  """SPEED_SIGN_UNIT encodes display state and unit: 1 = mph, 2 = km/h, 0 = none.
  Plausibility: 90 covers the highest US posting (85 mph); the 7-bit all-ones 127 is a sentinel."""
  speed_sign = sign["SPEED_SIGN"]
  if sign["SPEED_SIGN_UNIT"] == 1 and 0 < speed_sign <= 90:
    return float(speed_sign) * CV.MPH_TO_MS
  if sign["SPEED_SIGN_UNIT"] == 2 and 0 < speed_sign < 127:
    return float(speed_sign) * CV.KPH_TO_MS
  return 0.0


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    # Two sources share one layout. CAM_TRAFFIC_SIGNS is the camera's sign recognition, fused
    # with the map, and is sent blank by cars without it. NAV_SPEED_LIMIT is the navigation
    # unit's map limit, sent only with the nav SD card. Where both exist the camera wins: it
    # reads posted signs the map lacks or has stale.
    ret_sp.speedLimit = decode_speed_sign(can_parsers[Bus.cam].vl["CAM_TRAFFIC_SIGNS"])
    if ret_sp.speedLimit == 0.0:
      ret_sp.speedLimit = decode_speed_sign(can_parsers[Bus.pt].vl["NAV_SPEED_LIMIT"])
