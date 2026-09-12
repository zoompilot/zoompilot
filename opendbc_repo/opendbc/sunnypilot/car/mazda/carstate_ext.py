"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp_cam = can_parsers[Bus.cam]

    # CAM_TRAFFIC_SIGNS.SPEED_SIGN_UNIT encodes display state and unit: 1 = mph, 2 = km/h,
    # 0 = none. The limit is camera-detected or the car's own map fallback; both are used.
    # Plausibility: 90 covers the highest US posting (85 mph); the 7-bit all-ones 127 is a
    # sentinel.
    sign = cp_cam.vl["CAM_TRAFFIC_SIGNS"]
    speed_sign = sign["SPEED_SIGN"]
    if sign["SPEED_SIGN_UNIT"] == 1 and 0 < speed_sign <= 90:
      ret_sp.speedLimit = float(speed_sign) * CV.MPH_TO_MS
    elif sign["SPEED_SIGN_UNIT"] == 2 and 0 < speed_sign < 127:
      ret_sp.speedLimit = float(speed_sign) * CV.KPH_TO_MS
    else:
      ret_sp.speedLimit = 0.0
