"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.interfaces import has_speed_limit


class TestHasSpeedLimit(unittest.TestCase):
  def test_brands(self):
    for brand in ("mazda", "tesla", "toyota"):
      self.assertTrue(has_speed_limit(structs.CarParams(brand=brand), None))
    self.assertFalse(has_speed_limit(structs.CarParams(brand="honda"), None))

  def test_hyundai_needs_a_flag(self):
    CP = structs.CarParams(brand="hyundai")
    self.assertFalse(has_speed_limit(CP, None))
    self.assertFalse(has_speed_limit(CP, structs.CarParamsSP()))
    for flag in (HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE, HyundaiFlagsSP.HAS_LKAS12):
      self.assertTrue(has_speed_limit(CP, structs.CarParamsSP(flags=int(flag))))
