"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.structs import car
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.sunnylink.capabilities import CAR_CAPABILITY_FIELDS, generate_capabilities


class TestHasSpeedLimitCapability(OpenpilotTestCase):
  """sunnylink hides the Speed Limit Source row on cars that report no limit, like the device UIs."""

  @staticmethod
  def _caps(brand, sp_flags=0):
    params = Params()
    params.put("CarParamsPersistent", car.CarParams.new_message(brand=brand).to_bytes(), block=True)
    params.put("CarParamsSPPersistent", custom.CarParamsSP.new_message(flags=sp_flags).to_bytes(), block=True)
    return generate_capabilities(params)

  def test_is_a_car_field(self):
    assert "has_speed_limit" in CAR_CAPABILITY_FIELDS

  def test_no_car_yet(self):
    assert generate_capabilities(Params())["has_speed_limit"] is False

  def test_follows_the_car(self):
    assert self._caps("mazda")["has_speed_limit"] is True
    assert self._caps("honda")["has_speed_limit"] is False
    assert self._caps("hyundai")["has_speed_limit"] is False
    assert self._caps("hyundai", int(HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE))["has_speed_limit"] is True
