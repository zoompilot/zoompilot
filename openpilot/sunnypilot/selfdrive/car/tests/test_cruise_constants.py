"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The cruise sentinels have one upstream home, selfdrive.car.cruise. The card-side stack
cannot import it (cruise.py imports cruise_ext before defining them), so cruise_ext and
speed_limit keep copies; these tests pin them to upstream.
"""
from openpilot.selfdrive.car import cruise
from openpilot.sunnypilot.selfdrive.car import cruise_ext
from openpilot.sunnypilot.selfdrive.controls.lib import speed_limit


class TestCruiseSentinels:
  def test_unset_matches_upstream(self):
    assert speed_limit.V_CRUISE_UNSET == cruise.V_CRUISE_UNSET
    assert cruise_ext.V_CRUISE_UNSET == cruise.V_CRUISE_UNSET

  def test_max_matches_upstream(self):
    assert cruise_ext.V_CRUISE_MAX == cruise.V_CRUISE_MAX

  def test_min_matches_upstream(self):
    assert cruise_ext.V_CRUISE_MIN == cruise.V_CRUISE_MIN
