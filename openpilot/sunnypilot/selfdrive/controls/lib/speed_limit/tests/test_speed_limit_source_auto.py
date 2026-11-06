"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time
from unittest.mock import MagicMock

from opendbc.car.structs import car

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.source_auto import (AUTO, CAR_HOLD_FRAMES, LOOKAHEAD_HOLD_DIST, AutoSource,
                                                                                  CarLimitHold, SpeedLimitSource, car_shows_limit, is_auto)
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver

MPH35, MPH45 = 35 * CV.MPH_TO_MS, 45 * CV.MPH_TO_MS
V = 20.


class TestIsAuto:
  def test_car_and_map_only_stand(self):
    assert not is_auto(Policy.car_state_only, True)
    assert not is_auto(Policy.map_data_only, True)

  def test_first_and_combined_are_auto(self):
    for p in (Policy.car_state_priority, Policy.map_data_priority, Policy.combined):
      assert is_auto(p, True)

  def test_no_car_source_is_always_auto(self):
    # Auto without a car limit is the map, look-ahead included; car-only would never find a limit
    for p in Policy:
      assert is_auto(p, False)


class TestCarShowsLimit:
  """has_speed_limit itself is tested in opendbc (test_has_speed_limit.py)."""
  def test_delegates(self):
    assert car_shows_limit(car.CarParams.new_message(brand="mazda"), None)
    assert not car_shows_limit(car.CarParams.new_message(brand="honda"), None)

  def test_unknown_car_trusts_the_setting(self):
    assert car_shows_limit(None, None)


class TestAutoSource:
  def test_car_wins_when_both_have_a_limit(self):
    assert AutoSource().pick(MPH45, MPH35, 0., V) == SpeedLimitSource.car

  def test_map_fills_in_without_a_car_limit(self):
    assert AutoSource().pick(0., MPH35, 0., V) == SpeedLimitSource.map

  def test_none(self):
    assert AutoSource().pick(0., 0., 0., V) == SpeedLimitSource.none

  def test_lookahead_lower_limit_takes_over(self):
    assert AutoSource().pick(MPH45, MPH35, 150., V) == SpeedLimitSource.map

  def test_lookahead_higher_limit_does_not(self):
    # the resolver only swaps in a lower limit ahead, but a higher map limit never beats the car
    assert AutoSource().pick(MPH35, MPH45, 150., V) == SpeedLimitSource.car

  def test_lookahead_holds_until_the_car_reads_the_sign(self):
    a = AutoSource()
    assert a.pick(MPH45, MPH35, 50., V) == SpeedLimitSource.map
    # past the change point, the car still shows 45 for a few seconds
    for _ in range(20):
      assert a.pick(MPH45, MPH35, 0., V) == SpeedLimitSource.map
    # the car reads the new sign
    assert a.pick(MPH35, MPH35, 0., V) == SpeedLimitSource.car

  def test_stale_map_lookahead_lets_go(self):
    a = AutoSource()
    a.pick(MPH45, MPH35, 50., V)
    frames = int(LOOKAHEAD_HOLD_DIST / (V * DT_MDL)) + 2
    sources = [a.pick(MPH45, MPH35, 0., V) for _ in range(frames)]
    assert sources[0] == SpeedLimitSource.map
    assert sources[-1] == SpeedLimitSource.car
    # and it does not latch again on the same stale limit
    assert a.pick(MPH45, MPH35, 0., V) == SpeedLimitSource.car

  def test_car_limit_held_through_a_blank(self):
    h = CarLimitHold()
    assert h.update(MPH45) == MPH45
    held = [h.update(0.) for _ in range(CAR_HOLD_FRAMES)]
    assert all(v == MPH45 for v in held)
    assert h.update(0.) == 0.
    assert h.update(MPH35) == MPH35


class TestResolverLookahead:
  """End to end through the resolver: a 35 zone 50 m ahead while doing 45."""

  @staticmethod
  def _resolve(policy, car_limit):
    feeds = {
      'carStateSP': MagicMock(speedLimit=car_limit),
      'liveMapDataSP': MagicMock(speedLimit=MPH45, speedLimitValid=True, speedLimitAhead=MPH35,
                                 speedLimitAheadValid=True, speedLimitAheadDistance=50.),
      'gpsLocation': MagicMock(unixTimestampMillis=time.monotonic() * 1e3),
    }
    sm = MagicMock()
    sm.__getitem__.side_effect = feeds.__getitem__
    resolver = SpeedLimitResolver()
    resolver._gps_location_service = 'gpsLocation'
    resolver.policy = policy
    resolver.update(MPH45, sm)
    return resolver

  def test_lookahead_reaches_the_output(self):
    for policy, car_limit in ((Policy.map_data_only, 0.), (AUTO, 0.), (AUTO, MPH45)):
      r = self._resolve(policy, car_limit)
      assert r.speed_limit == MPH35 and r.distance > 0., (policy, car_limit)

  def test_car_only_waits_for_the_sign(self):
    r = self._resolve(Policy.car_state_only, MPH45)
    assert r.speed_limit == MPH45 and r.distance == 0.
