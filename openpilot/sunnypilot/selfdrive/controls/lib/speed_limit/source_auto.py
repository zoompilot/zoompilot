"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The "Auto" speed limit source: the limit the car shows, the map when the car shows none, and
the map's look-ahead for a lower limit coming up. It replaces car-first, map-first and combined,
which all come down to "which one do I trust" and left the driver to pick.
"""
from opendbc.sunnypilot.car.interfaces import has_speed_limit
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

# stored as upstream's car_state_priority; map_data_priority and combined fold into it
AUTO = Policy.car_state_priority

CAR_HOLD_T = 2.0  # s a car limit is held through a blank frame run before the map takes over
CAR_HOLD_FRAMES = round(CAR_HOLD_T / DT_MDL)
LOOKAHEAD_HOLD_DIST = 300.  # m past the map's change point the car gets to read the new sign


def car_shows_limit(CP, CP_SP) -> bool:
  """has_speed_limit, with an unknown car (no CP yet, debug tools) trusting the stored setting."""
  return CP is None or has_speed_limit(CP, CP_SP)


def is_auto(policy: int, car_reports: bool) -> bool:
  """Car-only and map-only stand; everything else is Auto. Without a car source Auto is the map,
  and car-only would never find a limit."""
  return not car_reports or policy not in (Policy.car_state_only, Policy.map_data_only)


class CarLimitHold:
  """A brief blank (sign recognition dropping a frame run, a nav message gap) keeps the last car
  limit, so the target does not bounce to the map and back."""
  def __init__(self):
    self._car = 0.
    self._blank_frames = 0

  def update(self, car_limit: float) -> float:
    if car_limit > 0.:
      self._car = car_limit
      self._blank_frames = 0
    elif self._car > 0.:
      self._blank_frames += 1
      if self._blank_frames > CAR_HOLD_FRAMES:
        self._car = 0.
    return self._car


class AutoSource:
  def __init__(self):
    self._lookahead_car: float | None = None  # car limit when the map's look-ahead took over
    self._past_change = 0.

  def pick(self, car: float, map_limit: float, map_distance: float, v_ego: float):
    # map_distance > 0 means the map solution is a lower limit ahead the car has not reached yet
    if map_limit > 0. and map_distance > 0. and (car <= 0. or map_limit < car):
      self._lookahead_car = car
      self._past_change = 0.
      return SpeedLimitSource.map

    if self._lookahead_car is not None:
      # Past the change point the car is still showing the old sign until its camera reads the new
      # one. Keep the map's lower limit until the car's limit changes, so the target does not go
      # back up for a few seconds; a stale map limit lets go after LOOKAHEAD_HOLD_DIST.
      self._past_change += v_ego * DT_MDL
      if car > 0. and car == self._lookahead_car and 0. < map_limit < car and self._past_change < LOOKAHEAD_HOLD_DIST:
        return SpeedLimitSource.map
      self._lookahead_car = None

    if car > 0.:
      return SpeedLimitSource.car
    if map_limit > 0.:
      return SpeedLimitSource.map
    return SpeedLimitSource.none
