"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper


def cs_sp(farther: bool):
  msg = messaging.new_message('carStateSP').carStateSP
  msg.zoompilot.distanceFarther = farther
  return msg


class TestDistanceFarther:
  """The wheel's second distance button reaches selfdrived as a level on carStateSP; the helper
  turns it into one release edge, the opposite direction to upstream's gapAdjustCruise cycle."""

  def test_one_step_per_release(self):
    helper = CruiseHelper(structs.CarParams())
    assert not helper.distance_farther_released(cs_sp(False))
    assert not helper.distance_farther_released(cs_sp(True))
    assert not helper.distance_farther_released(cs_sp(True))   # held: nothing yet
    assert helper.distance_farther_released(cs_sp(False))      # the release steps once
    assert not helper.distance_farther_released(cs_sp(False))

  def test_cars_without_the_button_never_step(self):
    helper = CruiseHelper(structs.CarParams())
    assert not any(helper.distance_farther_released(cs_sp(False)) for _ in range(10))
