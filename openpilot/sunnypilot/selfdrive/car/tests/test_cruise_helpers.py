"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from unittest.mock import Mock

from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import EventName, Events
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonEvent = structs.CarState.ButtonEvent
ButtonType = structs.CarState.ButtonEvent.Type


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


class FakeICBM:
  def __init__(self):
    self.activations = []

  def update_activation(self, CS_SP):
    self.activations.append(CS_SP.zoompilot.distanceFarther)


class TestAsSelfdrivedsBase:
  """Inside selfdrived the helper also steps the personality on the farther button and keeps ICBM's
  activation current, before selfdrived's own personality step."""

  def make(self, op_long: bool = True) -> CruiseHelper:
    helper = CruiseHelper(structs.CarParams(openpilotLongitudinalControl=op_long))
    helper.sm = {'carStateSP': cs_sp(False)}
    helper.params = Mock()
    helper.personality = 1
    helper.events = Events()
    helper.icbm = FakeICBM()
    return helper

  def step(self, helper: CruiseHelper, farther: bool, gap_released: bool = False) -> None:
    helper.sm['carStateSP'] = cs_sp(farther)
    CS = structs.CarState(cruiseState=structs.CarState.CruiseState(available=True))
    CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=False)] if gap_released else []
    helper.events.clear()
    helper.update(CS, EventsSP(), False)

  def test_a_farther_release_steps_the_personality_up(self):
    helper = self.make()
    self.step(helper, True)
    self.step(helper, False)
    assert helper.personality == 2
    assert helper.events.has(EventName.personalityChanged)
    helper.params.put.assert_called_once_with('LongitudinalPersonality', 2)

  def test_upstreams_gap_release_in_the_same_frame_steps_alone(self):
    helper = self.make()
    self.step(helper, True)
    self.step(helper, False, gap_released=True)
    assert helper.personality == 1 and not helper.events.has(EventName.personalityChanged)

  def test_no_step_without_openpilot_longitudinal(self):
    helper = self.make(op_long=False)
    self.step(helper, True)
    self.step(helper, False)
    assert helper.personality == 1

  def test_icbm_follows_card_every_frame(self):
    helper = self.make(op_long=False)
    for farther in (True, False, True):
      self.step(helper, farther)
    assert helper.icbm.activations == [True, False, True]
