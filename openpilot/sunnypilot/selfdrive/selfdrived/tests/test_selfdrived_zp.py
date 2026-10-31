"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.cereal import custom, log, messaging
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import EventName, Events
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.relc import RoadEdgeMode
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_zp import EVENTS_SP_ZP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
from openpilot.sunnypilot.selfdrive.selfdrived.selfdrived_zp import DISTANCE_LONG_PRESS, PARAMS_READ_FRAMES, SelfdrivedZP

ButtonType = structs.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert
AlertSize = log.SelfdriveState.AlertSize


def cs_sp(farther: bool = False):
  msg = messaging.new_message('carStateSP').carStateSP
  msg.zoompilot.distanceFarther = farther
  return msg


def car_state(gap: bool | None = None):
  CS = messaging.new_message('carState').carState
  CS.cruiseState.available = True
  CS.buttonEvents = [] if gap is None else [{'type': ButtonType.gapAdjustCruise, 'pressed': gap}]
  return CS


class FakeICBM:
  def __init__(self):
    self.activations = []

  def update_activation(self, CS_SP):
    self.activations.append(CS_SP.zoompilot.distanceFarther)


class Selfdrived:
  """The helper as selfdrived drives it: the update, then upstream's personality step on a
  gapAdjustCruise release, behind the helper's guard."""

  def __init__(self, op_long: bool = True, experimental_mode: bool = True):
    self.params = FakeParams(ExperimentalMode=experimental_mode)
    self.zp = SelfdrivedZP(structs.CarParams(openpilotLongitudinalControl=op_long), self.params)
    self.personality = 1
    self.icbm = FakeICBM()
    self.events, self.events_sp = Events(), EventsSP()

  def step(self, CS=None, farther: bool = False) -> None:
    CS = CS or car_state()
    self.events.clear()
    self.events_sp.clear()
    self.personality = self.zp.update(CS, cs_sp(farther), self.events, self.events_sp, self.icbm, self.personality)
    if self.zp.CP.openpilotLongitudinalControl:
      if any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents) and self.zp.gap_release_steps:
        self.personality = (self.personality - 1) % 3
        self.events.add(EventName.personalityChanged)

  def hold_gap(self, frames: int) -> None:
    self.step(car_state(True))
    for _ in range(frames - 1):
      self.step()
    self.step(car_state(False))


class TestExperimentalSwitch:
  def test_long_press_switches_mode_only(self):
    sd = Selfdrived()
    sd.hold_gap(DISTANCE_LONG_PRESS + 5)
    assert sd.zp.experimental_mode is False
    assert sd.params.values["ExperimentalMode"] is False
    assert sd.personality == 1 and not sd.events.has(EventName.personalityChanged)

  def test_long_press_raises_the_alert_once(self):
    sd = Selfdrived()
    raised = 0
    sd.step(car_state(True))
    for _ in range(DISTANCE_LONG_PRESS * 3):
      sd.step()
      raised += sd.events_sp.has(EventNameSP.experimentalModeSwitchedDEPRECATED)
    assert raised == 1

  def test_short_press_steps_personality_only(self):
    sd = Selfdrived()
    sd.hold_gap(DISTANCE_LONG_PRESS // 2)
    assert sd.zp.experimental_mode is True
    assert sd.personality == 0

  def test_press_after_a_long_press_steps_again(self):
    sd = Selfdrived()
    sd.hold_gap(DISTANCE_LONG_PRESS + 5)
    sd.hold_gap(5)
    assert sd.personality == 0

  def test_no_switch_without_openpilot_longitudinal(self):
    sd = Selfdrived(op_long=False)
    sd.hold_gap(DISTANCE_LONG_PRESS + 5)
    assert sd.params.values["ExperimentalMode"] is True
    assert sd.zp.experimental_mode is False

  def test_follows_the_setting(self):
    sd = Selfdrived()
    sd.params.values["ExperimentalMode"] = False
    for _ in range(PARAMS_READ_FRAMES):
      sd.step()
    assert sd.zp.experimental_mode is False

  def test_publishes_the_mode(self):
    sd = Selfdrived(experimental_mode=False)
    msg = messaging.new_message('selfdriveStateSP').selfdriveStateSP
    sd.zp.publish(msg)
    assert msg.zoompilot.experimentalMode is False
    sd.hold_gap(DISTANCE_LONG_PRESS + 5)
    sd.zp.publish(msg)
    assert msg.zoompilot.experimentalMode is True

  def test_alert_names_the_new_mode(self):
    alert = EVENTS_SP[EventNameSP.experimentalModeSwitchedDEPRECATED][ET.WARNING]
    sd = Selfdrived()
    for text in ("Legacy Chill On", "Legacy Chill Off"):
      sd.hold_gap(DISTANCE_LONG_PRESS + 5)
      assert alert(None, None, None, False, 0, None).alert_text_1 == text


class TestDistanceFarther:
  """The wheel's second distance button reaches selfdrived as a level on carStateSP; the helper
  turns it into one release edge, the opposite direction to upstream's gapAdjustCruise cycle."""

  def test_a_farther_release_steps_the_personality_up(self):
    sd = Selfdrived()
    sd.step(farther=True)
    sd.step(farther=False)
    assert sd.personality == 2
    assert sd.events.has(EventName.personalityChanged)
    assert sd.params.values['LongitudinalPersonality'] == 2

  def test_held_steps_once_on_release(self):
    sd = Selfdrived()
    for _ in range(5):
      sd.step(farther=True)
    assert sd.personality == 1
    sd.step(farther=False)
    sd.step(farther=False)
    assert sd.personality == 2

  def test_upstreams_gap_release_in_the_same_frame_steps_alone(self):
    sd = Selfdrived()
    sd.step(car_state(True), farther=True)
    sd.step(car_state(False), farther=False)
    assert sd.personality == 0

  def test_no_step_without_openpilot_longitudinal(self):
    sd = Selfdrived(op_long=False)
    sd.step(farther=True)
    sd.step(farther=False)
    assert sd.personality == 1

  def test_icbm_follows_card_every_frame(self):
    sd = Selfdrived(op_long=False)
    for farther in (True, False, True):
      sd.step(farther=farther)
    assert sd.icbm.activations == [True, False, True]


class TestRoadEdgeAlert:
  @pytest.mark.parametrize("mode, audible", [(RoadEdgeMode.AUDIBLE, AudibleAlert.prompt), (RoadEdgeMode.SILENT, AudibleAlert.none)])
  def test_silent_drops_only_the_chime(self, mode, audible):
    sd = Selfdrived()
    sd.params.put("RoadEdgeLaneChangeEnabled", mode)
    for _ in range(PARAMS_READ_FRAMES):
      sd.step()
    alert = EVENTS_SP_ZP[EventNameSP.laneChangeRoadEdge][ET.WARNING](None, None, None, False, 0, 0)
    assert alert.audible_alert == audible
    assert alert.alert_size == AlertSize.small, "the on-screen alert is the same in both modes"
