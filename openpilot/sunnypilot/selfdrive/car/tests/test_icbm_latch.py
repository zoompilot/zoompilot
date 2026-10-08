"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import IcbmActivation, IcbmLatch, icbm_active


def _cs_sp(activation=None):
  msg = messaging.new_message('carStateSP')
  if activation is not None:
    msg.carStateSP.zoompilot.icbmActivation = activation
  return messaging.log_from_bytes(msg.to_bytes()).carStateSP


@pytest.mark.parametrize("pcm_cruise_speed", [True, False])
class TestIcbmLatch:
  def test_latch_follows_the_boot_flag(self, pcm_cruise_speed):
    latch = IcbmLatch(custom.CarParamsSP.new_message(pcmCruiseSpeed=pcm_cruise_speed))
    assert latch.active == (not pcm_cruise_speed)
    assert latch.activation == (IcbmActivation.passive if pcm_cruise_speed else IcbmActivation.active)

  def test_unset_falls_back_to_the_boot_flag(self, pcm_cruise_speed):
    # a log from before the field, or no carStateSP received yet
    CP_SP = custom.CarParamsSP.new_message(pcmCruiseSpeed=pcm_cruise_speed)
    assert icbm_active(_cs_sp(), CP_SP) == (not pcm_cruise_speed)

  def test_published_value_wins_over_the_boot_flag(self, pcm_cruise_speed):
    CP_SP = custom.CarParamsSP.new_message(pcmCruiseSpeed=pcm_cruise_speed)
    assert icbm_active(_cs_sp(IcbmActivation.active), CP_SP)
    assert not icbm_active(_cs_sp(IcbmActivation.passive), CP_SP)

  def test_card_and_readers_agree(self, pcm_cruise_speed):
    CP = structs.CarParams(pcmCruise=True)
    CP_SP = structs.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed)
    helper = VCruiseHelper(CP, CP_SP)
    assert helper.pcm_cruise_speed == pcm_cruise_speed

    published = _cs_sp(helper.icbm_latch.activation)
    servo = IntelligentCruiseButtonManagement(CP, custom.CarParamsSP.new_message(pcmCruiseSpeed=True))
    servo.update_activation(published)
    assert servo.active == (not pcm_cruise_speed)
