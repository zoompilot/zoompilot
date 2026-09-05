"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import ParamKeyFlag
from openpilot.sunnypilot.system.hardware.hardwared_ext import HardwaredExt


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)
    self.cleared: list[ParamKeyFlag] = []

  def get_bool(self, key):
    return self.bools.get(key, False)

  def put_bool(self, key, value, block=False):
    self.bools[key] = value

  def clear_all(self, flag):
    self.cleared.append(flag)


class TestOnroadCycle:
  def test_clears_onroad_transition_params(self):
    params = FakeParams()
    HardwaredExt(params).on_onroad_cycle()
    assert params.cleared == [ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION]
