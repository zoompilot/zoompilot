"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.selfdrived.model_startup import SETTLE_S, TIMEOUT_S, ModelStartup


class FakeSM:
  """The four things ModelStartup reads off selfdrived's SubMaster."""
  def __init__(self):
    self.frame = 0
    self.seen = {'modelV2': False, 'deviceMotion': False}
    self.checks_ok = False
    self.device_motion = SimpleNamespace(inputsOK=False)

  def all_checks(self) -> bool:
    return self.checks_ok

  def __getitem__(self, service):
    assert service == 'deviceMotion'
    return self.device_motion


def seconds(s: float) -> int:
  return int(round(s / DT_CTRL))


class TestModelStartup:
  def setup_method(self):
    self.sm = FakeSM()
    self.startup = ModelStartup()

  def run_to(self, frame: int) -> bool:
    starting = True
    while self.sm.frame < frame:
      self.sm.frame += 1
      starting = self.startup.update(self.sm)
    return starting

  def test_holds_until_the_first_frame(self):
    assert self.run_to(seconds(20))
    self.sm.seen['modelV2'] = True
    assert self.run_to(seconds(20) + 1)

  def test_ends_once_the_model_and_what_follows_it_are_up(self):
    self.run_to(seconds(12))
    self.sm.seen['modelV2'] = self.sm.seen['deviceMotion'] = True
    assert self.run_to(seconds(13))

    self.sm.checks_ok = True
    assert self.run_to(seconds(13) + 1), "locationd still has no inputs"
    self.sm.device_motion.inputsOK = True
    assert not self.run_to(seconds(13) + 2)

  def test_ends_after_settling_if_what_follows_never_comes_up(self):
    self.run_to(seconds(12))
    self.sm.seen['modelV2'] = True
    first = self.sm.frame + 1
    assert self.run_to(first + seconds(SETTLE_S))
    assert not self.run_to(first + seconds(SETTLE_S) + 1)

  def test_ends_at_the_timeout_without_a_frame(self):
    assert self.run_to(seconds(TIMEOUT_S))
    assert not self.run_to(seconds(TIMEOUT_S) + 1)

  def test_never_comes_back(self):
    # deviceMotion not yet seen does not hold it
    self.sm.seen['modelV2'] = True
    self.sm.checks_ok = True
    assert not self.run_to(1)
    self.sm.checks_ok = False
    assert not self.run_to(seconds(TIMEOUT_S) * 2)
