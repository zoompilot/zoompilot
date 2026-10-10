"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.sunnypilot.selfdrive.selfdrived.tests.selfdrived_helpers import make_selfdrived

EventNameSP = custom.OnroadEventSP.EventName


class TestBigModelReady(OpenpilotTestCase):
  """The chime's edge cases; a load that works and one that fails are traced
  in test_selfdrived_traces.py."""

  def setUp(self):
    super().setUp()
    self.sd = make_selfdrived()

  def step(self, loading, active=None, big=False, alive=True):
    sd = self.sd
    sd.params.get_bool.return_value = loading
    sd.params.get.return_value = active
    sd.sm.alive['modelV2'] = alive
    sd.sm['modelV2'].big = big
    sd.update_events(SimpleNamespace(standstill=False, canValid=False))
    return EventNameSP.bigModelReady in sd.events_sp.names, EventName.bigModelFailed in sd.events.names

  def test_big_frame_from_a_dead_socket_does_not_chime(self):
    self.assertEqual(self.step(loading=False, big=True, alive=False), (False, False))
    self.assertEqual(self.step(loading=False, big=True, alive=False), (False, False))

  def test_rearms_after_a_fall_back_to_the_small_model(self):
    self.assertEqual(self.step(loading=False, active=True, big=True), (True, False))
    self.assertEqual(self.step(loading=False, active=True, big=False), (False, False))
    self.assertEqual(self.step(loading=False, active=True, big=True), (True, False))
    self.assertEqual(self.step(loading=False, active=True, big=True), (False, False))

