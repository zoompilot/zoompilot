"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.manager.process import PythonProcess
from openpilot.system.manager.process_config import always_run, managed_processes
from openpilot.sunnypilot.system.manager.processes import AudioProcess, RestartingPythonProcess


class FakeClock:
  t = 0.0
  starts = 0

  def now(self) -> float:
    return self.t

  def die(self):
    self.proc.exitcode = 1


class FakeRestarting(FakeClock, RestartingPythonProcess):
  pass


class FakeAudio(FakeClock, AudioProcess):
  card = False

  def sound_card_present(self) -> bool:
    return self.card


def fake_start(self):
  self.starts += 1
  self.proc = SimpleNamespace(exitcode=None, pid=1, is_alive=lambda: True)


def fake_stop(self, *args, **kwargs):
  self.proc = None


class TestRestartingProcess(OpenpilotTestCase):
  def setup_method(self):
    monkeypatch = self._fixture("monkeypatch")
    monkeypatch.setattr(PythonProcess, "start", fake_start)
    monkeypatch.setattr(PythonProcess, "stop", fake_stop)

  def test_backoff_doubles_then_resets_after_a_long_run(self):
    p = FakeRestarting("fake", "openpilot.fake", always_run)
    p.start()
    for expected in (p.BACKOFF, 2 * p.BACKOFF):
      p.t += 1.
      p.die()
      p.start()
      assert p.backoff == expected and p.proc is None
      p.t = p.next_start - 0.1
      p.start()
      assert p.proc is None
      p.t = p.next_start
      p.start()
      assert p.proc is not None
    p.t += p.QUICK_DEATH + 1.
    p.die()
    p.start()
    assert p.backoff == 0. and p.proc is not None

  def test_audio_processes_wait_for_the_sound_card(self):
    for name in ("micd", "soundd"):
      assert isinstance(managed_processes[name], AudioProcess), name

  def test_no_card_blocks_engagement(self):
    p = FakeAudio("fake", "openpilot.fake", always_run)
    p.start()
    state = p.get_process_state_msg()
    assert p.starts == 0 and state.shouldBeRunning and not state.running
    p.card = True
    p.start()
    state = p.get_process_state_msg()
    assert p.starts == 1 and state.shouldBeRunning and state.running

  def test_backoff_blocks_engagement(self):
    p = FakeAudio("fake", "openpilot.fake", always_run)
    p.card = True
    p.start()
    p.t = 1.
    p.die()
    p.start()
    state = p.get_process_state_msg()
    assert p.proc is None and state.shouldBeRunning and not state.running

  def test_stop_is_not_reported_down(self):
    p = FakeAudio("fake", "openpilot.fake", always_run)
    p.start()
    p.stop(block=False)
    assert not p.get_process_state_msg().shouldBeRunning
