"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's managed processes, applied to process_config's list by one call.
"""
import glob
import os
import time

from opendbc.car.structs import car
from openpilot.common.hardware import PC
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.manager.process import ManagerProcess, NativeProcess, PythonProcess

from openpilot.sunnypilot import jetlink_adapter

AUDIO_PROCESSES = ("micd", "soundd")


class RestartingPythonProcess(PythonProcess):
  """A PythonProcess that manager starts again after it dies: start() leaves
  a proc that has exited in place for good. For jetlinkd, which holds the USB
  gadget for as long as the link is on; jetlink's owner adopts what a dead
  one left and holds a crash loop back itself.

  One that dies within QUICK_DEATH of its start never got that far (an
  import error, a raise before the owner's loop, a second owner stepping
  aside for a live one), so the next start waits BACKOFF, doubling to
  BACKOFF_MAX, rather than forking manager twice a second for a whole drive.
  One that ran longer is started again on the next loop."""
  QUICK_DEATH = 10.0
  BACKOFF = 10.0
  BACKOFF_MAX = 300.0

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.started_at = 0.0
    self.backoff = 0.0
    self.next_start = 0.0

  def now(self) -> float:
    return time.monotonic()

  def start(self) -> None:
    now = self.now()
    if self.proc is not None and self.proc.exitcode is not None:
      if now - self.started_at < self.QUICK_DEATH:
        self.backoff = min(self.BACKOFF_MAX, 2 * self.backoff or self.BACKOFF)
        self.next_start = now + self.backoff
        cloudlog.warning(f"{self.name} died {now - self.started_at:.1f} s after it started, starting it again in {self.backoff:.0f} s")
      else:
        self.backoff = 0.0
      self.stop()  # reaps it, logs the exit code and clears proc
    if self.proc is None:
      if now < self.next_start:
        return
      self.started_at = now
    super().start()


class AudioProcess(RestartingPythonProcess):
  """micd and soundd: started once the sound card exists, which a slow boot can
  register 30 s after manager wants them, longer than their stream retry waits.
  Until then, and through a restart's backoff, they report shouldBeRunning with
  nothing running, so processNotRunning keeps openpilot from engaging silently."""
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.wanted = False

  @staticmethod
  def sound_card_present() -> bool:
    return PC or bool(glob.glob("/dev/snd/pcmC*"))

  def start(self) -> None:
    if self.proc is None and not self.sound_card_present():
      self.wanted = True
      return
    super().start()  # its reap of a dead proc goes through stop()
    self.wanted = True

  def stop(self, *args, **kwargs):
    self.wanted = False
    return super().stop(*args, **kwargs)

  def get_process_state_msg(self):
    state = super().get_process_state_msg()
    if self.wanted and self.proc is None:
      state.name = self.name
      state.shouldBeRunning = True
    return state


def use_github_runner(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started and not PC and params.get_bool("EnableGithubRunner") and (
    not params.get_bool("NetworkMetered") and not params.get_bool("GithubRunnerSufficientVoltage"))


def extend_procs(procs: list[ManagerProcess]) -> list[ManagerProcess]:
  """process_config's list with micd and soundd as AudioProcess, jetlinkd beside
  the model manager, and the GitHub runner at the end."""
  procs = [AudioProcess(p.name, p.module, p.should_run, enabled=p.enabled, sigkill=p.sigkill) if p.name in AUDIO_PROCESSES else p
           for p in procs]

  # runs onroad too: jetlinkd holds the USB gadget open for as long as the link
  # is enabled. A gadget whose owner exits leaves the bus, and that is the
  # unplug at every ignition edge this arrangement removes
  jetlinkd = RestartingPythonProcess(jetlink_adapter.OWNER, jetlink_adapter.__name__, jetlink_adapter.should_run)
  at = next((i + 1 for i, p in enumerate(procs) if p.name == "models_manager"), len(procs))
  procs.insert(at, jetlinkd)

  if os.path.exists("../../../release/ci/github_runner.sh"):
    procs.append(NativeProcess("github_runner_start", "release/ci", ["./github_runner.sh", "start"], use_github_runner, sigkill=False))
  return procs
