#!/usr/bin/env python3

import os
import unittest
import signal
import time
from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.params import Params
import openpilot.system.manager.manager as manager
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes, procs
from openpilot.common.hardware import HARDWARE

os.environ['FAKEUPLOAD'] = "1"

MAX_STARTUP_TIME = 3
BLACKLIST_PROCS = ['manage_athenad', 'pandad', 'pigeond']


class TestManager(OpenpilotTestCase):
  def setup_method(self):
    HARDWARE.set_power_save(False)

    # ensure clean CarParams
    params = Params()
    params.clear_all()

  def teardown_method(self):
    manager.manager_cleanup()

  def test_duplicate_procs(self):
    assert len(procs) == len(managed_processes), "Duplicate process names"

  def test_blacklisted_procs(self):
    # TODO: ensure there are blacklisted procs until we have a dedicated test
    assert len(BLACKLIST_PROCS), "No blacklisted procs to test not_run"

  def test_set_params_with_default_value(self, monkeypatch):
    params = Params()
    params.clear_all()
    monkeypatch.setattr(manager, "save_bootlog", lambda: None)

    os.environ['PREPAREONLY'] = '1'
    manager.main()
    for k in params.all_keys():
      default_value = params.get_default_value(k)
      if default_value is not None:
        assert params.get(k) == default_value
    assert params.get("OpenpilotEnabledToggle")
    assert params.get("RouteCount") == 0

  def test_connect_backend_reconciled_before_registration_and_sentry(self, monkeypatch):
    events = []

    class FakeParams:
      def clear_all(self, _flag):
        pass

      def get(self, _key):
        return None

      def get_bool(self, _key):
        return False

      def all_keys(self):
        return ()

      def put(self, _key, _value, block=False):
        assert block

      def put_bool(self, _key, _value, block=False):
        assert block

    build = SimpleNamespace(
      channel="test",
      development_channel=True,
      tested_channel=False,
      release_channel=False,
      release_sp_channel=False,
      openpilot=SimpleNamespace(
        version="test",
        git_commit="deadbeef",
        git_commit_date="now",
        git_origin="origin",
        git_normalized_origin="origin",
        is_dirty=True,
      ),
    )
    params = FakeParams()
    monkeypatch.setattr(manager, "Params", lambda: params)
    monkeypatch.setattr(manager, "save_bootlog", lambda: None)
    monkeypatch.setattr(manager, "get_build_metadata", lambda: build)
    monkeypatch.setattr(manager, "enforce_backend_state", lambda p: events.append(("enforce", p)))
    monkeypatch.setattr(manager, "register", lambda show_spinner: events.append(("register", show_spinner)) or "dongle")
    monkeypatch.setattr(manager.sentry, "init", lambda project: events.append(("sentry", project)))
    monkeypatch.setattr(manager.cloudlog, "bind_global", lambda **kwargs: None)
    monkeypatch.setattr(manager.os, "mkdir", lambda path: None)
    monkeypatch.setattr(manager.HARDWARE, "get_serial", lambda: "serial")
    monkeypatch.setattr(manager.HARDWARE, "get_device_type", lambda: "pc")

    manager.manager_init()

    assert [event[0] for event in events] == ["enforce", "register", "sentry"]
    assert events[0][1] is params

  @unittest.skip("this test is flaky the way it's currently written, should be moved to test_onroad")
  def test_clean_exit(self, subtests):
    """
      Ensure all processes exit cleanly when stopped.
    """
    HARDWARE.set_power_save(False)
    manager.manager_init()

    CP = car.CarParams.new_message()
    procs = ensure_running(managed_processes.values(), True, Params(), CP, not_run=BLACKLIST_PROCS)

    time.sleep(10)

    for p in procs:
      with subtests.test(proc=p.name):
        state = p.get_process_state_msg()
        assert state.running, f"{p.name} not running"
        exit_code = p.stop(retry=False)

        assert p.name not in BLACKLIST_PROCS, f"{p.name} was started"

        assert exit_code is not None, f"{p.name} failed to exit"

        # TODO: interrupted blocking read exits with 1 in cereal. use a more unique return code
        exit_codes = [0, 1]
        if p.sigkill:
          exit_codes = [-signal.SIGKILL]
        assert exit_code in exit_codes, f"{p.name} died with {exit_code}"


if __name__ == "__main__":
  unittest.main()
