"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The background fetch, with a shell script standing in for git lfs so the
process handling (pause, retry, a child left running) is the real thing.
"""
import os
import time
from pathlib import Path
from unittest import mock

from openpilot.sunnypilot.models import big_model_source as B
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeClock

POINTER = b"version https://git-lfs.github.com/spec/v1\noid sha256:" + b"a" * 64 + b"\nsize 766040736\n"
MODEL = b'\x80\x05pkl'


def model(tmp_path: Path, body: bytes) -> Path:
  path = tmp_path / 'openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl'
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(body)
  return path


def fake_git(script: str):
  return mock.patch.object(B, 'pull_command', lambda path, repo: ['sh', '-c', script])


def wait_until(condition, timeout=5.):
  deadline = time.monotonic() + timeout
  while not condition():
    assert time.monotonic() < deadline, "timed out"
    time.sleep(0.01)


def settle(fetcher, *args):
  # until the stand-in exits: update() only notices on its next call
  wait_until(lambda: fetcher.proc.poll() is not None)
  return fetcher.update(*args)


def alive(pid):
  try:
    os.kill(pid, 0)
    return True
  except ProcessLookupError:
    return False


def test_the_pull_is_one_file_past_the_exclude_and_niced(tmp_path):
  path = model(tmp_path, POINTER)
  cmd = B.pull_command(path, str(tmp_path))
  assert cmd[:3] == ['nice', '-n', '19']
  assert cmd[cmd.index('git'):] == ['git', 'lfs', 'pull', '--include', 'openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl',
                                    '--exclude', '']


def test_nothing_happens_without_a_chestnut_or_with_the_model_there(tmp_path):
  for body, seen in ((POINTER, False), (MODEL, True)):
    path = model(tmp_path, body)
    fetcher = B.BigModelFetcher(path, str(tmp_path))
    with mock.patch.object(B.subprocess, 'Popen') as popen:
      assert fetcher.update(True, True, seen) == ''
    popen.assert_not_called()


def test_a_pointer_waits_for_offroad_and_online(tmp_path):
  path = model(tmp_path, POINTER)
  fetcher = B.BigModelFetcher(path, str(tmp_path))
  with mock.patch.object(B.subprocess, 'Popen') as popen:
    for offroad, online in ((False, True), (True, False), (False, False)):
      assert fetcher.update(offroad, online, True) == 'waiting'
  popen.assert_not_called()


def test_a_pointer_is_fetched_in_the_background(tmp_path):
  path = model(tmp_path, POINTER)
  fetcher = B.BigModelFetcher(path, str(tmp_path))
  with fake_git(f"sleep 0.2; printf model > '{path}'"):
    assert fetcher.update(True, True, True) == 'downloading'
    assert fetcher.update(True, True, True) == 'downloading'
    assert settle(fetcher, True, True, True) == ''
  assert path.read_bytes() == b'model' and fetcher.proc is None


def test_going_onroad_stops_the_download_and_its_children(tmp_path):
  path = model(tmp_path, POINTER)
  child = tmp_path / 'child.pid'
  fetcher = B.BigModelFetcher(path, str(tmp_path))
  # git runs git-lfs as a child: the child is the download and must stop too
  with fake_git(f"sleep 30 & echo $! > '{child}'; wait"):
    assert fetcher.update(True, True, True) == 'downloading'
    wait_until(lambda: child.exists() and child.read_text().strip())
    pid = int(child.read_text())
    # signalled, never waited on: hardwared's loop goes on
    assert fetcher.update(False, True, True) == 'waiting'
    wait_until(lambda: fetcher.update(False, True, True) == 'waiting' and fetcher.proc is None)
  wait_until(lambda: not alive(pid))
  # a pause is not a failure: offroad again, it resumes at once
  with fake_git(f"printf model > '{path}'"):
    assert fetcher.update(True, True, True) == 'downloading'
    assert settle(fetcher, True, True, True) == ''


def test_a_fetch_that_ignores_sigterm_is_killed(tmp_path):
  path = model(tmp_path, POINTER)
  clock = FakeClock()
  fetcher = B.BigModelFetcher(path, str(tmp_path), now=clock)
  with fake_git("trap '' TERM; while :; do sleep 0.05; done"):
    assert fetcher.update(True, True, True) == 'downloading'
    time.sleep(0.2)  # the trap is set
    fetcher.update(False, True, True)
    time.sleep(0.2)
    assert fetcher.proc.poll() is None
    clock.t = B.STOP_GRACE
    wait_until(lambda: fetcher.update(False, True, True) == 'waiting' and fetcher.proc is None)


def test_a_failed_fetch_backs_off_then_retries(tmp_path):
  path = model(tmp_path, POINTER)
  clock = FakeClock()
  fetcher = B.BigModelFetcher(path, str(tmp_path), now=clock)
  with fake_git("echo 'batch response: 404' >&2; exit 2"):
    assert fetcher.update(True, True, True) == 'downloading'
    assert settle(fetcher, True, True, True) == 'waiting'
    clock.t = B.RETRY_BACKOFF[0] - 1
    assert fetcher.update(True, True, True) == 'waiting' and fetcher.proc is None
  # "succeeded" but left the pointer is a failure too
  clock.t = B.RETRY_BACKOFF[0]
  with fake_git("exit 0"):
    assert fetcher.update(True, True, True) == 'downloading'
    assert settle(fetcher, True, True, True) == 'waiting'
  assert fetcher.failures == 2


def test_the_backoff_stops_growing(tmp_path):
  path = model(tmp_path, POINTER)
  clock = FakeClock()
  fetcher = B.BigModelFetcher(path, str(tmp_path), now=clock)
  with mock.patch.object(B.subprocess, 'Popen', side_effect=FileNotFoundError('git')):
    for _ in range(len(B.RETRY_BACKOFF) + 2):
      clock.t = fetcher.next_try
      assert fetcher.update(True, True, True) == 'waiting'
  assert fetcher.next_try - clock.t == B.RETRY_BACKOFF[-1]


def test_a_pointer_is_not_compiled(tmp_path):
  # the big warps are tracked, so only the model tells a fetched install from a pointer
  from openpilot.selfdrive.modeld import helpers
  path = model(tmp_path, POINTER)
  for size in ('1344x760', '1928x1208'):
    (path.parent / f'big_driving_warp_{size}_tinygrad.pkl').write_bytes(b'w')
  with mock.patch.object(helpers, 'MODELS_DIR', path.parent), mock.patch.object(helpers, 'modeld_pkl_path', lambda chestnut: path):
    assert not helpers.chestnut_compiled()
    path.write_bytes(MODEL)
    assert helpers.chestnut_compiled()
