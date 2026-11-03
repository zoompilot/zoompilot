"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The big driving model, for a comma that has a chestnut.

zoompilot's .lfsconfig leaves the big model out of every install and update
fetch: most devices never run it, and it is about 800 MB. A comma that has
ever seen a chestnut (ChestnutSeen) fetches it in the background instead,
whenever it is offroad and on Wi-Fi, and again after an update checks out a
new pointer. The model is a prebuilt pkl, so nothing is rebuilt: modeld loads
it on the next drive.

Not at build time: a build sees the chestnut only if it has enumerated in
that second and the comma is online, and it would block boot on the download.
Not onroad: a download that size competes with the recorder for memory and
disk, which is what stalls USB transfers.
"""
import contextlib
import os
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import modeld_pkl_path

BIG_PKL = modeld_pkl_path(chestnut=True)
LFS_POINTER = b'version https://git-lfs.github.com/spec/v1'
# waits after a failed fetch, the last one repeating
RETRY_BACKOFF = (60., 300., 900., 3600.)
# how long a paused fetch has to exit before SIGKILL
STOP_GRACE = 5.


def is_lfs_pointer(path: Path) -> bool:
  try:
    with open(path, 'rb') as f:
      return f.read(len(LFS_POINTER)) == LFS_POINTER
  except OSError:
    return False


def pull_command(path: Path = BIG_PKL, repo: str = BASEDIR) -> list[str]:
  # --exclude "" drops the fetchexclude for this one pull
  cmd = ['git', 'lfs', 'pull', '--include', str(path.relative_to(repo)), '--exclude', '']
  # in the background of a comma that may be driving soon: last in line for CPU and disk
  if shutil.which('ionice'):
    cmd = ['ionice', '-c', '3'] + cmd
  return ['nice', '-n', '19'] + cmd


class BigModelFetcher:
  """hardwared's 2 Hz hook, so it never blocks. update() says what to tell the user:
  '' (nothing to fetch), 'downloading', or 'waiting' (missing, cannot be fetched now)."""

  def __init__(self, path: Path = BIG_PKL, repo: str = BASEDIR, now=time.monotonic) -> None:
    self.path, self.repo, self.now = path, repo, now
    self.proc: subprocess.Popen | None = None
    self.kill_at: float | None = None  # set while a pause waits for the group to exit
    self.err = None
    self.failures = 0
    self.next_try = 0.
    # an update takes effect at a reboot, which restarts hardwared: a fetched model stays fetched
    self.fetched = False

  def update(self, offroad: bool, online: bool, chestnut_seen: bool) -> str:
    if self.proc is not None and self.proc.poll() is not None:
      self._finish()
    elif self.proc is not None and not (offroad and online):
      self._stop(offroad, online)

    if self.fetched or not chestnut_seen:
      return ''
    if not is_lfs_pointer(self.path):
      cloudlog.warning(f"{self.path.name} is here")
      self.fetched = True
      return ''
    if self.proc is None and offroad and online and self.now() >= self.next_try:
      self._start()
    return 'downloading' if self.proc is not None and self.kill_at is None else 'waiting'

  def _start(self) -> None:
    cloudlog.warning(f"fetching {self.path.name} for the chestnut, attempt {self.failures + 1}")
    if self.err is None:
      self.err = tempfile.TemporaryFile()
    self.err.seek(0)
    self.err.truncate()
    try:
      # its own group: git runs git-lfs as a child, and the download is the child's
      self.proc = subprocess.Popen(pull_command(self.path, self.repo), cwd=self.repo,
                                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=self.err,
                                   start_new_session=True)
    except OSError as e:
      cloudlog.error(f"could not run git lfs: {e}")
      self._fail()

  def _finish(self) -> None:
    returncode, paused = self.proc.returncode, self.kill_at is not None
    self.proc, self.kill_at = None, None
    # a pause is not a failure: git lfs keeps the partial object and resumes it
    if paused or (returncode == 0 and not is_lfs_pointer(self.path)):
      return
    self.err.seek(max(0, self.err.seek(0, os.SEEK_END) - 500))
    tail = self.err.read().decode(errors='replace').strip()
    cloudlog.error(f"big model fetch failed ({returncode}): {tail}")
    self._fail()

  def _fail(self) -> None:
    self.next_try = self.now() + RETRY_BACKOFF[min(self.failures, len(RETRY_BACKOFF) - 1)]
    self.failures += 1

  def _stop(self, offroad: bool, online: bool) -> None:
    # signals only; the next updates reap it, and SIGKILL it if it outlives STOP_GRACE
    if self.kill_at is None:
      cloudlog.warning(f"big model fetch paused, offroad={offroad} online={online}")
      self.kill_at = self.now() + STOP_GRACE
      sig = signal.SIGTERM
    elif self.now() >= self.kill_at:
      sig = signal.SIGKILL
    else:
      return
    with contextlib.suppress(ProcessLookupError):
      os.killpg(self.proc.pid, sig)
