#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Presents the USB gadget and provisions whatever large model is selected.

Two jobs, and the first one is the reason this runs even when there is nothing
to do: **something has to hold the gadget open**. The comma is the USB device,
and its controller only binds while a process owns the FunctionFS endpoints.
Nothing holds them, nothing enumerates, and a Jetson powered on later never
appears. The two boxes come up on different power rails - usually the comma
first - so the comma has to sit there presenting itself until the Jetson
arrives, however long that takes.

The second job is provisioning: uploading the model and building a TensorRT
engine takes minutes, far longer than modeld's 60 s big-model timeout, so it
happens offroad and the result is cached on the Jetson and recorded in a param.
The engine is also left loaded on the server, so modeld's connect at ignition
is a round trip rather than a 13 to 25 s deserialization.

manager stops this daemon with SIGINT at the onroad transition and SIGKILLs it
5 s later. Every long wait in here polls `stop`, because a FunctionFS owner
killed mid-transfer leaves the gadget in a state only a reboot reliably clears.

Ownership of the link is exclusive: jetlinkd offroad, modeld onroad. manager's
only_offroad gate enforces that. On the handover the gadget briefly unbinds and
the Jetson re-enumerates, which both ends handle.
"""
from __future__ import annotations

import signal
import time

from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot import accelerators
from openpilot.sunnypilot.accelerators.jetlink import helpers, spec_cache, warp_cache

POLL_HZ = 2.0
RETRY_BACKOFF = 30.0       # after a failed provision
RETRY_BACKOFF_MAX = 900.0  # ceiling once the failures keep coming
RECONNECT_BACKOFF = 5.0    # after the link itself failed


def _timed_out(e: BaseException) -> bool:
  """Did the exchange time out with the stream still usable?

  jetlink draws that line itself: LinkTimeout is documented as leaving the
  stream in sync, every other LinkError as not.
  """
  try:
    from jetlink.transport.base import LinkTimeout
  except ImportError:
    return False  # no way to tell, so assume the worst and reopen
  return isinstance(e, LinkTimeout)


class Jetlinkd:
  def __init__(self):
    self.client = None
    self.stop = False
    self.ready = False
    self.next_attempt = 0.0
    self.next_provision = 0.0
    self.failures = 0
    self.was_attached = False
    self.fetch_failed = False
    self.verified = False   # the server has confirmed the ready param this attach
    self._identity: tuple | None = None   # (source, sha256, nbytes) of the hashed file
    self.warp_built = False  # tried the comma-side warp this run

  # -- lifecycle ------------------------------------------------------------

  def request_stop(self, *_) -> None:
    self.stop = True

  def close_link(self) -> None:
    """Always go through this. A FunctionFS owner that exits without closing
    leaves the gadget bound with nothing servicing it, and the next teardown
    can wedge the driver hard enough that only a reboot clears it."""
    client, self.client = self.client, None
    if client is not None:
      try:
        client.close()
      except Exception:
        cloudlog.exception("jetlink: error closing the link")

  def open_link(self) -> bool:
    """Present the gadget so a Jetson can enumerate whenever it powers on."""
    if self.client is not None:
      return True
    try:
      self.client = helpers.connect(deadline=5.0)
      cloudlog.warning("jetlink: gadget presented, waiting for a jetson")
      return True
    except Exception:
      cloudlog.exception("jetlink: could not present the gadget")
      self.next_attempt = time.monotonic() + RECONNECT_BACKOFF
      return False

  # -- provisioning ---------------------------------------------------------

  def fetch_model(self):
    """Download the pinned large model, once, on a device that has a Jetson.

    The install carries a pointer rather than the object, so the first time a
    Jetson is attached we have to go and get it. Minutes on a slow link, so it
    reports progress and gives up the moment manager wants us gone - the loop
    is single threaded and this is the one call in it that blocks for long.
    """
    if self.fetch_failed:
      return None
    try:
      path = helpers.fetch_shipped_model(
        progress=lambda frac: accelerators.report_progress('download', frac, 'downloading the large model'),
        should_stop=lambda: self.stop,
      )
    except Exception:
      cloudlog.exception("jetlink: could not fetch the large model")
      accelerators.report_progress('failed', 1.0, 'could not download the large model')
      # One attempt per run. Retrying a gigabyte on a loop would be worse than
      # staying on the small model until the next boot.
      self.fetch_failed = True
      return None
    return path

  def build_warp(self) -> None:
    """Compile the comma-side warp, once per run, while we are parked.

    Not part of provision(): the warp depends only on this device's camera and
    the small model's input size, not on which large model is selected or on
    the Jetson answering. A Jetson that never provisions still leaves a warp
    ready for the next one, and a warp that cannot be built costs the large
    model, not the drive - modeld falls back exactly as it does for any other
    big-model load failure.
    """
    if self.warp_built:
      return
    self.warp_built = True
    accelerators.report_progress('warp', 0.0, 'compiling the camera warp')
    if warp_cache.ensure(*warp_cache.device_geometry()):
      accelerators.clear_progress()

  def provision(self) -> bool:
    """Make the Jetson ready for the selected model. Host must be attached."""
    model_path = helpers.active_model_path()
    if model_path is None:
      model_path = self.fetch_model()
    if model_path is None:
      # Nothing selected, or still downloading. Not an error.
      helpers.set_engine_ready(None)
      accelerators.clear_progress()
      return False

    # Steady state must not touch the model file: hashing 766 MB takes longer
    # than this loop's period, so doing it per poll would peg a core for as
    # long as the car is parked. The identity is cached against the file it
    # came from and nothing else - keying it on the engine being ready too
    # would re-hash the file on every retry against a Jetson that is not
    # serving. The shapes come back from the server, which is the only side
    # that parses the ONNX: a stock device has no parser for every export.
    st = model_path.stat()
    source = (str(model_path), st.st_mtime_ns, st.st_size)
    cached = spec_cache.load()
    if cached is not None and spec_cache.source() == source:
      sha256, nbytes = cached.sha256, cached.nbytes
    elif self._identity is not None and self._identity[0] == source:
      _, sha256, nbytes = self._identity
    else:
      from jetlink.spec import sha256_file
      sha256, nbytes = sha256_file(str(model_path))
      # Remembered here as well as in the spec param, because the param is
      # only written once the server has answered, and a server that never
      # answers is the case the retry loop exists for.
      self._identity = (source, sha256, nbytes)

    # The param says ready, but the Jetson's cache may have been pruned,
    # re-flashed or swapped since. Ask once per attach; after that the answer
    # cannot change under us.
    if self.verified and helpers.engine_ready_for(sha256):
      return True

    cloudlog.warning("jetlink: provisioning %s (%d MB, sha %s)",
                     model_path.name, nbytes >> 20, sha256[:16])
    accelerators.report_progress('connect', 0.0, 'talking to the jetson')

    hello = self.client.hello(timeout=10.0)
    cloudlog.warning("jetlink: server %s trt %s", hello.get('device'), hello.get('trt_version'))
    spec = self.client.ensure_engine(sha256, nbytes, onnx_path=model_path,
                                     progress=accelerators.report_progress,
                                     build_timeout=1800.0, should_stop=lambda: self.stop)

    spec_cache.store(spec, model_path)
    helpers.set_engine_ready(spec.sha256)
    self.verified = True
    accelerators.report_progress('ready', 1.0, 'engine ready')
    helpers.cleanup_unchunked(keep=model_path)
    cloudlog.warning("jetlink: engine ready for %s", spec.sha256[:16])
    return True

  # -- main loop ------------------------------------------------------------

  def backoff(self) -> float:
    """How long to wait before provisioning again, doubling per failure.

    A Jetson that is powered but never answers is the case this exists for:
    at a flat 30 s it would be retried 120 times an hour for as long as the
    car is parked, and every one of those costs a round of USB churn.
    """
    return min(RETRY_BACKOFF * 2 ** (self.failures - 1), RETRY_BACKOFF_MAX)

  def step(self) -> None:
    if not helpers.enabled():
      if self.client is not None or self.ready:
        cloudlog.warning("jetlink: disabled, releasing the link")
        helpers.set_engine_ready(None)
        self.ready = False
        self.close_link()
      return

    if time.monotonic() < self.next_attempt:
      return
    if not self.open_link():
      return

    attached = helpers.host_attached()
    if attached != self.was_attached:
      cloudlog.warning("jetlink: jetson %s", "attached" if attached else "gone")
      self.was_attached = attached
      if attached:
        # A host that has just arrived gets a clean slate rather than sitting
        # out a backoff earned by whatever was on the link before it, and its
        # engine cache is checked before the ready param is trusted again.
        self.failures = 0
        self.next_provision = 0.0
        self.verified = False
      else:
        # It powered down or rebooted. Readiness is about the engine on the
        # Jetson, which survives, so keep it; modeld reconnects on its own.
        self.ready = False
    if not attached:
      return
    # Ahead of the provisioning backoff: this needs no server, and a Jetson
    # that is slow to answer must not leave modeld without a warp.
    self.build_warp()
    # Provisioning backs off on its own timer, so that a long wait for an
    # unresponsive server still leaves us watching for one that reappears.
    if time.monotonic() < self.next_provision:
      return

    try:
      self.ready = self.provision()
      self.failures = 0
      self.next_provision = 0.0
    except Exception as e:
      cloudlog.exception("jetlink: provisioning failed")
      accelerators.report_progress('failed', 1.0, 'see the log')
      self.ready = False
      # Only reopen when the link itself is suspect. Unbinding the gadget
      # makes the host re-enumerate, and doing that every time a server that
      # is simply not up yet fails to answer is hours of USB churn.
      if not _timed_out(e):
        self.close_link()
      self.failures += 1
      self.next_provision = time.monotonic() + self.backoff()

  def run(self) -> None:
    rk = Ratekeeper(POLL_HZ)
    while not self.stop:
      rk.keep_time()
      try:
        self.step()
      except Exception:
        # Nothing may escape: this daemon restarting in a loop would be worse
        # than it sitting out a cycle.
        cloudlog.exception("jetlink: unhandled error")
        self.close_link()
        self.next_attempt = time.monotonic() + RECONNECT_BACKOFF
    self.close_link()
    cloudlog.warning("jetlink: stopped")


def main() -> None:
  d = Jetlinkd()
  # manager stops us with SIGTERM at the onroad transition. Closing the link
  # properly on the way out is what keeps the driver healthy for modeld.
  signal.signal(signal.SIGTERM, d.request_stop)
  signal.signal(signal.SIGINT, d.request_stop)
  d.run()


if __name__ == "__main__":
  main()
