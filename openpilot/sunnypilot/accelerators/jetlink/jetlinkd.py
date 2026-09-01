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
from openpilot.sunnypilot.accelerators.jetlink import helpers, spec_cache

POLL_HZ = 2.0
RETRY_BACKOFF = 30.0      # after a failed provision
RECONNECT_BACKOFF = 5.0   # after the link itself failed


class Jetlinkd:
  def __init__(self):
    self.client = None
    self.stop = False
    self.ready = False
    self.next_attempt = 0.0
    self.was_attached = False
    self.fetch_failed = False

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
    # long as the car is parked.
    st = model_path.stat()
    source = (str(model_path), st.st_mtime_ns, st.st_size)
    spec = spec_cache.load()
    if (spec_cache.source() == source and spec is not None
        and helpers.engine_ready_for(spec.sha256)):
      return True

    from jetlink.spec import spec_from_onnx
    spec = spec_from_onnx(str(model_path))

    if helpers.engine_ready_for(spec.sha256):
      spec_cache.store(spec, model_path)   # record the source so the next poll is free
      return True

    cloudlog.warning("jetlink: provisioning %s (%d MB, sha %s)",
                     model_path.name, spec.nbytes >> 20, spec.sha256[:16])
    accelerators.report_progress('connect', 0.0, 'talking to the jetson')

    hello = self.client.hello(timeout=10.0)
    cloudlog.warning("jetlink: server %s trt %s", hello.get('device'), hello.get('trt_version'))
    self.client.ensure_engine(model_path, spec=spec, progress=accelerators.report_progress,
                              build_timeout=1800.0)

    spec_cache.store(spec, model_path)
    helpers.set_engine_ready(spec.sha256)
    accelerators.report_progress('ready', 1.0, 'engine ready')
    helpers.cleanup_unchunked(keep=model_path)
    cloudlog.warning("jetlink: engine ready for %s", spec.sha256[:16])
    return True

  # -- main loop ------------------------------------------------------------

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
      if not attached:
        # It powered down or rebooted. Readiness is about the engine on the
        # Jetson, which survives, so keep it; modeld reconnects on its own.
        self.ready = False
    if not attached:
      return

    try:
      self.ready = self.provision()
      self.next_attempt = 0.0
    except Exception:
      cloudlog.exception("jetlink: provisioning failed")
      accelerators.report_progress('failed', 1.0, 'see the log')
      self.ready = False
      # Drop the link too: a half-open session is worse than a fresh one.
      self.close_link()
      self.next_attempt = time.monotonic() + RETRY_BACKOFF

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
