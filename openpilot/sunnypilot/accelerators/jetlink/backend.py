"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The accelerator backend: everything core openpilot calls, and nothing else.

Holding the seam to this one class is what keeps the patch against upstream to
a handful of lines - easy to review, easy to rebase across a sync, and easy for
another fork to lift. Anything only jetlinkd needs lives in helpers or
spec_cache, not here.
"""
from __future__ import annotations

import time

from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.accelerators.base import Daemon
from openpilot.sunnypilot.accelerators.jetlink import helpers, spec_cache

# modeld loads the large model in a thread with BIG_MODEL_TIMEOUT = 60 s, so we
# have most of a minute. Use it: the comma and the Jetson power up on different
# rails, usually the comma first, and a Jetson still booting when the car goes
# onroad would otherwise cost the large model for the whole drive - openpilot's
# fallback to the small model is one-way.
CONNECT_TIMEOUT = 45.0
CONNECT_DELAY = 0.5


def _wait_for_host(deadline: float) -> bool:
  reported = False
  while time.monotonic() < deadline:
    if helpers.host_attached():
      return True
    if not reported:
      cloudlog.warning("jetlink: gadget up, waiting for the jetson to enumerate")
      reported = True
    time.sleep(CONNECT_DELAY)
  return False


def _connect_patiently():
  """Open the link, tolerating a busy gadget or a Jetson that is still booting."""
  deadline = time.monotonic() + CONNECT_TIMEOUT
  last = None
  while True:
    try:
      client = helpers.connect()
    except Exception as e:
      client, last = None, e
    if client is not None:
      if helpers.host_attached():
        return client
      # We hold the gadget now, so the Jetson can see us the moment it is up.
      # Keep it open and wait rather than churning the endpoints.
      if _wait_for_host(deadline):
        return client
      client.close()
      raise TimeoutError(f"no jetson attached within {CONNECT_TIMEOUT:.0f}s")
    if time.monotonic() > deadline:
      raise last if last is not None else TimeoutError("could not open the link")
    cloudlog.warning("jetlink: link not ready (%s), retrying", last)
    time.sleep(CONNECT_DELAY)


class JetlinkAccelerator:
  name = "jetlink"

  def present(self) -> bool:
    return helpers.gadget_present()

  def ready(self) -> bool:
    # Params only: no link IO, no file reads. Provisioning is jetlinkd's job,
    # so by the time modeld starts the answer is already recorded.
    if not helpers.enabled():
      return False
    spec = spec_cache.load()
    return spec is not None and helpers.engine_ready_for(spec.sha256)

  def unavailable_reason(self) -> str | None:
    return helpers.gadget_alert()

  def prepare(self) -> None:
    pass

  def make_model_state(self, cam_w: int, cam_h: int, small=None):
    from jetlink.client import EngineMissing
    from openpilot.sunnypilot.accelerators.jetlink.model_state import JetlinkModelState

    cached = spec_cache.load()
    if cached is None:
      raise RuntimeError("no cached jetlink model spec; jetlinkd has not provisioned")

    # Two things can keep us waiting, and both resolve on their own: manager
    # stops jetlinkd and starts modeld in the same pass without waiting, so the
    # endpoints may still be held; and the Jetson may still be booting.
    client = _connect_patiently()
    try:
      hello = client.hello(timeout=10.0)
      cloudlog.warning("jetlink: %s trt %s, engine %s, loaded %s",
                       hello.get('device'), hello.get('trt_version'),
                       hello.get('engine_state'), str(hello.get('loaded'))[:16])
      # jetlinkd left the engine loaded on the server, so this is normally one
      # round trip. If the server restarted it is a load from the plan cache,
      # 13 to 25 s measured, which the timeout has to cover.
      try:
        spec = client.ensure_engine(cached.sha256, cached.nbytes, frame_skip=cached.frame_skip,
                                    build_timeout=120.0)
      except EngineMissing:
        # The Jetson's cache was pruned, re-flashed or swapped since jetlinkd
        # recorded it as ready. Clear the record so jetlinkd provisions again
        # next time the car is parked, instead of every drive failing here.
        helpers.set_engine_ready(None)
        raise
    except BaseException:
      client.close()
      raise
    return JetlinkModelState(cam_w, cam_h, client, spec, small)

  def make_health_publisher(self, pm, model):
    from openpilot.sunnypilot.accelerators.jetlink.state import JetlinkHealth
    # No client when the large model failed to load and modeld fell back to the
    # small one; the publisher then reports an invalid message, as chestnut does.
    return JetlinkHealth(pm, getattr(model, 'client', None))

  def catalog(self) -> str | None:
    # Every bundle in the chestnut catalog is a tinygrad pkl for the comma's
    # own GPU; the Jetson runs ONNX from models.json instead.
    return None

  def daemon(self) -> Daemon:
    return Daemon("jetlinkd", "openpilot.sunnypilot.accelerators.jetlink.jetlinkd",
                  lambda started, params, CP: helpers.enabled())
