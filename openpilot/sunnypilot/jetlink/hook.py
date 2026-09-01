"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The seam modeld calls into: two functions, and nothing else.

Keeping it to this is what holds the patch against upstream's
selfdrive/modeld/modeld.py to an import and a few lines - easy to review, easy
to rebase across an upstream sync, and easy for another fork to lift. Anything
only jetlinkd needs lives in helpers or spec_cache, not here.
"""
from __future__ import annotations

import time

from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.jetlink import helpers, spec_cache

# modeld loads the big model in a thread with BIG_MODEL_TIMEOUT = 60 s, so we
# have most of a minute to play with. Use it: the comma and the Jetson power up
# on different rails, usually the comma first, and a Jetson still booting when
# the car goes onroad would otherwise cost the big model for the whole drive -
# openpilot's fallback to the small model is one-way.
CONNECT_TIMEOUT = 45.0
CONNECT_DELAY = 0.5


def available() -> bool:
  """Can modeld run the large model over the link right now?

  Cheap and side-effect free: no link IO, no file reads. Provisioning is
  jetlinkd's job; by the time modeld starts the answer is already a param.

  Never raises. modeld calls this before it has a model, so anything thrown
  here takes the whole process down - including a plain ImportError on a device
  where the jetlink package simply is not installed.
  """
  try:
    if not helpers.enabled():
      return False
    spec = spec_cache.load()
    return spec is not None and helpers.engine_ready_for(spec.sha256)
  except Exception:
    cloudlog.exception("jetlink: availability check failed, staying on the small model")
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


def make_model_state(cam_w: int, cam_h: int):
  """Build a ModelState backed by the Jetson, or raise so modeld falls back."""
  from openpilot.sunnypilot.jetlink.model_state import JetlinkModelState

  spec = spec_cache.load()
  if spec is None:
    raise RuntimeError("no cached jetlink model spec; jetlinkd has not provisioned")

  # Two things can keep us waiting here, and both resolve on their own:
  # manager stops jetlinkd and starts modeld in the same pass without waiting,
  # so the endpoints may still be held; and the Jetson may still be booting.
  client = _connect_patiently()

  hello = client.hello(timeout=10.0)
  cloudlog.warning("jetlink: %s trt %s, engine %s",
                   hello.get('device'), hello.get('trt_version'), hello.get('engine_state'))
  # The engine is already built and cached; this only loads it, ~1 s.
  client.ensure_engine('/nonexistent', spec=spec, build_timeout=120.0)
  return JetlinkModelState(cam_w, cam_h, client, spec)
