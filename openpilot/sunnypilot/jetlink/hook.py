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

CONNECT_ATTEMPTS = 10
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


def make_model_state(cam_w: int, cam_h: int):
  """Build a ModelState backed by the Jetson, or raise so modeld falls back."""
  from openpilot.sunnypilot.jetlink.model_state import JetlinkModelState

  spec = spec_cache.load()
  if spec is None:
    raise RuntimeError("no cached jetlink model spec; jetlinkd has not provisioned")

  # manager stops jetlinkd and starts modeld in the same pass without waiting,
  # so the FunctionFS endpoints can still be held for a moment. Retry rather
  # than spend the whole drive on the small model over a race.
  for attempt in range(CONNECT_ATTEMPTS):
    try:
      client = helpers.connect()
      break
    except Exception as e:
      if attempt == CONNECT_ATTEMPTS - 1:
        raise
      cloudlog.warning("jetlink: link busy (%s), retrying", e)
      time.sleep(CONNECT_DELAY)

  hello = client.hello()
  cloudlog.warning("jetlink: %s trt %s, engine %s",
                   hello.get('device'), hello.get('trt_version'), hello.get('engine_state'))
  # The engine is already built and cached; this only loads it, ~1 s.
  client.ensure_engine('/nonexistent', spec=spec, build_timeout=120.0)
  return JetlinkModelState(cam_w, cam_h, client, spec)
