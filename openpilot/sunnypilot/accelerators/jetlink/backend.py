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

# How long one attempt holds the gadget open waiting for a host. This is no
# longer a deadline on the large model: make_model_state returns immediately and
# JoiningModelState keeps retrying for the life of the drive, because the Jetson
# is on the ignition rail and its boot starts after the comma is already onroad.
# Holding the gadget while we wait is the point - the Jetson sees us the moment
# it enumerates rather than on our next poll.
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

  def prepare(self) -> bool:
    # Nothing process-wide to set up, and nothing to wait for on the link:
    # make_model_state returns immediately now and joins in the background.
    #
    # The warp is the one thing worth refusing on, and it is not a race we can
    # join our way out of: only jetlinkd compiles it and manager runs jetlinkd
    # offroad only, so a warp that is missing when modeld starts stays missing
    # for the whole drive. Saying no here keeps ChestnutActive honest instead of
    # leaving a joining thread to retry something that cannot arrive. The
    # constructor still checks, against the geometry modeld actually has.
    from openpilot.sunnypilot.accelerators.jetlink import warp_cache
    if not warp_cache.is_cached(*warp_cache.device_geometry()):
      cloudlog.warning("jetlink: no warp compiled yet, staying on the small model")
      return False
    return True

  def make_model_state(self, cam_w: int, cam_h: int, small=None):
    # Returns straight away with the small model in the driving seat. See
    # joining.py for why modeld must not be made to wait for a Jetson.
    from openpilot.sunnypilot.accelerators.jetlink.joining import JoiningModelState

    def build(client, spec):
      from openpilot.sunnypilot.accelerators.jetlink.model_state import JetlinkModelState
      return JetlinkModelState(cam_w, cam_h, client, spec, small)

    return JoiningModelState(cam_w, cam_h, small, self._open_link, build)

  def _open_link(self):
    """Get a client and a spec. Link IO only, so it is safe off modeld's thread.

    Everything that touches tinygrad stays in `build`, on modeld's own thread:
    unpickling the warp JIT next to the small model running frames on the same
    device is not a race worth taking.
    """
    from jetlink.client import EngineMissing

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
      return client, spec
    except BaseException:
      client.close()
      raise

  def make_health_publisher(self, pm, model):
    from openpilot.sunnypilot.accelerators.jetlink.state import JetlinkHealth
    # The model, not its client: with a joining state the link arrives after
    # modeld has already built this, and may go away and come back mid-drive.
    # Reading `model.client` per send is what lets chestnutState follow it.
    return JetlinkHealth(pm, model)

  def catalog(self) -> str | None:
    # Every bundle in the chestnut catalog is a tinygrad pkl for the comma's
    # own GPU; the Jetson runs ONNX from models.json instead.
    return None

  def daemon(self) -> Daemon:
    return Daemon("jetlinkd", "openpilot.sunnypilot.accelerators.jetlink.jetlinkd",
                  lambda started, params, CP: helpers.enabled())
