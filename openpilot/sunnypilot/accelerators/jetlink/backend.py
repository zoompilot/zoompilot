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

import threading
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
# How long the load may wait for the early gadget bind. Sub-second when the
# endpoints are free; jetlinkd may still be letting go of them.
PRESENT_TIMEOUT = 5.0

# How long hardwared waits for jetlinkd to shut the Jetson down before the comma
# goes ahead without it. Wake from suspend is ~8 s to a server on the bench.
SHUTDOWN_TIMEOUT = 25.0


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


def _present_early(ready: dict) -> None:
  """Open and bind the gadget now, from a thread that is not modeld's.

  Not on the caller's thread: this runs from modeld's loader thread, which
  inherited SCHED_FIFO 54 pinned to core 7 from main, and the FunctionFS
  reader thread the open creates would inherit that in turn and preempt the
  frame loop for the drive (see joining._background_priority). A helper that
  drops realtime first is what the reader inherits from instead. Bounded, so a
  hung open cannot hold modeld's load; a helper that finishes after we stopped
  waiting closes what it opened rather than leaving the gadget held by nobody.
  """
  from openpilot.sunnypilot.accelerators.jetlink.joining import _background_priority
  lock = threading.Lock()

  deadline = time.monotonic() + PRESENT_TIMEOUT

  def present():
    _background_priority()
    # Retried, not attempted once. The one moment this runs is the moment
    # manager has just stopped jetlinkd to start modeld, and jetlinkd may not
    # have let go of ep0 yet: a single try loses the early bind in exactly the
    # case it exists for, and fails in milliseconds rather than using the
    # window. The caller stops waiting at the same deadline either way.
    client = None
    while client is None:
      try:
        client = helpers.connect()
      except Exception as e:
        if time.monotonic() >= deadline:
          cloudlog.warning("jetlink: could not present the gadget early (%s), the join will", e)
          return
        time.sleep(0.2)
    with lock:
      if ready.get('abandoned'):
        client.close()
      else:
        ready['client'] = client

  t = threading.Thread(target=present, name='jetlink-present', daemon=True)
  t.start()
  t.join(max(0.0, deadline - time.monotonic()) + 0.5)
  with lock:
    if t.is_alive():
      ready['abandoned'] = True
      cloudlog.warning("jetlink: presenting the gadget took over %.0f s, the join will", PRESENT_TIMEOUT)


def _connect_patiently(client=None):
  """Open the link, tolerating a busy gadget or a Jetson that is still booting.

  A `client` already holding the gadget (presented early, see make_model_state)
  skips the open and goes straight to waiting for the host."""
  deadline = time.monotonic() + CONNECT_TIMEOUT
  last = None
  while True:
    if client is None:
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
    from openpilot.sunnypilot.accelerators.jetlink import warp_cache

    # The warp, loaded and warmed here rather than at the swap. This runs on
    # modeld's loader thread while its main thread is blocked in loader.join,
    # which is the one moment the GPU is idle and there are no frames to
    # drop; the swap itself lands on the frame loop. Sized from the cached
    # spec, which is what the link will hand back, so the swap can use it
    # without a load. Should the server answer with another geometry, build
    # falls back to loading the right warp there, slow but correct.
    ready: dict = {}

    def prepare():
      # The gadget first, so the Jetson enumerates and the server opens us
      # while the warp loads. Left to the join thread, the bind landed ~3 s
      # later than this on every ignition of the 2026-09-04 drives: that thread
      # starts as the small model runs its first frame, 1.3 s of tinygrad
      # holding the GIL, and the connect trails it. It also puts the bind, and
      # the USB enumeration it triggers, before any frame is in flight; one
      # ignition that day had a 655 ms small-model frame during the bind and
      # 17 s of modeldLagging for it. Failure here costs nothing: the join
      # thread opens the link itself if there is nothing to take over.
      _present_early(ready)
      cached = spec_cache.load()
      if cached is not None:
        img_h, img_w = cached.input_shapes['img'][2:]
        geometry = (img_w * 2, img_h * 2)
      else:
        geometry = warp_cache.device_geometry()[2:]
      warp = warp_cache.load_warp(cam_w, cam_h, *geometry)
      warp_cache.warm(warp, cam_w, cam_h)
      ready.update(warp=warp, geometry=geometry)

    def build(client, spec):
      from openpilot.sunnypilot.accelerators.jetlink.model_state import JetlinkModelState
      img_h, img_w = spec.input_shapes['img'][2:]
      warp = ready.get('warp') if ready.get('geometry') == (img_w * 2, img_h * 2) else None
      return JetlinkModelState(cam_w, cam_h, client, spec, small, warp=warp)

    def connect():
      # The early client is good for one attempt: after that the join thread
      # opens its own, as it does for every rejoin.
      return self._open_link(ready.pop('client', None))

    return JoiningModelState(cam_w, cam_h, small, connect, build, prepare)

  def _open_link(self, client=None):
    """Get a client and a spec. Link IO only, so it is safe off modeld's thread.

    Everything that touches tinygrad stays in `build`, on modeld's own thread:
    unpickling the warp JIT next to the small model running frames on the same
    device is not a race worth taking.
    """
    from jetlink.client import EngineMissing

    cached = spec_cache.load()
    if cached is None:
      if client is not None:
        client.close()
      raise RuntimeError("no cached jetlink model spec; jetlinkd has not provisioned")

    # Two things can keep us waiting, and both resolve on their own: manager
    # stops jetlinkd and starts modeld in the same pass without waiting, so the
    # endpoints may still be held; and the Jetson may still be booting.
    client = _connect_patiently(client)
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

  def shutdown(self, reason: str) -> None:
    """Take the Jetson down with us. Runs in hardwared, so it cannot touch the
    link itself: jetlinkd owns the gadget offroad, and a Jetson that is asleep
    has to be woken by presenting it, which only the owner can do. Hand the
    request over and wait; jetlinkd needs ~10 s for the wake and one round
    trip, and manager will not kill it until after we return.

    Skipped when no Jetson is known to be there, dormant counts as there. Not
    skipped when jetlinkd is busy in a long provision: it will not see the
    request in time, and the timeout is what covers that.
    """
    if not helpers.enabled() or not helpers.gadget_present():
      return
    cloudlog.warning("jetlink: asking the jetson to power off: %s", reason)
    if not helpers.request_shutdown(reason):
      return
    if helpers.await_shutdown(SHUTDOWN_TIMEOUT):
      cloudlog.warning("jetlink: shutdown request handed to the jetson")
    else:
      cloudlog.warning("jetlink: nobody took the shutdown request within %.0f s", SHUTDOWN_TIMEOUT)
