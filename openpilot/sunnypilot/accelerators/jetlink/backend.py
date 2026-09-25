"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The accelerator backend: everything core openpilot calls, and nothing else.

A module of functions behind sunnypilot.accelerators, the only thing core
openpilot imports. Anything only jetlinkd needs lives in helpers or spec_cache.
The `jetlink` client package can be absent on a device; this module imports
without it, and the functions that need it answer their negative default when
it is not there, logging once.
"""
from __future__ import annotations

import threading
import time

from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.accelerators.jetlink import helpers, spec_cache

# how long one attempt holds the gadget open waiting for a host. Not a deadline
# on the large model: JoiningModelState retries for the drive, since the Jetson
# boots after the comma is already onroad
CONNECT_TIMEOUT = 45.0
CONNECT_DELAY = 0.5
# ten frame periods; a dead server must not hold the frame thread for seconds
INFERENCE_TIMEOUT = 0.5
# how long the load may wait for the early gadget bind; jetlinkd may still be
# letting go of the endpoints
PRESENT_TIMEOUT = 5.0
# how long hardwared waits for jetlinkd to shut the Jetson down. Wake from
# suspend is ~8 s to a server
SHUTDOWN_TIMEOUT = 25.0

_missing_reported = False


def _package_missing(what: str) -> bool:
  """True, and logged once, if the jetlink client package cannot be imported."""
  global _missing_reported
  try:
    import jetlink.client  # noqa: F401
  except ImportError:
    if not _missing_reported:
      cloudlog.warning("jetlink: package not installed, %s unavailable", what)
      _missing_reported = True
    return True
  return False


class _Link:
  """modeld's end of the gadget: the lease, and the client that rides on it.

  Both are kept across join attempts. The lease never changes for the length of
  a drive, and opening the gadget again per attempt is an unplug as the Jetson
  sees it - which, while one boots and the join loop asks every few seconds, is
  an unplug a cycle. So an attempt that cannot use the link leaves it here
  rather than closing it, and only a deliberate close() lets go.
  """

  def __init__(self, name: str = 'modeld'):
    self.name = name
    self.client = None
    self.loan = None
    self._lock = threading.Lock()
    self._abandoned = False

  def open(self, deadline: float | None = None):
    """The client, opening one if we have not got one yet."""
    if self.client is None:
      self.client = helpers.connect(name=self.name, loan=self._borrow(deadline))
    return self.client

  def adopt(self, client) -> bool:
    """Take a client another thread opened. False once we have given up waiting
    for it, and then the caller closes what it opened."""
    with self._lock:
      if self._abandoned:
        return False
      self.client = client
      return True

  def abandon(self) -> None:
    with self._lock:
      self._abandoned = True

  def close(self) -> None:
    """Let the link go. The lease stays: jetlinkd should hold the gadget for
    the whole drive, however many times the join has to start over."""
    client, self.client = self.client, None
    if client is not None:
      try:
        client.close()
      except Exception:
        cloudlog.exception("jetlink: error closing the link")

  def _borrow(self, deadline: float | None):
    """The lease on the gadget jetlinkd owns, or None if there is nobody to ask.

    None is the ordinary answer on a device where the link was only just turned
    on, or whose daemon died: we open the gadget ourselves then, as modeld
    always did, so a drive never loses the large model to a daemon fault.
    """
    from openpilot.sunnypilot.accelerators.jetlink import lending
    if self.loan is not None and not self.loan.closed:
      return self.loan
    # bounded by whatever the caller has left: an early present that spends its
    # whole budget here has nothing left to open the link with
    timeout = lending.BORROW_TIMEOUT if deadline is None else max(0.0, deadline - time.monotonic())
    try:
      self.loan = lending.borrow(self.name, timeout=timeout)
    except Exception:
      cloudlog.exception("jetlink: could not ask jetlinkd for the gadget")
      self.loan = None
    return self.loan


def _present_early(link: _Link) -> None:
  """Take the link now, from a thread that is not modeld's.

  modeld's main thread is already SCHED_FIFO 54 on core 7, and the FunctionFS
  reader the open creates would inherit that and preempt the frame loop (see
  joining._background_priority). Bounded, so a hung open cannot hold modeld's
  load; a helper that finishes late closes what it opened.
  """
  from openpilot.sunnypilot.accelerators.jetlink.joining import _background_priority
  deadline = time.monotonic() + PRESENT_TIMEOUT

  def present():
    _background_priority()
    # retried: jetlinkd may still have the endpoints open from a provision it
    # was in the middle of, and a single try fails in milliseconds
    while True:
      try:
        client = link.open(deadline)
        break
      except Exception as e:
        if time.monotonic() >= deadline:
          cloudlog.warning("jetlink: could not present the gadget early (%s), the join will", e)
          return
        time.sleep(0.2)
    if not link.adopt(client):
      client.close()

  t = threading.Thread(target=present, name='jetlink-present', daemon=True)
  t.start()
  t.join(max(0.0, deadline - time.monotonic()) + 0.5)
  if t.is_alive():
    link.abandon()
    cloudlog.warning("jetlink: presenting the gadget took over %.0f s, the join will", PRESENT_TIMEOUT)


def _waiting_for_the_jetson() -> None:
  cloudlog.warning("jetlink: gadget up, waiting for the jetson to enumerate")


def _connect_patiently(link: _Link):
  """Open the link, tolerating a busy gadget or a Jetson that is still booting."""
  deadline = time.monotonic() + CONNECT_TIMEOUT
  last = None
  while True:
    try:
      client = link.open()
    except Exception as e:
      client, last = None, e
    if client is not None:
      if helpers.wait_for_host(max(0.0, deadline - time.monotonic()), bounce=client.rebind,
                               report=_waiting_for_the_jetson):
        return client
      # the link stays on `link`, still bound, for the next attempt
      raise TimeoutError(f"no jetson attached within {CONNECT_TIMEOUT:.0f}s")
    if time.monotonic() > deadline:
      # nothing is held here: this is a gadget we could not open at all,
      # usually jetlinkd still finishing an exchange on the endpoints
      raise last if last is not None else TimeoutError("could not open the link")
    cloudlog.warning("jetlink: link not ready (%s), retrying", last)
    time.sleep(CONNECT_DELAY)


# the chestnut runs the big model natively and the link stays off beside it,
# whatever the toggle says. Cached: the UI asks five times a second and the
# answer is a walk of the USB bus
CHESTNUT_TTL = 2.0
_chestnut: tuple[float, bool] | None = None


def _chestnut_fitted() -> bool:
  global _chestnut
  now = time.monotonic()
  if _chestnut is None or now - _chestnut[0] > CHESTNUT_TTL:
    from openpilot.selfdrive.modeld.helpers import chestnut_present
    _chestnut = (now, chestnut_present())
  return _chestnut[1]


def enabled() -> bool:
  return helpers.enabled() and not _chestnut_fitted()


def installed() -> bool:
  return helpers.package_installed()


def present() -> bool:
  return helpers.gadget_present()


def ready() -> bool:
  # params only, no link IO: jetlinkd has already recorded the answer
  if not enabled() or helpers.gadget_error() is not None:
    return False
  spec = spec_cache.load()
  selected = helpers.selected_model()
  return (spec is not None and selected is not None and spec.sha256 == selected['oid']
          and helpers.engine_ready_for(spec.sha256))


def unavailable_reason() -> str | None:
  return helpers.gadget_alert() if enabled() else None


def prepare() -> bool:
  # the link is not worth waiting for: make_model_state joins in the background.
  # The warp is: scons builds it before manager starts, so one missing now stays
  # missing for the drive, and saying no keeps modeld on the plain small model
  if _package_missing('the large model'):
    return False
  # modeld decides on enabled() alone, so this is where a device that cannot
  # present a gadget at all says so; nothing here would ever reach a Jetson
  if not helpers.link_configured():
    cloudlog.warning("jetlink: no usable gadget (%s), staying on the small model",
                     helpers.gadget_error() or 'not set up')
    return False
  from openpilot.sunnypilot.accelerators.jetlink import warp_cache
  if not warp_cache.is_cached(*warp_cache.device_geometry()):
    cloudlog.warning("jetlink: no warp compiled yet, staying on the small model")
    return False
  # the last hook before modeld goes SCHED_FIFO on core 7, and the GPU's init
  # spawns a thread that would inherit that. See warp_cache.init_device
  warp_cache.init_device()
  return True


def make_model_state(cam_w: int, cam_h: int, small=None):
  # returns straight away with the small model driving; see joining.py
  if _package_missing('the large model'):
    return None
  from openpilot.sunnypilot.accelerators.jetlink.joining import JoiningModelState
  from openpilot.sunnypilot.accelerators.jetlink import warp_cache

  # the warp is loaded and warmed here, before the frame loop exists, rather
  # than at the swap on a driving frame. Sized from the cached spec, which is
  # what the link will hand back; another geometry is rejected
  ready: dict = {}
  link = _Link()

  def prepare():
    from openpilot.sunnypilot.accelerators.jetlink.fallback import prepare_reset
    # the gadget first, so the Jetson enumerates while the warp loads. Left to
    # the join thread the bind landed ~3 s later, behind the small model's
    # first frame, and one ignition had a 655 ms frame during the bind
    _present_early(link)
    cached = spec_cache.load()
    if cached is not None:
      img_h, img_w = cached.model_hw
      geometry = (img_w * 2, img_h * 2)
    else:
      geometry = warp_cache.device_geometry()[2:]
    try:
      ready['reset_small'] = prepare_reset(small)
      warp = warp_cache.load_warp(cam_w, cam_h, *geometry)
      warp_cache.warm(warp, cam_w, cam_h)
    except Exception:
      link.close()
      raise
    ready.update(warp=warp, geometry=geometry)

  def build(client, spec):
    from openpilot.sunnypilot.accelerators.jetlink.model_state import JetlinkModelState
    img_h, img_w = spec.model_hw
    warp = ready.get('warp') if ready.get('geometry') == (img_w * 2, img_h * 2) else None
    if warp is None:
      raise RuntimeError('no prepared warp for the server model geometry')
    return JetlinkModelState(cam_w, cam_h, client, spec, warp=warp)

  def connect(should_stop=None):
    return _open_link(link, should_stop)

  return JoiningModelState(cam_w, cam_h, small, connect, build, prepare,
                           reset_small=lambda: ready['reset_small']())


def _open_link(link: _Link, should_stop=None):
  """Get a client and a spec. Link IO only, so it is safe off modeld's thread;
  everything that touches tinygrad stays in `build`.

  The model the user picked is built here if the Jetson has not got it. That
  takes minutes and the small model drives through all of them, which beats
  what it used to do: jetlinkd provisions offroad only, so a model picked in
  the driveway and driven off on cost the whole drive, with the link never
  even presented. Only ever reached with the small model driving - the join
  loop stops asking once it has joined - so a build here never unloads an
  engine that is steering.

  Whatever this attempt cannot use stays on `link`, still open, for the next
  one; see _Link.
  """
  from jetlink.client import EngineMissing
  from openpilot.sunnypilot.accelerators.jetlink import provision

  selected = helpers.selected_model()
  if selected is None:
    raise RuntimeError('no large model has been picked yet')

  # the endpoints may still be held by jetlinkd, and the Jetson may still be
  # booting; both resolve on their own
  client = _connect_patiently(link)
  try:
    hello = client.hello(timeout=10.0)
    cloudlog.warning("jetlink: %s trt %s, engine %s, loaded %s",
                     hello.get('device'), hello.get('trt_version'),
                     hello.get('engine_state'), str(hello.get('loaded'))[:16])
    sha256, nbytes = provision.identity(selected)
    if not helpers.engine_ready_for(sha256):
      cloudlog.warning("jetlink: %s is not built yet, building it with the small model driving",
                       selected.get('name', sha256[:16]))
    try:
      # normally one round trip, since jetlinkd left the engine loaded. A
      # server that restarted reloads from the plan cache, 13 to 25 s; one
      # that has never seen this model builds it, 102 to 294 s
      spec = provision.ensure(client, sha256, nbytes, helpers.shipped_model_path(),
                              progress=provision.report_with_eta, should_stop=should_stop)
    except EngineMissing:
      # neither end has the bytes. Fetching them needs the internet and a
      # gigabyte of it, which is a parked job; clear the record so the next
      # parked period provisions again
      helpers.set_engine_ready(None)
      raise
    client.deadline = INFERENCE_TIMEOUT
    return client, spec
  except BaseException:
    link.close()
    raise


def make_status_publisher(pm, model):
  from openpilot.sunnypilot.accelerators.jetlink.status import JetlinkStatus
  # the model, not its client: the link arrives after this is built and may
  # come and go mid-drive. Reading model.client per send follows it
  return JetlinkStatus(pm, model)


def big_catalog(catalog: dict, url: str) -> dict:
  """The big-model catalog with every newer one sunnypilot has published folded
  in, when the Jetson runs the big model. It runs the commit's ONNX, so a model
  sunnypilot only builds for its next runtime is still one it can run; see
  jetlink.registry.catalog.merge_catalogs. With a chestnut fitted, or the link
  off, the catalog is the model manager's as fetched. Never raises: a probe
  that fails leaves the catalog as it was."""
  if not helpers.enabled():
    return catalog
  try:
    from openpilot.selfdrive.modeld.helpers import chestnut_present
    if chestnut_present():
      return catalog
    from jetlink.registry.catalog import merge_catalogs, newer_catalogs
    from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
    newer = newer_catalogs(url)
    if not newer:
      return catalog
    merged = merge_catalogs([catalog, *newer], selector=REQUIRED_JSON_VERSION)
    added = len(merged.get('bundles', [])) - len(catalog.get('bundles', []))
    cloudlog.warning("jetlink: %d newer catalog(s) checked, %d model(s) only they list", len(newer), max(added, 0))
    return merged
  except Exception:
    cloudlog.exception("jetlink: could not check for newer catalogs")
    return catalog


def selected_model_name() -> str | None:
  """What the accelerator will run: the big-model slot's pick, or the default."""
  selected = helpers.selected_model()
  return selected['name'] if selected else None


def active_model_name() -> str | None:
  return selected_model_name() if ready() else None


def shutdown(reason: str, timeout: float = SHUTDOWN_TIMEOUT) -> None:
  """Take the Jetson down with the comma. Runs in hardwared, which cannot
  touch the link: jetlinkd owns the gadget offroad and is the only one that
  can wake a sleeping Jetson. Hand the request over and wait; the wake and one
  round trip take ~10 s, and manager will not kill jetlinkd until this returns.

  Skipped when no Jetson is known to be there (dormant counts as there). A
  jetlinkd busy in a long provision will not see the request; the timeout
  covers that.
  """
  if not helpers.enabled() or not helpers.gadget_present():
    return
  cloudlog.warning("jetlink: asking the jetson to power off: %s", reason)
  if not helpers.request_shutdown(reason):
    return
  if helpers.await_shutdown(timeout):
    cloudlog.warning("jetlink: shutdown request handed to the jetson")
  else:
    cloudlog.warning("jetlink: nobody took the shutdown request within %.0f s", timeout)
