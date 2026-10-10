"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

jetlink on this fork: the one module that holds every openpilot import
jetlink needs, and the functions the hooks call.

jetlink (the jetlink_repo submodule) runs the large driving model on an
attached Jetson, Mac or iPhone and never imports openpilot. It defines what it
needs as an interface (jetlink.openpilot.interface.Openpilot), and Adapter
below implements it. The hooks in manager, modeld, hardwared, the model
manager and the UI call the functions at the bottom, which answer as if the
link were off when jetlink is not checked out, or speaks another API.

comma's chestnut board is not jetlink's business: modeld, hardwared and the
UI handle it natively, and jetlink keeps the link off while one is fitted.

manager imports this module to build the process list, and runs it as
jetlinkd, the resident gadget owner, which stays at about 10 MB for the whole
drive. So the top level is the standard library only; everything else is
imported where it is used. tests/test_adapter.py holds the line.
"""
from __future__ import annotations

import os
import threading
from collections import namedtuple
from pathlib import Path

# the version of jetlink.openpilot's API this adapter is written to; any other
# is treated as jetlink being absent, with the reason as the offroad alert
API = 2

# the gadget owner, as manager names the process and selfdrived lists it
OWNER = 'jetlinkd'

# the Jetlink setting, stored as an index: jetlink.openpilot.MODES,
# written out so the panels build the setting without a jetlink checkout
MODES = ('off', 'usb', 'ios')

# the params jetlink reads and writes, all declared in params_keys.h. big_model
# and catalog are the model manager's big-model slot and catalog
# (models.helpers.ACTIVE_BUNDLE_KEYS['chestnut'] and ModelFetcher's cache): the
# owner cannot import the model manager, so they are written out here
_Keys = namedtuple('_Keys', 'link offroad progress spec pointers big_model catalog charge_phone')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='AcceleratorProgress', spec='JetlinkSpec',
             pointers='JetlinkModelPointers', big_model='ModelManager_ActiveBundleChestnut',
             catalog='ModelManager_ModelsCache_Chestnut', charge_phone='JetlinkChargePhone')

# comma's chestnut, running and in its ROM (common.hardware.usb): the comma's
# USB-C port hosts one and is never held as a jetlink device beside it. The
# hardware package is too heavy for the owner, so they are written out here
CHESTNUT_IDS = frozenset({(0xADD1, 0x0001), (0x3801, 0x0001), (0x174C, 0x2464), (0x174C, 0x2463)})

OWNER_LOG = Path('/data/log/jetlink-owner.log')

_AGNOS = os.path.isfile('/AGNOS')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT,
  else /data/params on a device and ~/.comma<OPENPILOT_PREFIX>/params
  elsewhere, then /<OPENPILOT_PREFIX, or d>. Per call, so it follows a prefix
  as Params does; the environment only, so it never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.path.join(os.environ.get('HOME', ''), '.comma' + prefix, 'params')
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  from jetlink.openpilot.interface import Keys, OwnerConfig

  from openpilot.common.basedir import BASEDIR
  # the provisioning run starts in the checkout with the checkout on its path,
  # where launch_chffrplus.sh links jetlink_repo/jetlink in as jetlink
  return OwnerConfig(params_dir=_params_dir(), keys=Keys(**KEYS._asdict()), chestnut_ids=CHESTNUT_IDS,
                     adapter=__name__, cwd=Path(BASEDIR), env={'PYTHONPATH': BASEDIR}, log_file=OWNER_LOG)


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process."""
  from jetlink.openpilot.owner import main as run_owner
  run_owner(owner_config())


def adapter() -> Adapter:
  """The adapter, for jetlink's entry points that run as their own process:
  the provisioning run."""
  return Adapter()


class Adapter:
  """jetlink.openpilot.interface.Openpilot over this fork."""

  def __init__(self):
    from jetlink.openpilot.interface import Keys

    from openpilot.common.basedir import BASEDIR
    from openpilot.common.swaglog import cloudlog
    self.keys = Keys(**KEYS._asdict())
    self.log = cloudlog
    self.basedir = Path(BASEDIR)
    # one Params per store: constructing one costs 144 us on the comma against
    # 110 us for the read, and the UI reads several five times a second. By
    # store, since a test or a bench runs under its own prefix
    self._stores: dict[Path, object] = {}

  # -- params ---------------------------------------------------------------

  def params_dir(self) -> Path:
    return _params_dir()

  def _params(self):
    where = _params_dir()
    store = self._stores.get(where)
    if store is None:
      from openpilot.common.params import Params
      store = self._stores[where] = Params()
    return store

  def get(self, key: str):
    # read from hardwared and the UI's threads, which UnknownKeyName (a
    # params library older than the key) must not take down
    try:
      return self._params().get(key)
    except Exception:
      return None

  def put(self, key: str, value, *, block: bool = False) -> None:
    self._params().put(key, value, block=block)

  def remove(self, key: str) -> None:
    self._params().remove(key)

  # -- the device -------------------------------------------------------------

  def chestnut_present(self) -> bool:
    from openpilot.selfdrive.modeld.helpers import chestnut_present
    return chestnut_present()

  def camera(self) -> tuple[int, int, int, int]:
    # this device's road camera, which names the warp modeld loads
    from openpilot.common.hardware import HARDWARE
    from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
    from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
    camera = _os_fisheye if HARDWARE.get_device_type() == "mici" else _ar_ox_fisheye
    return camera.width, camera.height, *MEDMODEL_INPUT_SIZE

  def warp_path(self, cam_w: int, cam_h: int, model_w: int, model_h: int) -> Path:
    # the warp modeld_tinygrad loads, which every checkout carries (LFS); stock
    # modeld loads the same compile from its own models dir, which only scons
    # fills. One per camera, for the one model input size, which jetlink
    # checks the warp makes
    return self.basedir / 'openpilot/sunnypilot/modeld_v2/models' / f'driving_warp_{cam_w}x{cam_h}_tinygrad.pkl'

  def model_root(self) -> Path:
    from openpilot.common.hardware.hw import Paths
    return Path(Paths.model_root())

  @property
  def catalog_selector(self) -> int:
    from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
    return REQUIRED_JSON_VERSION

  # -- modeld -----------------------------------------------------------------

  def model_face(self):
    """comma's large model's face: stock modeld's Parser, constants and action
    function, and modeld_v2's ModelConstants, which modeld_tinygrad reads off
    the model."""
    from jetlink.openpilot.interface import ModelFace

    from openpilot.selfdrive.modeld.constants import ModelConstants
    from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, get_action_from_model
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    from openpilot.sunnypilot.modeld_v2.constants import ModelConstants as V2ModelConstants
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    return ModelFace(parser=Parser, frame_size=lambda w, h: get_nv12_info(w, h)[3], desire_len=ModelConstants.DESIRE_LEN,
                     constants=V2ModelConstants, lat_smooth_seconds=LAT_SMOOTH_SECONDS,
                     long_smooth_seconds=LONG_SMOOTH_SECONDS, get_action_from_model=get_action_from_model)

  def event(self, name: str, **fields) -> None:
    self.log.event(name, **fields)


# -- what the hooks call ------------------------------------------------------

# jetlink's answers when it cannot run here: not checked out, or a package this
# build cannot use. The link off, by Jetlink method; a callable is handed the
# hook's arguments
_DEFAULTS = {'enabled': False, 'status': None, 'reason': None, 'prepare': False, 'attach': None,
             'request_shutdown': False, 'shutdown_pending': False, 'model_state': None,
             'should_extend_catalog': False, 'extend_catalog': lambda catalog: catalog}

# jetlink for this process, or why it cannot run here ('' with no checkout);
# None until first use
_bound = None
_binding = threading.Lock()


def _api():
  """jetlink for this process, bound to the adapter on first use, or None when
  it cannot run here. Kept, the null answer included: Python does not cache a
  failed import, and searching the path again on every UI and hardwared call
  costs more than the call. One per process: prepare() and attach() have to
  reach the same one."""
  global _bound
  if _bound is None:
    with _binding:
      if _bound is None:
        _bound = _bind()
  return None if isinstance(_bound, str) else _bound


def _bind():
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return ''   # an empty jetlink_repo, which Python takes for a namespace package
    import jetlink.openpilot as jl
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return ''   # no checkout: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("jetlink package too old for this build", e)
    return _unusable(f"jetlink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"jetlink failed to load: {type(e).__name__}: {e}", e)
  api = getattr(jl, 'API', None)
  if api != API:
    return _unusable(f"jetlink package API {api}, this build expects {API}")
  try:
    return jl.bind(Adapter())
  except Exception as e:
    return _unusable(f"jetlink failed to start: {type(e).__name__}: {e}", e)


def _unusable(why: str, error: Exception | None = None) -> str:
  _log_failure(why, error)
  return why


# manager, hardwared, the model manager and the UI call in here on every
# device, link on or off, and modeld on every drive: whatever jetlink does
# wrong turns the link off and is logged, and never takes one of them down.
# jetlink's own readers never raise; this is the net under that promise.
# Hook -> the failure last logged for it, cleared by a call that works
_failed_hooks: dict[str, str] = {}


def _log_failure(what: str, error: Exception | None) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.error("jetlink: %s", what, exc_info=error)
  except Exception:
    pass


def _failed(name: str, error: Exception) -> None:
  # once per distinct error, as jetlink's readers log: the UI would log a
  # failing status five times a second
  text = f"{type(error).__name__}: {error}"
  if _failed_hooks.get(name) != text:
    _failed_hooks[name] = text
    _log_failure(f"{name}() failed", error)


def _hook(name: str, *args):
  """Jetlink.<name>(*args), or the link off (_DEFAULTS) when jetlink cannot run
  here or raises."""
  default = _DEFAULTS[name]
  try:
    api = _api()
    if api is not None:
      result = getattr(api, name)(*args)
      _failed_hooks.pop(name, None)
      return result
  except Exception as e:
    _failed(name, e)
  return default(*args) if callable(default) else default


def should_run(started: bool, params, CP) -> bool:
  """manager's rule for jetlinkd: the link is on and no chestnut is fitted.
  jetlinkd runs onroad too: a gadget whose owner exits leaves the bus."""
  return _hook('enabled')


def status():
  """One snapshot for the UI and the panels (jetlink.openpilot.Status), or
  None when there is no jetlink here."""
  return _hook('status')


def reason() -> str | None:
  """Why the link the user turned on cannot run: hardwared's offroad alert.
  Files only, so hardwared can ask twice a second."""
  if _api() is None and _bound:
    # a package this build cannot use says why, to someone who turned the
    # link on. The setting as jetlink reads it, a file
    try:
      on = 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
    except (OSError, ValueError):
      on = False
    return _bound if on else None
  return _hook('reason')


def prepare() -> bool:
  """modeld, before config_realtime_process: will the link join this modeld?
  The GPU's setup has to happen now, or its threads inherit the frame loop's
  realtime priority and core."""
  return _hook('prepare')


# what in_control() reads; both modelds subscribe to all three
IN_CONTROL = ('carState', 'carControl', 'carControlSP')


def in_control(sm) -> bool:
  """modeld, before every frame, onto the model: is openpilot or MADS in
  control? jetlink's large model swaps in only while it is not. selfdrived's
  own answer (enabled or mads.enabled, what accelerator_events is handed),
  as controlsd republishes it, read off modeld's SubMaster. MADS counts with
  its lateral paused (a stop, a blinker, the brake): it steers again on its
  own. A service late or invalid counts as in control; without carState a
  card that died would have the swap land on stale controls."""
  try:
    result = not (sm.all_alive(IN_CONTROL) and sm.all_valid(IN_CONTROL)) or \
      bool(sm['carControl'].enabled or sm['carControlSP'].mads.enabled)
  except Exception as e:
    _failed('in_control', e)
    return True
  _failed_hooks.pop('in_control', None)
  return result


def attach(small, cam_w: int, cam_h: int):
  """modeld, once the camera is up and `small` is built: the model to run,
  `small` driving until the link has joined; None unless prepare() said yes."""
  return _hook('attach', small, cam_w, cam_h)


def request_shutdown(reason: str = '') -> bool:
  """hardwared, once, when the comma is about to power off for good: ask for
  the far end to go down with it. Returns at once: True when the request now
  waits for jetlinkd, which shutdown_pending() follows."""
  return _hook('request_shutdown', reason)


def shutdown_pending() -> bool:
  """hardwared, every loop after request_shutdown(), until it puts DoShutdown:
  has jetlinkd still to take the request? A stat."""
  return _hook('shutdown_pending')


def model_state(ref: str) -> str | None:
  """The big-model list, when it opens: 'ready' when the Jetson has built the
  model, 'downloaded' when its file is on the comma, else None."""
  return _hook('model_state', ref)


def should_extend_catalog() -> bool:
  """Should the big-model catalog carry the models newer catalogs list?
  Hardware, not the link setting: the model manager drops a pick its catalog
  does not list."""
  return _hook('should_extend_catalog')


def extend_catalog(catalog: dict) -> dict:
  """The big-model catalog with those models folded in."""
  return _hook('extend_catalog', catalog)
