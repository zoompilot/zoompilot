"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Where the model is, whether the Jetson is attached, and how far along it is.

The large model is the model manager's big-model pick, the same slot a
chestnut runs from (see ModelManagerSP._fetch_big_model_files). Its bundles
are tinygrad pkls for a GPU the Jetson does not have, but each one names the
comma commit it was compiled from, and that commit's ONNX in comma's LFS is
what the Jetson runs. The catalog says what exists, the slot says which one,
the pointer at the commit says which bytes. A commit that ships a precompiled
pkl instead names its export, and jetlink's registry follows that to comma's
model repo; see jetlink.registry.lfs.
"""
from __future__ import annotations

import os
import re
import time
from pathlib import Path

from openpilot.sunnypilot.accelerators.jetlink import gadget
from openpilot.sunnypilot.accelerators.jetlink.gadget import set_logger as _set_gadget_logger

# What moved to gadget.py, still reachable as helpers.<name>. Forwarded rather
# than imported so there is one seam: gadget's own functions read these out of
# gadget's namespace, and a test that patches them there is seen here too.
_FORWARDED = frozenset((
  'AGNOS', 'CC_ORIENTATION', 'DORMANT', 'FFS_MOUNT', 'GADGET_PATH', 'GADGET_SETUP_TIMEOUT',
  'GADGET_STATUS', 'HOST_POLL', 'P_ENABLED', 'P_ENDPOINT', 'P_READY', 'SHUTDOWN_REQUEST',
  'STALLED_ENUMERATION', 'STALLED_STATES', 'UDC_PATH', 'bound_udc', 'can_setup_gadget',
  'dormant', 'enabled', 'finish_shutdown', 'gadget_error', 'host_attached',
  'link_configured', 'offroad',
  'link_endpoint', 'package_installed', 'params_dir', 'pending_shutdown', 'port_has_host',
  'repo_root', 'request_shutdown', 'set_dormant', 'setup_gadget', 'udc_state', 'wait_for_host',
))


def __getattr__(name: str):
  # deliberately not cached into this module's namespace: binding the value
  # would freeze whatever gadget held at first use, and a test that patches
  # gadget would stop being visible through here, which is the whole point of
  # the forward. A frozenset lookup and a getattr is a microsecond
  if name in _FORWARDED:
    return getattr(gadget, name)
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL_REF

# none of these are CLEAR_ON_MANAGER_START: readiness must survive a reboot or
# every ignition rebuilds a 160 s engine


# One handle per params store. Constructing a Params costs 144 us on the comma
# against 110 us for the read itself, so a fresh one per read more than doubles
# every param this module touches, and jetlinkd touches several twice a second
# for the whole time the car is parked. Keyed on the prefix because a test or a
# bench runs under its own store and must not be handed the device's.
# gadget.py logs through a plain logger so the owner needs no swaglog; every
# process that imports helpers is heavy already and wants its lines in the drive
_set_gadget_logger(cloudlog)

_params: dict[str, Params] = {}


def params() -> Params:
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  store = _params.get(prefix)
  if store is None:
    store = _params[prefix] = Params()
  return store


def _get(key: str, default=None):
  """Read a param, tolerating a params library that predates the key.

  Called from hardwared and the UI's param thread, so UnknownKeyName here
  would take down a process that has nothing to do with jetlink.
  """
  try:
    return params().get(key)
  except Exception:
    return default


def await_shutdown(timeout: float) -> bool:
  """Wait for jetlinkd to take the request. False if nobody did in time."""
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if not gadget.SHUTDOWN_REQUEST.exists():
      return True
    time.sleep(0.25)
  gadget.finish_shutdown()
  return False


# jetlinkd holds the gadget for as long as the link is enabled, so presence no
# longer blinks at every handover. What is left to bridge is a USB3 link
# recovery passing through "addressed", and the one deliberate re-enumeration
# this design still costs: see Jetlinkd.settle
PRESENCE_HOLD = 5.0
_last_configured = 0.0


def gadget_present() -> bool:
  """Is a Jetson actually on the other end right now?

  True once something holds the gadget open and a host has configured us,
  held for PRESENCE_HOLD after that stops.
  """
  global _last_configured
  if gadget.link_endpoint() is not None:
    return True
  if gadget.dormant():
    # no enumeration during suspend; the CC line still tells a sleeping host from an unplugged one
    return gadget.port_has_host()
  now = time.monotonic()
  if gadget.host_attached():
    _last_configured = now
    return True
  return now - _last_configured < PRESENCE_HOLD


def connect(deadline: float | None = None, name: str | None = None, loan=None):
  """Open the link. USB unless an endpoint override is set.

  `deadline` is per frame and defaults to FRAME_TIMEOUT: modeld blocks on a
  frame the way it blocks on a chestnut. `name` is what the server logs this
  connection as; two comma processes share one gadget and the Jetson's journal
  has no clock to tell them apart by. With a `loan`, jetlinkd owns the gadget
  and this end only opens the endpoint files: see lending.py.
  """
  from jetlink.client import FRAME_TIMEOUT, JetlinkClient
  deadline = FRAME_TIMEOUT if deadline is None else deadline
  endpoint = gadget.link_endpoint()
  if endpoint is not None:
    host, port = endpoint
    cloudlog.warning("jetlink: connecting over tcp to %s:%d", host, port)
    return JetlinkClient.open_tcp(host, port, deadline=deadline, name=name)
  if loan is not None:
    return JetlinkClient.open_borrowed_ffs(loan.mount, loan.udc, bounce=loan.bounce,
                                           deadline=deadline, name=name)
  # the comma is the gadget and the Jetson the host; see gadget_present()
  return JetlinkClient.open_ffs(str(gadget.FFS_MOUNT), gadget=str(gadget.GADGET_PATH), deadline=deadline, name=name)



def gadget_alert() -> str | None:
  """The gadget failure worth an alert: only for someone who asked for the link."""
  return gadget.gadget_error() if gadget.enabled() else None


# -- the model ------------------------------------------------------------

P_MODEL_LEGACY = "JetlinkModel"      # the accelerator's own pick, before the big-model slot was the one choice
P_POINTERS = "JetlinkModelPointers"  # ref -> {oid, size}; a commit's tree never changes
# the model manager's copy of sunnypilot's big-model catalog; the key is
# models.fetcher.ModelFetcher.MODEL_SOURCES['chestnut']'s
CATALOG_PARAM = "ModelManager_ModelsCache_Chestnut"

POINTER_TIMEOUT = 10.0
_REF = re.compile(r'[0-9a-f]{40}')
# the index and the slot are JSON params, and the UI names the active model
# every frame; the status line can lag a new pick by this long
INDEX_TTL = 2.0
_index_cache: tuple[float, list[dict]] | None = None
_slot_cache: tuple[float, str | None] | None = None


def catalog() -> list[dict]:
  """sunnypilot's big-model bundles as {name, ref}, newest first, as the model
  manager's own picker lists them.

  From the cached JSON rather than the parsed bundles: parsing builds capnp
  objects and writes chunk manifests, for two fields. Read from the UI's
  param thread, so nothing escapes.
  """
  try:
    from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
    bundles = (params().get(CATALOG_PARAM) or {}).get('bundles', [])
    found = [b for b in bundles if _REF.fullmatch(str(b.get('ref')))
             and int(b.get('minimum_selector_version', 0)) == REQUIRED_JSON_VERSION]
  except Exception:
    cloudlog.exception("jetlink: could not read the big-model catalog")
    return []
  found.sort(key=lambda b: int(b.get('index', 0)), reverse=True)
  return [{'name': str(b.get('display_name') or b['ref'][:10]), 'ref': b['ref']} for b in found]


def pointers() -> dict[str, dict]:
  value = _get(P_POINTERS)
  return value if isinstance(value, dict) else {}


def fetch_pointer(ref: str) -> tuple[str, int]:
  """The oid and size of the ONNX a comma commit names, in its tree or, for a
  precompiled-pkl commit, in comma's model repo. The Jetson's registry does
  the same lookup, so the two ends agree on every model's identity."""
  from jetlink.registry.lfs import fetch_pointer as registry_fetch_pointer
  pointer = registry_fetch_pointer(ref, timeout=POINTER_TIMEOUT)
  return pointer.oid, pointer.size


def resolve_pointer(ref: str) -> tuple[str, int]:
  """The oid and size behind a catalog model, fetched the first time and kept for good."""
  global _index_cache
  known = pointers()
  if ref in known:
    return known[ref]['oid'], int(known[ref]['size'])
  oid, size = fetch_pointer(ref)
  known[ref] = {'oid': oid, 'size': size}
  # blocking: the next lookup reads this back, and a put still in flight would be lost under it
  params().put(P_POINTERS, known, block=True)
  _index_cache = None
  cloudlog.warning("jetlink: %s is %s, %d MB", ref[:10], oid[:16], size >> 20)
  return oid, size


def model_index() -> list[dict]:
  """Every catalog model, {name, ref, oid, size}. oid and size are None until
  the model has been selected and resolved. No network."""
  global _index_cache
  now = time.monotonic()
  if _index_cache is not None and now - _index_cache[0] < INDEX_TTL:
    return _index_cache[1]
  known = pointers()
  out = []
  for b in catalog():
    p = known.get(b['ref']) or {}
    out.append({**b, 'oid': p.get('oid'), 'size': int(p['size']) if p.get('size') else None})
  _index_cache = (now, out)
  return out


def selected_ref() -> str | None:
  """The big-model slot's pick. The raw dict, not a parsed bundle, and memoised:
  the slot is the model manager's to validate, and the UI asks every frame."""
  global _slot_cache
  now = time.monotonic()
  if _slot_cache is not None and now - _slot_cache[0] < INDEX_TTL:
    return _slot_cache[1]
  from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS
  slot = _get(ACTIVE_BUNDLE_KEYS["chestnut"])
  ref = slot.get('ref') if isinstance(slot, dict) else None
  _slot_cache = (now, ref if isinstance(ref, str) and ref else None)
  return _slot_cache[1]


def selected_model() -> dict | None:
  """The big model the device picked, or the fork's default big model, or the newest.

  The pick is the model manager's big-model slot, a choice for whichever
  hardware runs it. A ref the catalog dropped falls back the same way: leaving
  the device with no model at all would be worse than quietly using the default.
  """
  models = model_index()
  if not models:
    return None
  wanted = selected_ref()
  if wanted:
    for m in models:
      if m['ref'] == wanted:
        return m
    cloudlog.warning("jetlink: no catalog model for %r, using the default", wanted)
  return next((m for m in models if m['ref'] == DEFAULT_BIG_MODEL_REF), models[0])


def migrate_selection() -> None:
  """JetlinkModel was the accelerator's own pick; the big-model slot is the one
  choice now. Move it there, once, at jetlinkd start.

  A ref is written as the slot the model manager would write, minus the files,
  which it fetches itself if a chestnut is fitted. A name from before the
  catalog maps to the ref of the engine that is ready, a pointer fetch per
  catalog model until the match. Without the network, or the catalog, it is
  left for the next run. A slot already picked wins.
  """
  wanted = _get(P_MODEL_LEGACY)
  if not wanted:
    return
  store = params()
  if selected_ref() is None:
    ref = wanted if _REF.fullmatch(wanted) else None
    if ref is None and (ready := _get(gadget.P_READY)):
      for m in catalog():
        try:
          oid, _ = resolve_pointer(m['ref'])
        except Exception as e:
          cloudlog.warning("jetlink: cannot migrate the selection %r yet: %s", wanted, e)
          return
        if oid == ready:
          ref = m['ref']
          break
    if ref is not None:
      try:
        stored = _store_slot(store, ref)
      except LookupError as e:
        cloudlog.warning("jetlink: cannot migrate the selection %r yet: %s", wanted, e)
        return
      if stored:
        cloudlog.warning("jetlink: selection %r is now the big-model slot, %s", wanted, ref[:10])
      else:
        cloudlog.warning("jetlink: selection %r is not a catalog model, using the default", wanted)
  store.remove(P_MODEL_LEGACY)


def _store_slot(params, ref: str) -> bool:
  """Write the big-model slot as the model manager does for a bundle it has
  downloaded. False if the catalog does not list the ref; LookupError if
  there is no catalog to ask yet."""
  global _slot_cache
  from openpilot.sunnypilot.models.fetcher import get_cached_bundles
  from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, resolve_bundle_by_ref
  bundles = get_cached_bundles(params, "chestnut")
  if not bundles:
    raise LookupError("no big-model catalog cached")
  resolved = resolve_bundle_by_ref(ref, {"chestnut": bundles})
  if resolved is None:
    return False
  params.put(ACTIVE_BUNDLE_KEYS["chestnut"], resolved[0].to_dict(), block=True)
  _slot_cache = None
  return True


def model_dir() -> Path:
  """Ours, under the model manager's root: its cache clear removes every file
  it does not recognise and leaves directories alone."""
  return Path(Paths.model_root()) / 'jetlink'


def model_file_name(model: dict) -> str:
  """One file per model, so switching back does not re-download."""
  return f"{model['oid'][:16]}.onnx"


def shipped_model_path() -> Path | None:
  """The chosen large model, if it has been fetched.

  Keyed on the oid, not the in-tree pointer, which moves with upstream syncs.
  Size is the cheap check that the file is the one we mean.
  """
  model = selected_model()
  if model is None or not model['oid']:
    return None
  path = model_dir() / model_file_name(model)
  if path.is_file() and path.stat().st_size == model['size']:
    return path
  return None


def fetch_shipped_model(progress=None, should_stop=None) -> Path | None:
  """Download the chosen large model if it is not here yet. None when nothing is chosen."""
  from openpilot.sunnypilot.accelerators.jetlink import lfs
  model = selected_model()
  if model is None or not model['oid']:
    return None
  dest = model_dir() / model_file_name(model)
  return lfs.fetch_oid(model['oid'], model['size'], dest, gadget.repo_root(),
                       progress=progress, should_stop=should_stop)


# -- readiness ------------------------------------------------------------

def engine_ready_for(sha256: str | None) -> bool:
  if not sha256:
    return False
  return (_get(gadget.P_READY) or '') == sha256


def set_engine_ready(sha256: str | None) -> None:
  store = params()
  if sha256:
    store.put(gadget.P_READY, sha256)
  else:
    store.remove(gadget.P_READY)
