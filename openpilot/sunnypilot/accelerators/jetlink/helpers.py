"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Where the model is, whether the Jetson is attached, and how far along it is.

Deliberately reuses sunnypilot's existing plumbing rather than adding a
parallel one: the model itself is whatever the model manager has already
downloaded into the "chestnut" slot, so selecting a large model in the UI and
watching it download works exactly as it does with a real chestnut.
"""
from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path

from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# Params. All CLEAR_ON_MANAGER_START-free: readiness must survive a reboot,
# otherwise every ignition cycle would rebuild a 160 s engine.
P_ENABLED = "JetlinkEnabled"        # user toggle; absent means "auto"
P_READY = "JetlinkEngineReady"      # sha256 of the model the Jetson has built
P_ENDPOINT = "JetlinkEndpoint"      # optional "host:port" to use TCP instead of USB

UNCHUNKED_SUFFIX = ".jetlink-unchunked"


def _get(key: str, default=None):
  """Read a param, tolerating one this build does not know about.

  These keys are declared in params_keys.h, which is compiled into the params
  library. A device running an older build of that library - a partial update,
  or a prebuilt branch - raises UnknownKeyName. Several of these predicates are
  called from hardwared and from the UI's param thread, so a raise here takes
  down a process that has nothing to do with jetlink. One guard, at the read.
  """
  try:
    return Params().get(key)
  except Exception:
    return default


def link_endpoint() -> tuple[str, int] | None:
  """A host:port override, for running the Jetson over ethernet during bring-up."""
  raw = _get(P_ENDPOINT)
  if not raw:
    return None
  host, _, port = raw.strip().partition(':')
  return host, int(port or 5599)


# The comma is the USB *gadget* and the Jetson is the host. That is decided by
# what the two kernels have, not by which side is "the client": AGNOS has
# CONFIG_USB_F_FS and libcomposite built in, while a Jetson host needs no driver
# at all (libusb goes through usbfs) - which matters because L4T rootfs images
# are often stripped of the gadget modules. See jetlink/docs/transport.md.
GADGET_PATH = Path("/sys/kernel/config/usb_gadget/jetlink")
FFS_MOUNT = Path("/dev/ffs-jetlink")
UDC_PATH = Path("/sys/class/udc")
# Written by scripts/setup_gadget.sh at boot: "ok", or "error: <reason>".
# tmpfs, so it describes this boot and costs the flash nothing.
GADGET_STATUS = Path("/dev/shm/jetlink-gadget")
CC_ORIENTATION = Path('/sys/class/power_supply/usb/typec_cc_orientation')


def gadget_error() -> str | None:
  """Why the USB gadget is unavailable, if it is.

  The gadget is set up once at boot, by root, from launch_chffrplus.sh - long
  before any of this runs and nowhere a user would look. Without this the whole
  feature just silently does not appear on a device whose kernel lacks the
  gadget drivers, or where the package was never installed.

  A missing file is not an error: it means a build that never ran the setup at
  all, which is the same as jetlink not being installed here.
  """
  try:
    reason = GADGET_STATUS.read_text().strip()
  except OSError:
    return None
  if not reason or reason == 'ok':
    return None
  return reason.removeprefix('error:').strip() or None


def gadget_bound() -> bool:
  """Has our gadget been attached to a device controller?"""
  try:
    return bool((GADGET_PATH / "UDC").read_text().strip())
  except OSError:
    return False


def host_attached() -> bool:
  """Has a host (the Jetson) enumerated and configured us?"""
  try:
    udc = (GADGET_PATH / "UDC").read_text().strip()
  except OSError:
    return False
  if not udc:
    return False
  try:
    return (UDC_PATH / udc / "state").read_text().strip() == "configured"
  except OSError:
    return False


def link_configured() -> bool:
  """Can we even attempt a link? The gadget exists, or TCP is configured.

  Deliberately NOT host_attached(): the comma's UDC only binds when something
  opens ep0, and nothing opens ep0 unless the link looks usable. Gating the
  attempt on a host already being there deadlocks - the Jetson can never
  enumerate because nobody ever presented the gadget to it.
  """
  if gadget_error() is not None:
    return False
  if link_endpoint() is not None:
    return True
  try:
    return (FFS_MOUNT / "ep0").exists()
  except OSError:
    # A root-only mount raises PermissionError from stat rather than returning
    # False. We could not open it either way, so treat it as unusable.
    return False


# jetlinkd writes its pid here when it has released the gadget on purpose so
# the Jetson can sleep (Jetlinkd.go_dormant). The Jetson is still there, only
# unreachable until something presents the gadget again, so presence has to
# come from this rather than from the UDC. A marker whose writer is dead is a
# leftover from a kill, not a state, which is what the pid is for.
DORMANT = Path("/dev/shm/jetlink-dormant")
# hardwared's way of asking jetlinkd to power the Jetson off; see
# JetlinkAccelerator.shutdown. jetlinkd unlinks it when it has dealt with it.
SHUTDOWN_REQUEST = Path("/dev/shm/jetlink-shutdown")


def set_dormant(on: bool) -> None:
  try:
    if on:
      DORMANT.write_text(str(os.getpid()))
    else:
      DORMANT.unlink(missing_ok=True)
  except OSError:
    cloudlog.exception("jetlink: could not update the dormant marker")


def dormant() -> bool:
  """Has a live jetlinkd released the gadget on purpose?"""
  try:
    pid = int(DORMANT.read_text())
  except (OSError, ValueError):
    return False
  try:
    os.kill(pid, 0)
  except ProcessLookupError:
    return False
  except PermissionError:
    pass  # alive, just not ours to signal
  return True


def request_shutdown(reason: str) -> bool:
  try:
    SHUTDOWN_REQUEST.write_text(json.dumps({'reason': reason}))
    return True
  except OSError:
    cloudlog.exception("jetlink: could not write the shutdown request")
    return False


def pending_shutdown() -> str | None:
  """The reason in a shutdown request that has not been dealt with, if any."""
  try:
    return str(json.loads(SHUTDOWN_REQUEST.read_text()).get('reason', ''))
  except (OSError, ValueError):
    return None


def finish_shutdown() -> None:
  try:
    SHUTDOWN_REQUEST.unlink(missing_ok=True)
  except OSError:
    cloudlog.exception("jetlink: could not remove the shutdown request")


def await_shutdown(timeout: float) -> bool:
  """Wait for jetlinkd to take the request. False if nobody did in time."""
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if not SHUTDOWN_REQUEST.exists():
      return True
    time.sleep(0.25)
  finish_shutdown()
  return False


# How long chestnutPresent stays true after the UDC last read "configured".
# selfdrived soft-disables on chestnutPresent dropping while the big model is
# active, and a USB3 link recovery the client rides out passes through
# "addressed" for a moment. Chestnut's own check is an enumerated USB id, which
# is stickier than a UDC state read at 2 Hz; this makes ours comparable.
PRESENCE_HOLD = 5.0
_last_configured = 0.0


def gadget_present() -> bool:
  """Is a Jetson actually on the other end right now?

  This is the one that answers "is an accelerator attached", so it is what
  deviceState.chestnutPresent uses. It only becomes true once something is
  holding the gadget open and a host has configured us, and it holds for
  PRESENCE_HOLD after that stops being true.
  """
  global _last_configured
  if link_endpoint() is not None:
    return True
  if dormant():
    # Enumeration is deliberately absent during suspend; physical cable
    # detection still distinguishes a sleeping host from an unplugged one.
    try:
      return int(CC_ORIENTATION.read_text()) != 0
    except (OSError, ValueError):
      return False
  now = time.monotonic()
  if host_attached():
    _last_configured = now
    return True
  return now - _last_configured < PRESENCE_HOLD


def connect(deadline: float | None = None):
  """Open the link. USB unless an endpoint override is set (bring-up over ethernet).

  `deadline` is per frame and defaults to the client's FRAME_TIMEOUT: modeld
  blocks on a frame the way it blocks on a chestnut, and only a stall that long
  means the Jetson is gone.
  """
  from jetlink.client import FRAME_TIMEOUT, JetlinkClient
  deadline = FRAME_TIMEOUT if deadline is None else deadline
  endpoint = link_endpoint()
  if endpoint is not None:
    host, port = endpoint
    cloudlog.warning("jetlink: connecting over tcp to %s:%d", host, port)
    return JetlinkClient.open_tcp(host, port, deadline=deadline)
  # Over USB the comma is the gadget and the Jetson is the host; see
  # gadget_present() for why round that way.
  return JetlinkClient.open_ffs(str(FFS_MOUNT), gadget=str(GADGET_PATH), deadline=deadline)


def enabled() -> bool:
  """Should we run the link at all? Absent param means "auto"."""
  v = _get(P_ENABLED)
  return link_configured() if v is None else bool(v)


def opted_in() -> bool:
  """Has the user actually asked for the link, rather than left it on auto?

  The distinction only matters for complaining. On auto, a device that cannot
  present the gadget should simply not offer the feature; once someone has
  turned it on, silence is the wrong answer.
  """
  return bool(_get(P_ENABLED))


def gadget_alert() -> str | None:
  """The gadget failure worth putting in front of the user, if any."""
  return gadget_error() if opted_in() else None


# -- the model ------------------------------------------------------------

def active_bundle():
  from openpilot.sunnypilot.models.helpers import get_selected_bundle
  return get_selected_bundle(Params(), "chestnut")


def _artifact_names(bundle) -> list[str]:
  out = []
  for model in getattr(bundle, 'models', []) or []:
    artifact = getattr(model, 'artifact', None)
    name = getattr(artifact, 'fileName', None) if artifact else None
    if name and name.endswith('.onnx'):
      out.append(name)
  return out


def active_model_path() -> Path | None:
  """Path to the selected large model's ONNX, materialising chunks if needed.

  Returns None when there is no large model here yet; the caller then simply
  stays on the small model.

  No bundle selected is the normal case, not a dead end: every bundle the
  model manager offers is a tinygrad pkl compiled for chestnut's GPU, and none
  of them ships an ONNX, so what a Jetson actually runs is the model openpilot
  itself pins. Falling through to it is the whole point.
  """
  bundle = active_bundle()
  root = Path(Paths.model_root())
  for name in _artifact_names(bundle) if bundle is not None else []:
    plain = root / name
    if plain.is_file() and plain.stat().st_size > 1_000_000:
      return plain
    # Chunked download: reassemble once, next to the chunks.
    manifest = root / f'{name}.chunkmanifest'
    if manifest.is_file():
      return _materialise(root / name)
  return shipped_model_path()


BIG_MODEL_NAME = 'big_driving_supercombo.onnx'
MODEL_INDEX = Path(__file__).with_name('models.json')
P_MODEL = "JetlinkModel"          # name of the chosen entry in models.json


def repo_root() -> Path:
  return Path(__file__).resolve().parents[4]


def big_model_pointer() -> Path:
  from openpilot.selfdrive.modeld.helpers import MODELS_DIR
  return MODELS_DIR / BIG_MODEL_NAME


def model_index() -> list[dict]:
  """The large models we know a Jetson can run.

  Hand-maintained from comma's history rather than discovered, because there is
  nothing to discover from: openpilot overwrites one file, so the older models
  exist only as git-lfs objects that no manifest lists. See models.json.
  """
  try:
    with open(MODEL_INDEX) as f:
      return json.load(f).get('models', [])
  except Exception:
    cloudlog.exception("jetlink: could not read the model index")
    return []


def selected_model() -> dict | None:
  """The entry the user picked, or the default.

  An unknown name falls back rather than leaving the device with no model at
  all: the index can shrink under a param that outlived it.
  """
  models = model_index()
  if not models:
    return None
  wanted = _get(P_MODEL)
  if wanted:
    for m in models:
      if m.get('name') == wanted:
        return m
    cloudlog.warning("jetlink: no model called %r in the index, using the default", wanted)
  return next((m for m in models if m.get('default')), models[0])


def shipped_model_path() -> Path | None:
  """The chosen large model, if it has been fetched.

  Keyed on the index entry rather than on whatever the worktree pins: the
  in-tree pointer moves with upstream syncs and is also what a chestnut device
  compiles, so tying the Jetson's model to it would couple two unrelated
  decisions. Size is the cheap check that the file on disk is the one we mean.
  """
  model = selected_model()
  if model is None:
    return None
  path = Path(Paths.model_root()) / model_file_name(model)
  if path.is_file() and path.stat().st_size == model['size']:
    return path
  return None


def _materialise(path: Path) -> Path | None:
  from openpilot.common.file_chunker import open_file_chunked
  out = path.with_name(path.name + UNCHUNKED_SUFFIX)
  if out.is_file() and out.stat().st_size > 1_000_000:
    return out
  free = shutil.disk_usage(path.parent).free
  try:
    with open_file_chunked(str(path)) as src, open(out, 'wb') as dst:
      shutil.copyfileobj(src, dst, length=4 << 20)
  except Exception:
    cloudlog.exception("jetlink: could not reassemble %s (%d MB free)", path.name, free >> 20)
    out.unlink(missing_ok=True)
    return None
  return out


def cleanup_unchunked(keep: Path | None = None) -> None:
  root = Path(Paths.model_root())
  for p in root.glob(f'*{UNCHUNKED_SUFFIX}'):
    if keep is None or p != keep:
      p.unlink(missing_ok=True)


def model_file_name(model: dict) -> str:
  """One file per model, so switching back does not re-download."""
  return f"{model['oid'][:16]}.onnx"


def fetch_shipped_model(progress=None, should_stop=None) -> Path | None:
  """Download the chosen large model if it is not here yet.

  Costs 0.2-1.8 GB once per model, on a device that has an accelerator
  attached, rather than on every install. Returns None when nothing is chosen.
  """
  from openpilot.sunnypilot.accelerators.jetlink import lfs
  model = selected_model()
  if model is None:
    return None
  dest = Path(Paths.model_root()) / model_file_name(model)
  return lfs.fetch_oid(model['oid'], model['size'], dest, repo_root(),
                       progress=progress, should_stop=should_stop)


# -- readiness ------------------------------------------------------------

def engine_ready_for(sha256: str | None) -> bool:
  if not sha256:
    return False
  return (_get(P_READY) or '') == sha256


def set_engine_ready(sha256: str | None) -> None:
  params = Params()
  if sha256:
    params.put(P_READY, sha256)
  else:
    params.remove(P_READY)
