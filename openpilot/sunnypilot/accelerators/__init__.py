"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Which accelerator runs the large model, if any.

Backends are discovered, not hardcoded, and every call here is guarded: a
backend that is missing or broken costs the large model, never the calling
process. modeld, manager, hardwared and the UI all come through this module,
so adding one is a directory plus a line in _BACKENDS.
"""
from __future__ import annotations

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.accelerators.base import Accelerator, Daemon

# Priority order: the first backend that is ready runs the large model. Local
# hardware first, so a chestnut with a Jetson also attached still uses the board.
_BACKENDS = (
  "openpilot.sunnypilot.accelerators.chestnut:ChestnutAccelerator",
  "openpilot.sunnypilot.accelerators.jetlink.backend:JetlinkAccelerator",
)

# Written by whichever backend is provisioning, read by the UI. A param rather
# than a field because the writer is an offroad daemon in another process.
P_PROGRESS = "AcceleratorProgress"

_cache: list[Accelerator] | None = None


def backends() -> list[Accelerator]:
  """Every backend this install carries. Cached: the set cannot change at runtime."""
  global _cache
  if _cache is None:
    found = []
    for spec in _BACKENDS:
      module, _, cls = spec.partition(':')
      try:
        found.append(getattr(__import__(module, fromlist=[cls]), cls)())
      except ImportError:
        pass  # a backend this fork does not ship, or whose package is absent
      except Exception:
        cloudlog.exception("accelerators: %s failed to load", spec)
    _cache = found
  return _cache


def _ask(backend: Accelerator, question: str, default=None):
  try:
    return getattr(backend, question)()
  except Exception:
    cloudlog.exception("accelerators: %s.%s failed", backend.name, question)
    return default


def present() -> bool:
  """Is any accelerator attached? Answers deviceState.chestnutPresent."""
  return any(_ask(b, 'present', False) for b in backends())


def active() -> Accelerator | None:
  """The backend that should run the large model now, or None for the small one."""
  return next((b for b in backends() if _ask(b, 'ready', False)), None)


def ready() -> bool:
  """Can something run the large model right now? What the UI calls 'compiled'."""
  return active() is not None


def model_choices() -> list[dict]:
  """Optional backend-owned models, separate from model-manager bundles."""
  return [dict(choice, backend=b.name) for b in backends()
          if hasattr(b, 'model_choices') for choice in _ask(b, 'model_choices', [])]


def select_model(backend: str, name: str) -> None:
  for b in backends():
    if b.name == backend and hasattr(b, 'select_model'):
      b.select_model(name)
      return
  raise ValueError(f'unknown accelerator: {backend}')


def active_model_name() -> str | None:
  backend = active()
  if backend is None or not hasattr(backend, 'model_choices'):
    return None
  return next((m['name'] for m in _ask(backend, 'model_choices', []) if m['selected']), None)


def catalog() -> str | None:
  """The model-manager catalog the attached accelerator draws from, if any.

  Only a present backend gets a say, and the first one wins like everywhere
  else here. None means the manager stays on the small-model catalog.
  """
  return next((c for b in backends() if _ask(b, 'present', False) and (c := _ask(b, 'catalog'))), None)


def unavailable_reason() -> str | None:
  """The first backend complaint worth showing offroad, if any."""
  return next((r for b in backends() if (r := _ask(b, 'unavailable_reason'))), None)


def daemons() -> list[Daemon]:
  """Offroad processes the backends need, for process_config to build."""
  return [d for b in backends() if (d := _ask(b, 'daemon')) is not None]


def progress() -> dict | None:
  """{stage, frac, msg} while a backend provisions, else None.

  Read from the UI's param thread, so nothing may escape - including
  UnknownKeyName on a build whose params library predates this key.
  """
  try:
    value = Params().get(P_PROGRESS)
  except Exception:
    return None
  return value if isinstance(value, dict) else None


def report_progress(stage: str, frac: float, msg: str = '') -> None:
  """For backends. Never raises: called from except handlers."""
  try:
    Params().put(P_PROGRESS, {'stage': stage, 'frac': round(frac, 4), 'msg': msg})
  except Exception:
    cloudlog.exception("accelerators: could not report progress")


def clear_progress() -> None:
  try:
    Params().remove(P_PROGRESS)
  except Exception:
    cloudlog.exception("accelerators: could not clear progress")


def shutdown(reason: str = '') -> None:
  """The device is about to power off. Every backend that cares gets told.

  Optional on the protocol: comma's board dies with the device and has no
  say. Nothing may escape, this runs on hardwared's way out.
  """
  for b in backends():
    fn = getattr(b, 'shutdown', None)
    if fn is None:
      continue
    try:
      fn(reason)
    except Exception:
      cloudlog.exception("accelerators: %s.shutdown failed", b.name)
