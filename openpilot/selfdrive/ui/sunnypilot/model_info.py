"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import contextlib
import os
import time

from openpilot.common.hardware.hw import Paths
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.sunnypilot import accelerators
from openpilot.sunnypilot.models.fetcher import get_cached_bundles
from openpilot.sunnypilot.models.helpers import get_active_source, get_selected_bundle, resolve_bundle_by_ref
from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL, DEFAULT_MODEL


def model_cache_size_mb() -> float:
  """Bytes on disk under the model cache directory, in MB."""
  model_root = Paths.model_root()
  total = 0
  if os.path.isdir(model_root):
    for name in os.listdir(model_root):
      with contextlib.suppress(OSError):
        total += os.path.getsize(os.path.join(model_root, name))
  return total / (1024 ** 2)


def active_source() -> str:
  return get_active_source(chestnut=ui_state.chestnut_present,
                           chestnut_active=ui_state.chestnut_active, chestnut_loading=ui_state.chestnut_loading,
                           offroad=ui_state.is_offroad())


def bundles_for_source(source: str):
  if source == active_source():
    return ui_state.sm["modelManagerSP"].availableBundles
  return get_cached_bundles(ui_state.params, source)


def default_model(source: str) -> str:
  return DEFAULT_BIG_MODEL if source == 'chestnut' else DEFAULT_MODEL


def default_model_name(source: str) -> str:
  return f"{default_model(source)} (Default)"


def big_model_state() -> str | None:
  """'failed' | 'loading' | 'ready' | None, from the same state the icons render."""
  return {ChestnutState.UNCOMPILED: 'failed',
          ChestnutState.FAILED: 'failed',
          ChestnutState.LOADING: 'loading',
          ChestnutState.WAITING: 'ready'}.get(ui_state.chestnut_state)


def big_model_progress() -> tuple[str, float, str] | None:
  """(stage, 0..1, message) while an accelerator is working, else None. The message
  is carried because a stage like "waiting for the jetson" has no meaningful fraction"""
  progress = getattr(ui_state, 'accelerator_progress', None)
  if not progress:
    return None
  stage = str(progress.get('stage', ''))
  if stage in ('', 'ready'):
    return None
  return stage, float(progress.get('frac', 0.0)), str(progress.get('msg', ''))


def carrying_model() -> tuple[str | None, str | None, str | None]:
  """(source, internal name, display name) of what actually drives. Runner-matched:
  when a Default big cannot carry, stock modeld runs the Default small, never the
  small slot's pick; a custom big has no automatic fallback yet -> (None, None, None)."""
  # only when no board is fitted does the chestnut state describe the jetlink view
  if not ui_state.chestnut_present and ui_state.chestnut_state == ChestnutState.ACTIVE:
    name = accelerators.active_model_name()
    if name is not None:
      return 'accelerator', name, name
  source = active_source()
  if source == "chestnut":
    bundle = get_selected_bundle(ui_state.params, "chestnut")
    if bundle:
      return "chestnut", bundle.internalName, bundle.displayName
    name = default_model_name("chestnut")
    return "chestnut", name, name
  if ui_state.chestnut_present:
    if get_selected_bundle(ui_state.params, "chestnut") is None:
      name = default_model_name("qcom")
      return "qcom", name, name
    return None, None, None
  bundle = get_selected_bundle(ui_state.params, "qcom")
  if bundle:
    return "qcom", bundle.internalName, bundle.displayName
  name = default_model_name("qcom")
  return "qcom", name, name


def queued_name(current_ref) -> str | None:
  ref = ui_state.params.get("ModelManager_DownloadRef")
  if ref and ref != current_ref:
    source_bundles = {source: bundles_for_source(source) for source in ("qcom", "chestnut")}
    if resolved := resolve_bundle_by_ref(ref, source_bundles):
      return resolved[0].internalName
  return None


def slot_bundle(source: str):
  return get_selected_bundle(ui_state.params, source)


def model_info() -> tuple[str, str, str]:
  """returns (active source, active model name, other model name)

  Names come from the params slots, never modelManagerSP.activeBundle — the
  manager republishes a tick after a chestnut change, so the stale bundle
  would flash the wrong model."""
  source = active_source()
  other = "qcom" if source == "chestnut" else "chestnut"
  active_bundle = slot_bundle(source)
  other_bundle = slot_bundle(other)

  active_name = active_bundle.displayName if active_bundle else default_model_name(source)
  other_name = other_bundle.displayName if other_bundle else default_model_name(other)
  return source, active_name, other_name


# mirrors the manager's ModelCache keys; the manager restamps them on a successful fetch
MODEL_SYNC_KEYS = ("ModelManager_LastSyncTime", "ModelManager_LastSyncTime_Chestnut")
MODEL_SYNC_TIMEOUT = 20.0


def refresh_model_list() -> None:
  # zeroing the sync keys makes the manager refetch each manifest on its next tick
  for key in MODEL_SYNC_KEYS:
    ui_state.params.put(key, 0)


def refresh_in_progress(started_at: float | None) -> bool:
  """Whether a user refresh is still outstanding. A failed fetch never restamps the
  sync keys, so the spinner is bounded by MODEL_SYNC_TIMEOUT rather than sticking."""
  if started_at is None or time.monotonic() - started_at > MODEL_SYNC_TIMEOUT:
    return False
  return not all(ui_state.params.get(key) for key in MODEL_SYNC_KEYS)
