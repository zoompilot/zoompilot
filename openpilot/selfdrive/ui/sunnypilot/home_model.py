"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The home screen's model name: the model list's short name (MDMV3) of whatever
carries, through big/small fall over and handback.
"""
import functools
import re

from openpilot.selfdrive.ui.sunnypilot.model_info import MODEL_SYNC_KEYS, carrying_model, default_model, default_model_name
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.models.fetcher import get_cached_bundles
from openpilot.sunnypilot.models.helpers import get_selected_bundle
from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL_REF, DEFAULT_MODEL_REF

# a manifest display name's release date, "(September 17, 2026)"
RELEASE_DATE = re.compile(r"\s*\([^()]*\)$")
DEFAULT_REFS = {"qcom": DEFAULT_MODEL_REF, "chestnut": DEFAULT_BIG_MODEL_REF}


@functools.lru_cache(maxsize=1)
def _list_short_names(params, _synced: tuple) -> dict[str, str]:
  """Short name by ref and by undated display name, over both cached lists; rebuilt
  only when the manager refetches one."""
  names: dict[str, str] = {}
  for source in ("qcom", "chestnut"):
    for bundle in get_cached_bundles(params, source):
      names[bundle.ref] = bundle.internalName
      names.setdefault(RELEASE_DATE.sub("", bundle.displayName), bundle.internalName)
  return names


def carrying_short_name() -> str:
  source, internal, display = carrying_model()
  if source is None:
    # a custom big with no fallback: its pick, which drives once it is ready
    return get_selected_bundle(ui_state.params, "chestnut").internalName
  names = _list_short_names(ui_state.params, tuple(ui_state.params.get(key) for key in MODEL_SYNC_KEYS))
  if source == "accelerator":
    # jetlink names a model by its list display name, dated or not
    undated = RELEASE_DATE.sub("", display)
    return names.get(undated, undated)
  if internal == default_model_name(source):
    # an empty slot's default is named for the tree, not for its list entry
    return names.get(DEFAULT_REFS[source], default_model(source))
  return internal
