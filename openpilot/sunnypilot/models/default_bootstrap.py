"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ship the Firehose Model as the default driving model.

The model binary is not committed to the release. Instead the bundle is queued for download the
first time the manifest lists it, through the same hash-validated path the model selector uses.
This runs from the model manager main loop, which refreshes the available bundles and processes
``ModelManager_DownloadRef`` every second, so an offline boot is a no-op that retries next loop.

Applied once, guarded by ``DefaultModelApplied``: a device that already has a model selected keeps
it, and a user who later reverts to the stock model is never overridden again.
"""

from openpilot.common.swaglog import cloudlog
from openpilot.cereal import custom
from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS

# Firehose Model in sunnypilot's driving_models manifest. Matched by short name rather than
# index: the manifest is renumbered on every regeneration (FM was 30 in v17, 28 in v22).
DEFAULT_MODEL_SHORT_NAME = "FM"


def find_default_bundle(available_bundles: list["custom.ModelManagerSP.ModelBundle"]) -> "custom.ModelManagerSP.ModelBundle | None":
  matches = [b for b in available_bundles if b.internalName == DEFAULT_MODEL_SHORT_NAME and b.ref]
  return max(matches, key=lambda b: b.index) if matches else None


def maybe_apply_default_model(params, available_bundles: list["custom.ModelManagerSP.ModelBundle"]) -> None:
  if params.get_bool("DefaultModelApplied"):
    return

  # a model is already selected (migrated in, or picked by the user): keep it and never touch the default again
  if params.get(ACTIVE_BUNDLE_KEYS["qcom"]) is not None:
    params.put_bool("DefaultModelApplied", True)
    return

  # a download the user already queued wins; try again next loop
  if params.get("ModelManager_DownloadRef") is not None:
    return

  # offline or manifest not fetched yet: retry next loop
  bundle = find_default_bundle(available_bundles)
  if bundle is None:
    return

  params.put("ModelManager_DownloadRef", bundle.ref)
  params.put_bool("DefaultModelApplied", True)
  cloudlog.warning(f"Applying {bundle.displayName} as the default model (queued ref {bundle.ref})")
