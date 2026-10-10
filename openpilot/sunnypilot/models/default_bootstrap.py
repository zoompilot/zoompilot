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

``carry_over_picks`` keeps each device's pick across a selector bump (``REQUIRED_JSON_VERSION``),
which would otherwise drop every device back to the stock model.
"""

from openpilot.common.swaglog import cloudlog
from openpilot.cereal import custom
from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, REQUIRED_JSON_VERSION, get_selected_bundle

# Firehose Model in sunnypilot's driving_models manifest. Matched by short name rather than
# index: the manifest is renumbered on every regeneration (FM was 30 in v17, 28 in v22).
DEFAULT_MODEL_SHORT_NAME = "FM"
CARRY_OVER_KEY = "ModelManager_CarryOver"
_carry_over_queued: set[str] = set()


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


def carry_over_picks(params, source_bundles: dict[str, list["custom.ModelManagerSP.ModelBundle"]]) -> None:
  """Run before validate_active_bundles. A pick stored under an older selector version no longer
  parses, so validation clears its slot. Remember it first, and once the new catalog lists the same
  model (by ref, else by short name) queue that download, which fills the slot again. Kept until the
  slot is filled, so an offline first boot or a download cleared by going onroad tries again; queued
  once per model per process, so a download that keeps failing does not spin."""
  pending = params.get(CARRY_OVER_KEY) or {}
  changed = False
  for source, key in ACTIVE_BUNDLE_KEYS.items():
    raw = params.get(key)
    if isinstance(raw, dict) and raw and raw.get("minimumSelectorVersion") != REQUIRED_JSON_VERSION:
      pending[source] = {"ref": raw.get("ref", ""), "internalName": raw.get("internalName", "")}
      changed = True

  for source, pick in list(pending.items()):
    bundles = source_bundles.get(source) or []
    if get_selected_bundle(params, source) is not None:  # downloaded, or the user picked another
      del pending[source]
      changed = True
      continue
    if not bundles:  # offline, or the catalog is not fetched yet
      continue
    match = next((b for b in bundles if pick["ref"] and b.ref == pick["ref"]), None) or \
      max((b for b in bundles if b.ref and b.internalName == pick["internalName"]), key=lambda b: b.index, default=None)
    if match is None:
      cloudlog.warning(f"Model pick {pick} is not in the {source} catalog, leaving the default")
      del pending[source]
      changed = True
    elif match.ref not in _carry_over_queued and params.get("ModelManager_DownloadRef") is None:
      _carry_over_queued.add(match.ref)
      params.put("ModelManager_DownloadRef", match.ref)
      cloudlog.warning(f"Carrying the {source} pick {match.displayName} over the selector bump (queued ref {match.ref})")

  if changed:
    if pending:
      params.put(CARRY_OVER_KEY, pending)
    else:
      params.remove(CARRY_OVER_KEY)
