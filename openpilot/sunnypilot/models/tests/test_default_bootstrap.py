"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.sunnypilot.models.default_bootstrap import (DEFAULT_MODEL_SHORT_NAME, find_default_bundle,
                                                            maybe_apply_default_model)

FM_REF = "b74f5189a74446015c0cf78a4a9f0134a347ae3b"


class FakeBundle:
  def __init__(self, index, internal_name, ref="", display_name=""):
    self.index = index
    self.internalName = internal_name
    self.ref = ref
    self.displayName = display_name or internal_name


class FakeParams:
  """dict-backed stand-in for the get/put surface the bootstrap uses"""
  def __init__(self, initial=None):
    self._store = dict(initial or {})

  def get(self, key):
    return self._store.get(key)

  def put(self, key, val):
    self._store[key] = val

  def get_bool(self, key):
    return bool(self._store.get(key, False))

  def put_bool(self, key, val):
    self._store[key] = bool(val)


def _available_with_fm():
  return [FakeBundle(0, "C210M", "aaaa"), FakeBundle(28, DEFAULT_MODEL_SHORT_NAME, FM_REF, "Firehose Model"), FakeBundle(5, "NV", "bbbb")]


class TestFindDefaultBundle:
  def test_matches_by_short_name(self):
    assert find_default_bundle(_available_with_fm()).ref == FM_REF

  def test_highest_index_wins_when_rebuilt(self):
    bundles = [FakeBundle(28, "FM", "old"), FakeBundle(61, "FM", "new")]
    assert find_default_bundle(bundles).ref == "new"

  def test_ignores_fm_without_ref(self):
    assert find_default_bundle([FakeBundle(28, "FM", "")]) is None

  def test_none_when_absent(self):
    assert find_default_bundle([FakeBundle(0, "C210M", "aaaa")]) is None


class TestDefaultModelBootstrap:
  def test_queues_firehose_when_nothing_active(self):
    params = FakeParams()
    maybe_apply_default_model(params, _available_with_fm())
    assert params.get("ModelManager_DownloadRef") == FM_REF
    assert params.get_bool("DefaultModelApplied") is True

  def test_offline_is_noop_and_retries(self):
    params = FakeParams()
    maybe_apply_default_model(params, [FakeBundle(0, "C210M", "aaaa")])
    assert params.get("ModelManager_DownloadRef") is None
    assert params.get_bool("DefaultModelApplied") is False
    maybe_apply_default_model(params, _available_with_fm())
    assert params.get("ModelManager_DownloadRef") == FM_REF
    assert params.get_bool("DefaultModelApplied") is True

  def test_existing_active_bundle_is_respected(self):
    params = FakeParams({"ModelManager_ActiveBundle": {"ref": "aaaa"}})
    maybe_apply_default_model(params, _available_with_fm())
    assert params.get("ModelManager_DownloadRef") is None
    assert params.get_bool("DefaultModelApplied") is True

  def test_chestnut_selection_does_not_count(self):
    params = FakeParams({"ModelManager_ActiveBundleChestnut": {"ref": "cccc"}})
    maybe_apply_default_model(params, _available_with_fm())
    assert params.get("ModelManager_DownloadRef") == FM_REF

  def test_user_queued_download_wins(self):
    params = FakeParams({"ModelManager_DownloadRef": "aaaa"})
    maybe_apply_default_model(params, _available_with_fm())
    assert params.get("ModelManager_DownloadRef") == "aaaa"
    assert params.get_bool("DefaultModelApplied") is False

  def test_applied_once_then_left_alone(self):
    params = FakeParams({"DefaultModelApplied": True})
    maybe_apply_default_model(params, _available_with_fm())
    assert params.get("ModelManager_DownloadRef") is None
