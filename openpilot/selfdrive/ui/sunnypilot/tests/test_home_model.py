"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from unittest import mock

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.sunnypilot import home_model, model_info
from openpilot.selfdrive.ui.sunnypilot.tests.helpers import jetlink_status
from openpilot.selfdrive.ui.ui_state import ChestnutState
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL_REF, DEFAULT_MODEL_REF


def _entry(short: str, display: str, ref: str) -> dict:
  return {"index": 1, "short_name": short, "display_name": display, "ref": ref, "generation": 12, "environment": "release",
          "minimum_selector_version": REQUIRED_JSON_VERSION, "models": [], "runner": custom.ModelManagerSP.Runner.tinygrad}


SMALL = _entry("CSM", "Custom Small Model (May 01, 2026)", "a" * 40)
BIG = _entry("CTV3M", "Cinque Terre V3 Model (September 17, 2026)", "c" * 40)


def _slot(entry: dict) -> dict:
  bundle = custom.ModelManagerSP.ModelBundle.new_message()
  bundle.ref, bundle.internalName, bundle.displayName = entry["ref"], entry["short_name"], entry["display_name"]
  bundle.minimumSelectorVersion = REQUIRED_JSON_VERSION
  bundle.runner = custom.ModelManagerSP.Runner.tinygrad
  return bundle.to_dict()


class TestCarryingShortName(OpenpilotTestCase):
  """The home names whatever carries by its list's short name: a slot's bundle, jetlink's
  model, or a slot default, which is named for the tree rather than its list entry."""

  def setUp(self):
    super().setUp()
    home_model._list_short_names.cache_clear()
    self.params = {
      "ModelManager_ModelsCache": {"bundles": [_entry("C210M", "CD210 Model (February 03, 2026)", DEFAULT_MODEL_REF), SMALL]},
      "ModelManager_ModelsCache_Chestnut": {"bundles": [_entry("MDMV3", "Mountain Dew Model v3 (October 14, 2026)",
                                                               DEFAULT_BIG_MODEL_REF), BIG]},
      "ModelManager_ActiveBundle": _slot(SMALL),
      "ModelManager_LastSyncTime": 1, "ModelManager_LastSyncTime_Chestnut": 1,
    }
    self.ui_state = mock.MagicMock()
    self.ui_state.chestnut_present = False
    self.ui_state.chestnut_active = False
    self.ui_state.chestnut_loading = False
    self.ui_state.chestnut_state = ChestnutState.DISCONNECTED
    self.ui_state.jetlink = None
    self.ui_state.is_offroad.return_value = False
    self.ui_state.params.get.side_effect = lambda key, **_: self.params.get(key)
    for module in (model_info, home_model):
      patcher = mock.patch.object(module, "ui_state", self.ui_state)
      patcher.start()
      self.addCleanup(patcher.stop)

  def test_jetlink_fall_over_and_handback(self):
    # onroad the mici home is a tap away, so the name follows the carrier both ways
    self.ui_state.jetlink = jetlink_status(enabled=True, mode='usb', present=True, ready=True,
                                           model="Cinque Terre V3 Model (September 17, 2026)")
    for state, short in ((ChestnutState.WAITING, "CSM"),   # small drives until the swap window
                         (ChestnutState.ACTIVE, "CTV3M"),  # joined
                         (ChestnutState.FAILED, "CSM"),    # link lost: the small model drives on
                         (ChestnutState.LOADING, "CSM"),   # rejoining
                         (ChestnutState.ACTIVE, "CTV3M")):  # handed back
      self.ui_state.chestnut_state = state
      assert home_model.carrying_short_name() == short, state

  def test_jetlink_default_is_undated(self):
    self.ui_state.chestnut_state = ChestnutState.ACTIVE
    self.ui_state.jetlink = jetlink_status(enabled=True, mode='usb', present=True, ready=True, model="Mountain Dew Model v3")
    assert home_model.carrying_short_name() == "MDMV3"

  def test_failed_default_big_chestnut_falls_back_to_the_default_small(self):
    # a Default big falls back to the Default small (stock modeld), never the small slot's pick
    self.ui_state.chestnut_present = True
    self.ui_state.chestnut_active = True
    assert home_model.carrying_short_name() == "MDMV3"
    self.ui_state.chestnut_active = False
    self.ui_state.chestnut_state = ChestnutState.FAILED
    assert home_model.carrying_short_name() == "C210M"

  def test_custom_big_with_no_fallback_names_its_pick(self):
    self.ui_state.chestnut_present = True
    self.params["ModelManager_ActiveBundleChestnut"] = _slot(BIG)
    assert model_info.carrying_model() == (None, None, None)
    assert home_model.carrying_short_name() == "CTV3M"

  def test_a_name_no_list_holds_keeps_its_own_undated(self):
    self.ui_state.chestnut_state = ChestnutState.ACTIVE
    self.ui_state.jetlink = jetlink_status(enabled=True, mode='usb', present=True, ready=True, model="Flip Flop Model (October 23, 2026)")
    assert home_model.carrying_short_name() == "Flip Flop Model"
    # the lists are read once per refetch, not per call
    self.params["ModelManager_ModelsCache_Chestnut"]["bundles"].append(_entry("FFM", "Flip Flop Model (October 23, 2026)", "f" * 40))
    assert home_model.carrying_short_name() == "Flip Flop Model"
    self.params["ModelManager_LastSyncTime_Chestnut"] = 2
    assert home_model.carrying_short_name() == "FFM"
