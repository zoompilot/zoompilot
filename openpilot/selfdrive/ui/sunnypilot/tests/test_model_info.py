"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from unittest import mock

from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.sunnypilot import accelerator_link, model_info
from openpilot.selfdrive.ui.sunnypilot.tests.helpers import jetlink_status
from openpilot.selfdrive.ui.ui_state import ChestnutState
from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL, DEFAULT_MODEL

V3_REF = "bf3e3631b3f91d92a1020a5e0dd4298b93ff4244"


def _raw_bundle(ref: str) -> dict:
  bundle = custom.ModelManagerSP.ModelBundle.new_message()
  bundle.ref = ref
  bundle.minimumSelectorVersion = REQUIRED_JSON_VERSION
  bundle.internalName = ref
  bundle.displayName = ref
  bundle.runner = custom.ModelManagerSP.Runner.tinygrad
  return bundle.to_dict()


class TestCarryingModel(OpenpilotTestCase):
  """What the UI names as driving has to be what manager runs. The small model is
  the model manager's whatever the Jetlink setting says: the stored qcom bundle
  drives until jetlink joins, and again if it goes."""

  def setUp(self):
    super().setUp()
    self.ui_state = mock.MagicMock()
    self.ui_state.chestnut_present = False
    self.ui_state.chestnut_state = ChestnutState.DISCONNECTED
    self.ui_state.chestnut_active = False
    self.ui_state.chestnut_loading = False
    self.ui_state.jetlink = None
    self.ui_state.is_offroad.return_value = True
    self.ui_state.params.get.side_effect = lambda key: {"ModelManager_ActiveBundle": _raw_bundle("custom_small")}.get(key)
    patcher = mock.patch.object(model_info, "ui_state", self.ui_state)
    patcher.start()
    self.addCleanup(patcher.stop)

  def test_the_link_on_still_names_the_stored_bundle(self):
    for enabled in (True, False):
      self.ui_state.jetlink = jetlink_status(enabled=enabled, mode='usb' if enabled else 'off')
      assert model_info.carrying_model() == ("qcom", "custom_small", "custom_small")
      source, active, _ = model_info.model_info()
      assert (source, active) == ("qcom", "custom_small")

  def test_an_empty_slot_names_the_default_small_model(self):
    self.ui_state.params.get.side_effect = lambda key: None
    assert model_info.carrying_model() == ("qcom", f"{DEFAULT_MODEL} (Default)", f"{DEFAULT_MODEL} (Default)")

  def test_link_active_names_the_accelerator_model(self):
    self.ui_state.chestnut_state = ChestnutState.ACTIVE
    self.ui_state.jetlink = jetlink_status(enabled=True, mode='usb', present=True, ready=True, model="big")
    assert model_info.carrying_model() == ("accelerator", "big", "big")

  def test_fitted_board_is_chestnut_notjetlink_status(self):
    # a real chestnut ACTIVE keeps comma's semantics whatever the accelerator says
    self.ui_state.chestnut_present = True
    self.ui_state.chestnut_state = ChestnutState.ACTIVE
    self.ui_state.params.get.side_effect = lambda key: {"ModelManager_ActiveBundleChestnut": _raw_bundle("big_custom")}.get(key)
    self.ui_state.jetlink = jetlink_status(enabled=True, mode='usb', present=True, ready=True, model="jetlink_big")
    assert model_info.carrying_model() == ("chestnut", "big_custom", "big_custom")


class TestDefaultBigModelName(OpenpilotTestCase):
  """An empty big slot runs the chestnut's in-tree model when a board is fitted,
  and the accelerator's default when none is: jetlink's, named from its catalog."""

  def setUp(self):
    super().setUp()
    self.ui_state = mock.MagicMock()
    patcher = mock.patch.object(model_info, "ui_state", self.ui_state)
    patcher.start()
    self.addCleanup(patcher.stop)

  def test_a_fitted_chestnut_names_the_in_tree_model(self):
    self.ui_state.chestnut_present = True
    self.ui_state.jetlink = jetlink_status(default_model="a chestnut never names jetlink's default")
    assert model_info.default_model_name("chestnut") == f"{DEFAULT_BIG_MODEL} (Default)"
    assert model_info.default_model_name("qcom") == f"{DEFAULT_MODEL} (Default)"

  def test_without_a_chestnut_the_accelerator_names_its_default(self):
    # jetlink's default ref, from the pinned jetlink, not the fork's model_name,
    # named off the model manager's catalog as the adapter hands it over
    from openpilot.common.params import Params
    from openpilot.sunnypilot import jetlink_adapter
    self.ui_state.chestnut_present = False
    bundles = [{"display_name": "Cinque Terre V3 Model (September 17, 2026)", "ref": V3_REF, "index": 1,
                "minimum_selector_version": REQUIRED_JSON_VERSION},
               {"display_name": "BMRLNAP Model v4 (August 30, 2026)", "ref": "f877d7a0ccc3cce943c76e285214c020cd65c899",
                "index": 2, "minimum_selector_version": REQUIRED_JSON_VERSION}]
    Params().put(jetlink_adapter.KEYS.catalog, {"bundles": bundles}, block=True)
    with mock.patch.object(jetlink_adapter, "_bound", None):
      self.ui_state.jetlink = jetlink_adapter.status()
    assert model_info.default_model_name("chestnut") == "Cinque Terre V3 Model (Default)"
    assert model_info.default_model_name("qcom") == f"{DEFAULT_MODEL} (Default)"

  def test_an_accelerator_with_no_catalog_falls_back_to_the_in_tree_name(self):
    self.ui_state.chestnut_present = False
    for jetlink in (jetlink_status(default_model=None), None):
      self.ui_state.jetlink = jetlink
      assert model_info.default_model_name("chestnut") == f"{DEFAULT_BIG_MODEL} (Default)"


class TestAChestnutArrivingMidRender(OpenpilotTestCase):
  """The params thread sets ui_state.jetlink to None when a chestnut is
  plugged in, between any two reads the render thread makes. Each function
  reads it once, so a layout drawing at that moment cannot raise."""

  def setUp(self):
    super().setUp()
    self.ui_state = mock.MagicMock()
    self.ui_state.chestnut_present = False
    self.ui_state.chestnut_state = ChestnutState.ACTIVE
    snapshot = jetlink_status(enabled=True, mode='usb', present=True, ready=True, model="big", default_model="jetlink's",
                        progress={'stage': 'build', 'frac': 0.5, 'msg': 'building'})
    self.reads = 0

    def read():
      # the snapshot on a call's first read, and None after it: the chestnut arrived
      self.reads += 1
      return snapshot if self.reads == 1 else None
    type(self.ui_state).jetlink = mock.PropertyMock(side_effect=read)
    for module in (model_info, accelerator_link):
      patcher = mock.patch.object(module, "ui_state", self.ui_state)
      patcher.start()
      self.addCleanup(patcher.stop)

  def test_each_reader_answers_from_the_snapshot_it_read(self):
    for call, expected in ((lambda: model_info.default_model("chestnut"), "jetlink's"),
                           (accelerator_link.big_model_progress, ("build", 0.5, "building")),
                           (model_info.carrying_model, ("accelerator", "big", "big"))):
      self.reads = 0
      assert call() == expected
