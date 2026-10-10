"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

ui_state's jetlink view beside upstream's chestnut state, and the tici
models panel's link toggle and status line. jetlink is one mock: the adapter's
status(), which the params pass reads.
"""
import os
import time
from contextlib import contextmanager
from types import SimpleNamespace
from unittest import mock

import pytest

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.ui.sunnypilot.tests.helpers import jetlink_status

# conftest's window, once for the module; every test then runs under its own prefix
pytestmark = pytest.mark.usefixtures("gui")


class UITest(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    from openpilot.common.params import Params
    from openpilot.selfdrive.ui.ui_state import ui_state
    self.params = Params()
    ui_state.params = self.params
    ui_state.update_params()


@contextmanager
def jetlink(installed=True, **fields):
  """The adapter's status() while the block runs, and a params pass over it.
  Not installed is None: no jetlink on this device."""
  from openpilot.selfdrive.ui.ui_state import ui_state
  status = jetlink_status(**fields) if installed else None
  with mock.patch("openpilot.sunnypilot.jetlink_adapter.status", return_value=status) as reads:
    ui_state.update_params()
    yield reads


class FakeSM:
  """What the jetlink view and UIStateZP's per-frame update read off the UI's SubMaster."""
  def __init__(self, board=False, big=False, alive=False, recv=0, state='none'):
    self.board = board
    self.recv_frame = {"modelV2": recv}
    self.alive = {"modelV2": alive}
    self.big = big
    self.state = state

  def __getitem__(self, name):
    return {
      "deviceState": SimpleNamespace(chestnutPresent=self.board),
      "modelDataV2SP": SimpleNamespace(acceleratorState=self.state),
      "modelV2": SimpleNamespace(big=self.big),
      "carOutput": SimpleNamespace(actuatorsOutput=SimpleNamespace(torque=0.0)),
      "carState": SimpleNamespace(vEgo=0.0),
    }[name]


class TestUIStateJetlinkView(UITest):
  @staticmethod
  def _with(sm, started=False):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state.chestnut_present
    ui_state.sm, ui_state.started, ui_state.started_frame = sm, started, 0
    ui_state.chestnut_present = sm.board
    return saved

  @staticmethod
  def _restore(saved):
    from openpilot.selfdrive.ui.ui_state import ui_state
    ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state.chestnut_present = saved

  def test_a_fitted_chestnut_never_asks_jetlink(self):
    """A board present is upstream's path byte for byte: no view, and nothing in the
    state update consults jetlink."""
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=True))
    try:
      with mock.patch("openpilot.sunnypilot.jetlink_adapter.status", side_effect=AssertionError("consulted")):
        ui_state.update_params()
        assert ui_state.jetlink is None and ui_state.jetlink_view is None
        ui_state.chestnut_compiled = True
        ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.READY
    finally:
      self._restore(saved)

  def test_no_board_and_nothing_to_show_is_no_view(self):
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False))
    try:
      # no jetlink here, and jetlink with the link off and nothing attached
      for installed in (False, True):
        with jetlink(installed=installed):
          assert ui_state.jetlink_view is None
          ui_state._update_chestnut_state()
        assert ui_state.chestnut_state == ChestnutState.DISCONNECTED
    finally:
      self._restore(saved)

  def test_an_attached_accelerator_builds_the_view(self):
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False))
    try:
      with jetlink(present=True, ready=True, enabled=True):
        view = ui_state.jetlink_view
        assert view is not None and view.present and view.ready
        ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.READY
    finally:
      self._restore(saved)

  def test_progress_alone_builds_the_view(self):
    # a provisioning run still reporting after the link was turned off
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False))
    try:
      with jetlink(present=True, progress={'stage': 'upload', 'frac': 0.5, 'msg': ''}):
        assert ui_state.jetlink_view is not None
        ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.LOADING
    finally:
      self._restore(saved)

  def test_a_stored_tinygrad_bundle_reads_compiled_with_the_link_on_or_off(self):
    """The small model is the model manager's under the link too: manager runs the
    stored bundle's modeld, so a stored tinygrad bundle counts as compiled as it
    does on develop, whatever the toggle says."""
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS
    saved = self._with(FakeSM(board=False))
    compiled, real = ui_state.chestnut_compiled, ui_state.params
    # served from a wrapper rather than written: the model manager's validation
    # discards a bundle it cannot resolve, from whatever thread reaches it first
    fake = mock.Mock(wraps=real)
    fake.get.side_effect = lambda key, *a, **k: {"runner": "tinygrad"} if key == ACTIVE_BUNDLE_KEYS["qcom"] else real.get(key, *a, **k)
    try:
      for enabled in (True, False):
        ui_state.chestnut_compiled = False
        ui_state.params = fake
        with jetlink(present=True, enabled=enabled):
          pass
        assert ui_state.model_runner_tinygrad
        assert ui_state.chestnut_compiled is True
    finally:
      ui_state.params = real
      ui_state.chestnut_compiled = compiled
      self._restore(saved)

  @staticmethod
  def _state(view, state='none', sm=None, started=False):
    """chestnut_state for jetlink's snapshot `view` and modeld's acceleratorState by name."""
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state._accelerator_state_name
    ui_state.sm, ui_state.started, ui_state.started_frame = sm or FakeSM(), started, 0
    ui_state.jetlink, ui_state._accelerator_state_name = view, state
    try:
      ui_state._update_chestnut_state()
      return ui_state.chestnut_state
    finally:
      ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state._accelerator_state_name = saved

  @staticmethod
  def _view(present=True, ready=False, progress=None):
    """The link on."""
    return jetlink_status(enabled=True, present=present, ready=ready, progress=progress)

  def test_offroad_states(self):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    self.assertEqual(self._state(self._view(present=False)), ChestnutState.DISCONNECTED)
    self.assertEqual(self._state(self._view(ready=True)), ChestnutState.READY)
    self.assertEqual(self._state(self._view()), ChestnutState.UNCOMPILED)
    self.assertEqual(self._state(self._view(progress={'stage': 'build', 'frac': 0.3})), ChestnutState.LOADING)
    self.assertEqual(self._state(self._view(progress={'stage': 'failed', 'frac': 1.0})), ChestnutState.FAILED)
    self.assertEqual(self._state(self._view(ready=True, progress={'stage': 'connect', 'frac': 0.0})), ChestnutState.LOADING)
    self.assertEqual(self._state(self._view(ready=True, progress={'stage': 'failed', 'frac': 1.0})), ChestnutState.FAILED)
    self.assertEqual(self._state(self._view(ready=True, progress={'stage': 'ready', 'frac': 1.0})), ChestnutState.READY)

  def test_onroad_states(self):
    from openpilot.selfdrive.ui.ui_state import ChestnutState, ui_state
    # the link on with nothing on the port is still the link's icon, not the chestnut's
    with mock.patch.object(ui_state, 'jetlink', self._view(present=False, ready=True)):
      self.assertIsNotNone(ui_state.jetlink_view)
    driving = FakeSM(alive=True, recv=1)
    for view, state, expected in (
      # an accelerator that is not there does not pulse "loading" all drive
      (self._view(present=False, ready=True), 'retrying', ChestnutState.DISCONNECTED),
      (self._view(ready=True), 'joining', ChestnutState.LOADING),
      (self._view(ready=True), 'retrying', ChestnutState.LOADING),
      # loaded and waiting for a window, which on a MADS car is the rest of the drive
      # unless the driver stops: not the loading pulse
      (self._view(ready=True), 'ready', ChestnutState.WAITING),
      (self._view(ready=True), 'running', ChestnutState.ACTIVE),
      (self._view(ready=True), 'unavailable', ChestnutState.FAILED),
      (self._view(), 'none', ChestnutState.UNCOMPILED),
    ):
      with self.subTest(state=state, present=view.present):
        self.assertEqual(self._state(view, state, driving, started=True), expected)
    # nothing from modeld yet is loading, not failed
    self.assertEqual(self._state(self._view(ready=True), sm=FakeSM(), started=True), ChestnutState.LOADING)
    # a big frame is proof, whatever the status field says
    big = FakeSM(big=True, alive=True, recv=1)
    self.assertEqual(self._state(self._view(ready=True), 'unavailable', big, started=True), ChestnutState.ACTIVE)

  def test_the_state_name_is_read_where_sm_updates(self):
    # the per-frame pass caches it for the icon, so the params thread never reads a message
    from openpilot.selfdrive.ui.sunnypilot.ui_state_zp import UIStateZP
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.has_icbm, ui_state._accelerator_state_name
    ui_state.sm, ui_state.has_icbm = FakeSM(state='joining'), False
    try:
      UIStateZP.update(ui_state)
      self.assertEqual(ui_state._accelerator_state_name, 'joining')
    finally:
      ui_state.sm, ui_state.has_icbm, ui_state._accelerator_state_name = saved

  @staticmethod
  @contextmanager
  def _usb(present):
    from openpilot.selfdrive.ui import ui_state as module
    with mock.patch.object(module, 'read_int', return_value=1), mock.patch.object(module, 'get_usb_state', return_value=[]), \
         mock.patch("openpilot.sunnypilot.jetlink_adapter.status", return_value=jetlink_status(present=present)):
      yield

  def test_a_present_accelerator_is_not_an_unknown_usb_device(self):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown = True, time.monotonic() - 11.0, False
      with self._usb(present=True):
        ui_state.update_params()  # builds the view
        ui_state.usb_connected_ts = time.monotonic() - 11.0
        ui_state.update_params()  # decides
      self.assertIsNotNone(ui_state.jetlink_view)
      self.assertIs(ui_state.usb_unknown, False)
      with self._usb(present=False):
        ui_state.update_params()
        ui_state.usb_connected_ts = time.monotonic() - 11.0
        ui_state.update_params()
      self.assertIsNone(ui_state.jetlink_view)
      self.assertIs(ui_state.usb_unknown, True)
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink = saved

  def test_an_accelerator_recognised_after_the_grace_period_clears_unknown(self):
    """the Jetson configures the gadget ~25 s after the UI starts, after the one-shot
    usb_unknown decision; presence arriving later must still clear it"""
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown = True, None, True
      with self._usb(present=False):
        ui_state.update_params()
        self.assertIs(ui_state.usb_unknown, True)
      with self._usb(present=True):
        ui_state.update_params()
        self.assertIs(ui_state.usb_unknown, False)
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink = saved


class TestTiciModelsPanel(UITest):
  """The big model is the model manager's slot for chestnut and accelerator alike;
  the panel adds only the link toggle and a status line."""

  def setUp(self):
    super().setUp()
    self.ui_state = ui_state_module().ui_state
    saved = self.ui_state.jetlink, self.ui_state.chestnut_present

    def restore():
      self.ui_state.jetlink, self.ui_state.chestnut_present = saved
    self.addCleanup(restore)

  @staticmethod
  def _layout():
    from openpilot.selfdrive.ui.sunnypilot.layouts.settings.models import ModelsLayout
    return ModelsLayout()

  def test_hidden_on_a_plain_device(self):
    # no jetlink here and the link unset or off
    for link in (None, 0):
      if link is None:
        self.params.remove("JetlinkLink")
      else:
        self.params.put("JetlinkLink", link, block=True)
      with jetlink(installed=False):
        layout = self._layout()
      assert not layout.accelerator_link_item.is_visible, link

  def test_shown_with_an_accelerator(self):
    with jetlink(present=True, model='Cinque Terre'):
      layout = self._layout()
    assert layout.accelerator_link_item.is_visible

  def test_shown_wherever_jetlink_is_installed(self):
    # with the link off there is no gadget for a Jetson to enumerate, so
    # presence alone would hide the toggle that turns the link on. A model built
    # with the Jetson out of the car, which modeld still tries at the next
    # ignition, and a complaint are what the off position is for
    self.params.remove("JetlinkLink")
    for fields in ({}, {'ready': True}, {'reason': 'no gadget'}):
      with jetlink(**fields):
        layout = self._layout()
      assert layout.accelerator_link_item.is_visible, fields

  def test_shown_while_the_link_is_on_even_without_a_usable_jetlink(self):
    # the offroad alert says why it cannot run; the toggle is how to turn it off
    self.params.put("JetlinkLink", 1, block=True)
    with jetlink(installed=False):
      layout = self._layout()
    assert layout.accelerator_link_item.is_visible

  def test_the_toggle_says_what_is_on_the_port(self):
    with jetlink(port='empty'):
      layout = self._layout()
      assert layout.accelerator_link_item.description.endswith("Nothing on the USB port.")
    with jetlink(port='host'):
      layout._refresh_accelerator_items()
      assert layout.accelerator_link_item.description.endswith("A device is on the USB port.")
    with jetlink(present=True, port='host'):
      layout._refresh_accelerator_items()
      assert layout.accelerator_link_item.description.endswith("Jetlink connected: USB.")
    # a kernel without the CC pin in sysfs claims nothing rather than an empty port
    with jetlink(port=None):
      layout._refresh_accelerator_items()
      assert layout.accelerator_link_item.description.endswith("Turns off ADB.")

  def test_the_status_names_the_transport(self):
    # the setting names the host: USB for a Jetson, a Linux PC or a Mac, iOS for
    # a phone dialed in over the gadget's network interface. jetlink says which
    from openpilot.selfdrive.ui.sunnypilot.accelerator_link import link_status
    with jetlink(present=True, transport='iOS over USB (192.168.60.3)'):
      assert link_status() == "Jetlink connected: iOS over USB (192.168.60.3)."
    with jetlink(installed=False):
      assert link_status() == ""

  def test_the_buttons_are_bound_to_the_one_param(self):
    # the small model is the model manager's: the link does not decide which modeld runs
    from openpilot.selfdrive.ui.sunnypilot.accelerator_link import link_mode
    with jetlink(present=True), mock.patch.object(ui_state_module().ui_state, "is_offroad", return_value=True):
      layout = self._layout()
      action = layout.accelerator_link_item.action_item
      assert action.param_key == "JetlinkLink"
      for index, mode in enumerate(("off", "usb", "ios")):
        self.params.put("JetlinkLink", index, block=True)
        layout._refresh_accelerator_items()
        assert action.get_selected_button() == index
        assert link_mode() == mode

  def test_the_modes_are_inert_onroad(self):
    self.params.remove("JetlinkLink")
    with jetlink(present=True), mock.patch.object(ui_state_module().ui_state, "is_offroad", return_value=False):
      layout = self._layout()
      layout._refresh_accelerator_items()
      assert not layout.accelerator_link_item.action_item.enabled

  def test_status_note_names_the_accelerator_not_the_chestnut(self):
    ui_state = ui_state_module().ui_state
    saved = ui_state.jetlink, ui_state.chestnut_present
    try:
      ui_state.chestnut_present = False
      with jetlink(present=True, model='Cinque Terre'), \
           mock.patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.models.big_model_state", return_value=None):
        layout = self._layout()
        note = layout._status_note()
        assert "chestnut" not in note
        assert "Cinque Terre will drive when Jetlink is ready." == note
        ui_state.jetlink = jetlink_status(present=True, ready=True, enabled=True, model='Cinque Terre')
        # no "until the next drive": the link rejoins all drive
        assert layout._status_note() == "Cinque Terre will drive."
    finally:
      ui_state.jetlink, ui_state.chestnut_present = saved

  def test_the_note_names_the_model_that_drives_until_the_pick_is_ready(self):
    # the last model the Jetson built drives while the pick downloads and builds
    self.ui_state.chestnut_present = False
    with jetlink(present=True, enabled=True, model='ResAction Preview', standin='Cinque Terre V3'), \
         mock.patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.models.big_model_state", return_value=None):
      assert self._layout()._status_note() == "Cinque Terre V3 drives until ResAction Preview is ready."

  def test_a_big_model_line_says_whether_it_is_built_or_here(self):
    from openpilot.sunnypilot import jetlink_adapter
    bundle = mock.Mock(ref='r1', displayName='Cinque Terre V3', internalName='CTV3')
    self.ui_state.chestnut_present = False
    with jetlink(present=True, enabled=True), mock.patch.object(jetlink_adapter, 'model_state', return_value='ready'):
      layout = self._layout()
      assert layout._bundle_to_node(bundle, noted=True).data['display_name'] == "Cinque Terre V3 · ready on Jetson"
      assert layout._bundle_to_node(bundle).data['display_name'] == "Cinque Terre V3"

  def test_the_note_says_what_the_switch_is_waiting_for(self):
    ui_state = ui_state_module().ui_state
    saved = ui_state.jetlink, ui_state.chestnut_present
    try:
      ui_state.chestnut_present = False
      with jetlink(present=True, ready=True, enabled=True, model='Cinque Terre'), \
           mock.patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.models.big_model_state", return_value='ready'):
        layout = self._layout()
        assert layout._status_note() == "Cinque Terre is ready. Disengage fully, then re-engage to switch."
    finally:
      ui_state.jetlink, ui_state.chestnut_present = saved

  def test_a_chestnut_hides_the_link_toggle(self):
    from openpilot.selfdrive.ui.sunnypilot.accelerator_link import link_toggle_meaningful
    ui_state = ui_state_module().ui_state
    saved = ui_state.chestnut_present
    try:
      with jetlink(present=True, model='Cinque Terre'):
        ui_state.chestnut_present = False
        assert link_toggle_meaningful()
        ui_state.chestnut_present = True
        assert not link_toggle_meaningful()
    finally:
      ui_state.chestnut_present = saved

  def test_panel_renders(self):
    import pyray as rl
    with jetlink(present=True, model='Cinque Terre'):
      layout = self._layout()
      layout.render(rl.Rectangle(0, 0, 800, 600))

  def test_an_empty_big_slot_names_whose_default_runs(self):
    # a chestnut runs the model in the tree; without one the slot is the
    # accelerator's, and so is its default
    import pyray as rl
    from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL
    ui_state = ui_state_module().ui_state
    saved = ui_state.chestnut_present
    self.params.remove("ModelManager_ActiveBundleChestnut")
    try:
      for board, expected in ((True, f"{DEFAULT_BIG_MODEL} (Default)"), (False, "Cinque Terre V3 Model (Default)")):
        with jetlink(present=True, default_model="Cinque Terre V3 Model"):
          ui_state.chestnut_present = board
          layout = self._layout()
          layout.render(rl.Rectangle(0, 0, 800, 600))
          assert layout.big_model_item.action_item.value == expected
    finally:
      ui_state.chestnut_present = saved

  def test_refresh_spins_beside_the_link_until_both_catalogs_are_stamped(self):
    # sunnypilot's refresh, as upstream: the model manager restamps each catalog it
    # refetches, the big-model one extended for the accelerator or not
    import pyray as rl
    from openpilot.selfdrive.ui.sunnypilot.model_info import MODEL_SYNC_KEYS
    for key in MODEL_SYNC_KEYS:
      self.params.put(key, 1, block=True)
    with jetlink(present=True, model='Cinque Terre'), \
         mock.patch.object(ui_state_module().ui_state, "is_offroad", return_value=True):
      layout = self._layout()
      button = layout.refresh_item.action_item

      def shown():
        layout.render(rl.Rectangle(0, 0, 800, 600))
        return button.text, button.enabled

      assert layout.accelerator_link_item.is_visible and layout.refresh_item in layout.items
      assert shown() == ("REFRESH", True)
      layout._refresh_models()
      wait_until(lambda: not any(self.params.get(key) for key in MODEL_SYNC_KEYS))
      assert shown() == ("FETCHING...", False)
      self.params.put(MODEL_SYNC_KEYS[0], 2, block=True)
      assert shown() == ("FETCHING...", False)
      self.params.put(MODEL_SYNC_KEYS[1], 2, block=True)
      assert shown() == ("REFRESH", True)

  def test_the_spinner_watches_the_keys_the_manager_stamps(self):
    from openpilot.selfdrive.ui.sunnypilot.model_info import MODEL_SYNC_KEYS
    from openpilot.sunnypilot.models.fetcher import ModelCache, ModelFetcher
    stamped = tuple(ModelCache(self.params, suffix=suffix)._LAST_SYNC_KEY for _, suffix in ModelFetcher.MODEL_SOURCES.values())
    assert MODEL_SYNC_KEYS == stamped


def wait_until(condition, timeout=2.0):
  """The panels write with a non-blocking put(), which lands on a background thread."""
  import time
  deadline = time.monotonic() + timeout
  while not condition():
    assert time.monotonic() < deadline, "timed out"
    time.sleep(0.005)


def ui_state_module():
  from openpilot.selfdrive.ui import ui_state
  return ui_state


class TestTheUsbPort(UITest):
  """ADB and Jetlink share the comma's USB port: the link on
  turns ADB off and greys its toggle out."""

  def setUp(self):
    super().setUp()
    from openpilot.selfdrive.ui.ui_state import ui_state
    self.ui = ui_state
    self.saved = ui_state.jetlink

  def tearDown(self):
    self.ui.jetlink = self.saved
    super().tearDown()

  def set(self, adb, link):
    """link is jetlink's snapshot: None (a chestnut, or no jetlink) or enabled or not."""
    self.params.put_bool("AdbEnabled", adb, block=True)
    self.ui.jetlink = None if link is None else jetlink_status(enabled=link)
    self.ui._enforce_usb_port()
    return self.params.get_bool("AdbEnabled"), self.ui.adb_blocked

  def test_the_link_on_turns_adb_off_and_blocks_it(self):
    self.assertEqual(self.set(adb=True, link=True), (False, True))

  def test_the_link_off_leaves_adb_alone(self):
    self.assertEqual(self.set(adb=True, link=False), (True, False))
    self.assertEqual(self.set(adb=False, link=False), (False, False))

  def test_no_jetlink_snapshot_leaves_adb_alone(self):
    # a fitted chestnut, or no jetlink on this device
    self.assertEqual(self.set(adb=True, link=None), (True, False))

  def test_both_developer_panels_grey_adb_out(self):
    from openpilot.selfdrive.ui.sunnypilot.layouts.settings.developer import DeveloperLayoutSP
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.developer import DeveloperLayoutMiciSP
    tici, mici = DeveloperLayoutSP(), DeveloperLayoutMiciSP()
    with mock.patch.object(self.ui, 'is_offroad', return_value=True):
      for link, enabled in ((False, True), (True, False)):
        self.set(adb=False, link=link)
        self.assertEqual(tici._adb_toggle.action_item.enabled, enabled)
        self.assertEqual(mici._adb_toggle.enabled, enabled)

