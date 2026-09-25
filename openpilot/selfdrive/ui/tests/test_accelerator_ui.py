"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# ui_state's accelerator view beside upstream's chestnut state, and the tici
# models panel's link toggle and status line.

import os
from contextlib import ExitStack
from unittest import mock

import pytest

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")


@pytest.fixture(scope="module")
def gui():
  """Hidden raylib window + isolated params dir. Widgets need textures, so a window is required."""
  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("test_accelerator_ui", fps=30)
    yield gui_app
    gui_app.close()


@pytest.fixture
def params(gui):
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.ui_state import ui_state

  p = Params()
  ui_state.params = p
  ui_state.update_params()
  return p


def accelerator(present=False, ready=False, progress=None, enabled=False, selected=None, installed=False):
  stack = ExitStack()
  for name, value in (("present", present), ("ready", ready), ("progress", progress),
                      ("enabled", enabled), ("selected_model_name", selected),
                      ("unavailable_reason", None), ("installed", installed)):
    stack.enter_context(mock.patch(f"openpilot.sunnypilot.accelerators.{name}", return_value=value))
  return stack


class FakeSM:
  def __init__(self, board, big=False, alive=False, recv=0, state='none'):
    self.board = board
    self.recv_frame = {"modelV2": recv}
    self.alive = {"modelV2": alive}
    self.big = big
    self.state = state

  def __getitem__(self, name):
    if name == "deviceState":
      return type("DS", (), {"chestnutPresent": self.board})()
    if name == "modelDataV2SP":
      return type("SP", (), {"acceleratorState": self.state})()
    assert name == "modelV2"
    return type("M", (), {"big": self.big})()


class TestUIStateAcceleratorView:
  @staticmethod
  def _with(sm, started=False):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.accelerator_view, ui_state.chestnut_present
    ui_state.sm, ui_state.started, ui_state.started_frame = sm, started, 0
    ui_state.chestnut_present = sm.board
    return saved

  @staticmethod
  def _restore(saved):
    from openpilot.selfdrive.ui.ui_state import ui_state
    ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.accelerator_view, ui_state.chestnut_present = saved

  def test_a_fitted_chestnut_never_asks_the_accelerator(self, params):
    """A board present is upstream's path byte for byte: no view, and nothing in the
    state update consults sunnypilot/accelerators."""
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=True))
    try:
      with accelerator(present=True, ready=True, enabled=True):
        ui_state.update_params()
      assert ui_state.accelerator_view is None
      ui_state.chestnut_compiled = True
      with mock.patch("openpilot.sunnypilot.accelerators.present", side_effect=AssertionError("consulted")), \
           mock.patch("openpilot.sunnypilot.accelerators.ready", side_effect=AssertionError("consulted")):
        ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.READY
    finally:
      self._restore(saved)

  def test_no_board_and_nothing_of_ours_is_no_view(self, params):
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False))
    try:
      with accelerator():
        ui_state.update_params()
      assert ui_state.accelerator_view is None
      ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.DISCONNECTED
    finally:
      self._restore(saved)

  def test_an_attached_accelerator_builds_the_view(self, params):
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False))
    try:
      with accelerator(present=True, ready=True, enabled=True):
        ui_state.update_params()
      view = ui_state.accelerator_view
      assert view is not None and view.present and view.ready
      ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.READY
    finally:
      self._restore(saved)

  def test_a_stored_tinygrad_bundle_reads_compiled_with_the_link_on_or_off(self, params):
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
        with accelerator(present=True, enabled=enabled):
          ui_state.update_params()
        assert ui_state.model_runner_tinygrad
        assert ui_state.chestnut_compiled is True
    finally:
      ui_state.params = real
      ui_state.chestnut_compiled = compiled
      self._restore(saved)

  def test_onroad_disconnected_then_active(self, params):
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False, alive=True, recv=1, state='retrying'), started=True)
    try:
      with accelerator(present=False, ready=True, enabled=True):
        ui_state.update_params()
      assert ui_state.accelerator_view is not None
      ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.DISCONNECTED

      ui_state.sm = FakeSM(board=False, big=True, alive=True, recv=1, state='running')
      with accelerator(present=True, ready=True, enabled=True):
        ui_state.update_params()
      ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.ACTIVE
    finally:
      self._restore(saved)

  def test_onroad_ready_is_not_the_loading_pulse(self, params):
    # Loaded and waiting for a window, which on a MADS car is the rest of the
    # drive unless the driver stops. A pulsing "loading" icon through all of it
    # is what "waited for the green icon, never turned off the car" was.
    from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
    from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
    saved = self._with(FakeSM(board=False, alive=True, recv=1, state='ready'), started=True)
    try:
      # the status name is cached where sm is read, and update_params only copies
      # it into the view; without this the view carries the previous 'none' and
      # the state falls through to FAILED
      UIStateSP.update(ui_state)
      with accelerator(present=True, ready=True, enabled=True):
        ui_state.update_params()
      ui_state._update_chestnut_state()
      assert ui_state.chestnut_state == ChestnutState.WAITING
    finally:
      self._restore(saved)

  def test_the_status_name_is_read_where_sm_updates(self, params):
    from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = self._with(FakeSM(board=False, state='joining'))
    try:
      UIStateSP.update(ui_state)
      with accelerator(present=True):
        ui_state.update_params()
      assert ui_state.accelerator_view.state == 'joining'
    finally:
      self._restore(saved)


class TestTiciModelsPanel:
  """The big model is the model manager's slot for chestnut and accelerator alike;
  the panel adds only the link toggle and a status line."""

  @staticmethod
  def _layout():
    from openpilot.selfdrive.ui.sunnypilot.layouts.settings.models import ModelsLayout
    return ModelsLayout()

  def test_hidden_on_a_plain_device(self, params):
    with accelerator():
      layout = self._layout()
    assert not layout.accelerator_link_item.is_visible

  def test_shown_with_an_accelerator(self, params):
    with accelerator(present=True, selected='Cinque Terre'):
      layout = self._layout()
    assert layout.accelerator_link_item.is_visible

  def test_shown_wherever_the_package_is_installed(self, params):
    # with the link off there is no gadget for a Jetson to enumerate, so present()
    # alone would hide the toggle that turns the link on
    params.remove("JetlinkEnabled")
    with accelerator(installed=True):
      layout = self._layout()
    assert layout.accelerator_link_item.is_visible

  def test_the_toggle_says_what_is_on_the_port(self, params):
    link = "openpilot.selfdrive.ui.sunnypilot.accelerator_link"
    with accelerator(installed=True), mock.patch(f"{link}.read", return_value="0"):
      layout = self._layout()
      assert layout.accelerator_link_item.description.endswith("Nothing on the USB port.")
      with mock.patch(f"{link}.read", return_value="2"):
        layout._refresh_accelerator_items()
      assert layout.accelerator_link_item.description.endswith("A device is on the USB port.")
    with accelerator(installed=True, present=True), mock.patch(f"{link}.read", return_value="0"):
      layout._refresh_accelerator_items()
      assert layout.accelerator_link_item.description.endswith("Accelerator connected.")
    # a kernel without the CC pin in sysfs claims nothing rather than an empty port
    with accelerator(installed=True), mock.patch(f"{link}.read", return_value=None):
      layout._refresh_accelerator_items()
      assert layout.accelerator_link_item.description.endswith("accelerator.")

  def test_toggle_writes_the_param_and_leaves_the_runner_alone(self, params):
    # the small model is the model manager's: the link does not decide which modeld runs
    with accelerator(present=True), mock.patch.object(ui_state_module().ui_state, "is_offroad", return_value=True), \
         mock.patch.object(params, "remove", wraps=params.remove) as remove:
      layout = self._layout()
      layout._set_link_state(True)
      assert params.get_bool("JetlinkEnabled") is True
      layout._set_link_state(False)
      assert params.get_bool("JetlinkEnabled") is False
    assert "ModelRunnerTypeCache" not in {c.args[0] for c in remove.call_args_list}

  def test_toggle_is_inert_onroad(self, params):
    params.remove("JetlinkEnabled")
    with accelerator(present=True), mock.patch.object(ui_state_module().ui_state, "is_offroad", return_value=False):
      layout = self._layout()
      layout._set_link_state(True)
      assert params.get("JetlinkEnabled") is None
      assert layout.accelerator_link_item.action_item.get_state() is False

  def test_status_note_names_the_accelerator_not_the_chestnut(self, params):
    from openpilot.selfdrive.ui.sunnypilot.ui_state import AcceleratorView
    ui_state = ui_state_module().ui_state
    saved = ui_state.accelerator_view, ui_state.chestnut_present
    try:
      ui_state.chestnut_present = False
      with accelerator(present=True, selected='Cinque Terre'), \
           mock.patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.models.big_model_state", return_value=None):
        layout = self._layout()
        ui_state.accelerator_view = AcceleratorView(True, False, None, 'none')
        note = layout._status_note()
        assert "chestnut" not in note
        assert "Cinque Terre will drive when the accelerator is ready." == note
        ui_state.accelerator_view = AcceleratorView(True, True, None, 'none')
        note = layout._status_note()
        assert note.startswith("Cinque Terre will drive.") and "chestnut" not in note
    finally:
      ui_state.accelerator_view, ui_state.chestnut_present = saved

  def test_the_note_says_what_the_switch_is_waiting_for(self, params):
    from openpilot.selfdrive.ui.sunnypilot.ui_state import AcceleratorView
    ui_state = ui_state_module().ui_state
    saved = ui_state.accelerator_view, ui_state.chestnut_present
    try:
      ui_state.chestnut_present = False
      with accelerator(present=True, ready=True, selected='Cinque Terre'), \
           mock.patch("openpilot.selfdrive.ui.sunnypilot.layouts.settings.models.big_model_state", return_value='ready'):
        layout = self._layout()
        ui_state.accelerator_view = AcceleratorView(True, True, None, 'ready')
        assert layout._status_note() == "Cinque Terre is ready. Stop with cruise off, or turn lateral off, to switch."
    finally:
      ui_state.accelerator_view, ui_state.chestnut_present = saved

  def test_a_chestnut_hides_the_link_toggle(self, params):
    from openpilot.selfdrive.ui.sunnypilot.accelerator_link import link_toggle_meaningful
    ui_state = ui_state_module().ui_state
    saved = ui_state.chestnut_present
    try:
      with accelerator(present=True, selected='Cinque Terre'):
        ui_state.chestnut_present = False
        assert link_toggle_meaningful()
        ui_state.chestnut_present = True
        assert not link_toggle_meaningful()
    finally:
      ui_state.chestnut_present = saved

  def test_panel_renders(self, params):
    import pyray as rl
    with accelerator(present=True, selected='Cinque Terre'):
      layout = self._layout()
      layout.render(rl.Rectangle(0, 0, 800, 600))


def ui_state_module():
  from openpilot.selfdrive.ui import ui_state
  return ui_state
