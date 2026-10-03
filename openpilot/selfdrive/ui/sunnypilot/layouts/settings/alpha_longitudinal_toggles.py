"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The alpha switch and the modes that run on openpilot longitudinal control, at the top of the tizi
Cruise panel. Toggles and Developer drop their copies of the first two (TogglesLayoutSP,
DeveloperLayoutSP) so each switch appears once.
"""
from openpilot.selfdrive.ui.layouts.settings.developer import DESCRIPTIONS as DEVELOPER_DESCRIPTIONS
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog

# upstream's Toggles text (layouts/settings/toggles.py), shown again in the confirm dialog
EXPERIMENTAL_MODE_DESCRIPTION = tr_noop(
  "sunnypilot defaults to driving in chill mode. Experimental mode enables alpha-level features that aren't ready for chill mode. " +
  "Experimental features are listed below:<br>" +
  "<h4>End-to-End Longitudinal Control</h4><br>" +
  "Let the driving model control the gas and brakes. sunnypilot will drive as it thinks a human would, including stopping for red lights and stop signs. " +
  "Since the driving model decides the speed to drive, the set speed will only act as an upper bound. This is an alpha quality feature; " +
  "mistakes should be expected.<br>" +
  "<h4>New Driving Visualization</h4><br>" +
  "The driving visualization will transition to the road-facing wide-angle camera at low speeds to better show some turns. " +
  "The Experimental mode logo will also be shown in the top right corner."
)


class AlphaLongitudinalToggles:
  """Not a panel: CruiseLayout lists `items` in its own scroller and drives the two hooks."""
  def __init__(self):
    self._alpha_long_toggle = toggle_item_sp(
      title=lambda: tr("sunnypilot Longitudinal Control (Alpha)"),
      description=lambda: tr(DEVELOPER_DESCRIPTIONS["alpha_longitudinal"]),
      callback=self._on_alpha_long,
      enabled=ui_state.is_offroad)

    self._experimental_toggle = toggle_item_sp(
      title=lambda: tr("Experimental Mode"),
      description=lambda: tr(EXPERIMENTAL_MODE_DESCRIPTION),
      callback=self._on_experimental_mode)

    self._dec_toggle = toggle_item_sp(
      title=lambda: tr("Enable Dynamic Experimental Control"),
      description=lambda: tr("Enable toggle to allow the model to determine when to use zoompilot ACC or zoompilot End to End Longitudinal."),
      param="DynamicExperimentalControl")

    self._set_speed_toggle = toggle_item_sp(
      title=lambda: tr("Set Speed Nudge"),
      description=lambda: tr("In Experimental Mode, gently raise speed toward the set speed when nothing calls for slowing down. " +
                             "Has no effect while Dynamic Experimental Control is on."),
      param="ExperimentalModeSetSpeed")

    self._refresh_toggles = (
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ExperimentalMode", self._experimental_toggle),
      ("DynamicExperimentalControl", self._dec_toggle),
      ("ExperimentalModeSetSpeed", self._set_speed_toggle),
    )
    self.items = [item for _, item in self._refresh_toggles]

    ui_state.add_offroad_transition_callback(self.refresh)
    ui_state.add_engaged_transition_callback(self.refresh)

  def update_state(self):
    CP = ui_state.CP
    has_long = CP is not None and ui_state.has_longitudinal_control
    self._alpha_long_toggle.set_visible(CP is not None and CP.alphaLongitudinalAvailable)
    self._experimental_toggle.action_item.set_enabled(CP is None or has_long)
    self._dec_toggle.action_item.set_enabled(has_long)
    # DEC decides the mode on its own, and the set-speed floor never acts under it
    self._set_speed_toggle.action_item.set_enabled(has_long and not self._dec_toggle.action_item.get_state())

  def refresh(self):
    for key, item in self._refresh_toggles:
      item.action_item.set_state(ui_state.params.get_bool(key))

  @staticmethod
  def _confirm(item, callback) -> None:
    content = f"<h1>{item.title}</h1><br><p>{item.description}</p>"
    gui_app.push_widget(ConfirmDialog(content, tr("Enable"), rich=True, callback=callback))

  def _on_alpha_long(self, state: bool):
    if not state:
      ui_state.params.put_bool("AlphaLongitudinalEnabled", False, block=True)
      return

    def on_result(result: DialogResult):
      if result == DialogResult.CONFIRM:
        # param only: card watches for the change and requests the onroad cycle itself, after
        # any radar hand-back the brand needs
        ui_state.params.put_bool("AlphaLongitudinalEnabled", True, block=True)
      else:
        self._alpha_long_toggle.action_item.set_state(False)

    self._confirm(self._alpha_long_toggle, on_result)

  def _on_experimental_mode(self, state: bool):
    if not state or ui_state.params.get_bool("ExperimentalModeConfirmed"):
      ui_state.params.put_bool("ExperimentalMode", state, block=True)
      return

    def on_result(result: DialogResult):
      if result == DialogResult.CONFIRM:
        ui_state.params.put_bool("ExperimentalMode", True, block=True)
        ui_state.params.put_bool("ExperimentalModeConfirmed", True, block=True)
      else:
        self._experimental_toggle.action_item.set_state(False)

    self._confirm(self._experimental_toggle, on_result)
