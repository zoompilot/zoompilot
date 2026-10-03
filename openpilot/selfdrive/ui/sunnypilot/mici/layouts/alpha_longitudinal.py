"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise > alpha longitudinal: the alpha switch and the modes that run on openpilot longitudinal
control. Toggles and Developer drop their copies of the first two (TogglesLayoutMiciSP,
DeveloperLayoutMiciSP) so each switch appears once.
"""
from openpilot.selfdrive.ui.mici.layouts.settings.developer import AlphaLongConfirmPage
from openpilot.selfdrive.ui.mici.layouts.settings.toggles import ExperimentalModeConfirmPage
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigToggle
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class AlphaLongitudinalLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._alpha_long_toggle = BigToggle(tr("alpha longitudinal"), toggle_callback=self._on_alpha_long)
    self._experimental_toggle = BigToggle(tr("experimental mode"), toggle_callback=self._on_experimental_mode)
    self._dec_toggle = BigParamControl(tr("dynamic experimental control"), "DynamicExperimentalControl")
    self._set_speed_toggle = BigParamControl(tr("speed assist"), "ExperimentalModeSetSpeed")

    self._refresh_toggles = (
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ExperimentalMode", self._experimental_toggle),
      ("DynamicExperimentalControl", self._dec_toggle),
      ("ExperimentalModeSetSpeed", self._set_speed_toggle),
    )
    self._scroller.add_widgets([item for _, item in self._refresh_toggles])

    self._alpha_long_toggle.set_enabled(ui_state.is_offroad)
    ui_state.add_offroad_transition_callback(self._refresh)
    ui_state.add_engaged_transition_callback(self._refresh)

  def show_event(self):
    super().show_event()
    self._refresh()

  def _update_state(self):
    super()._update_state()

    CP = ui_state.CP
    has_long = CP is not None and ui_state.has_longitudinal_control
    self._alpha_long_toggle.set_visible(CP is not None and CP.alphaLongitudinalAvailable and not ui_state.is_release)
    self._experimental_toggle.set_enabled(CP is None or has_long)
    self._dec_toggle.set_enabled(has_long)
    # DEC decides the mode on its own, and the set-speed floor never acts under it
    self._set_speed_toggle.set_enabled(has_long and not self._dec_toggle._checked)

  def _refresh(self):
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

  def _on_alpha_long(self, state: bool):
    if state:
      # stays off until confirmed
      self._alpha_long_toggle.set_checked(False)
      gui_app.push_widget(AlphaLongConfirmPage(lambda: self._set_alpha_long(True)))
    else:
      self._set_alpha_long(False)

  def _set_alpha_long(self, state: bool):
    # param only: card watches for the change and requests the onroad cycle itself, after any
    # radar hand-back the brand needs
    ui_state.params.put_bool("AlphaLongitudinalEnabled", state, block=True)
    self._alpha_long_toggle.set_checked(state)

  def _on_experimental_mode(self, state: bool):
    if state and not ui_state.params.get_bool("ExperimentalModeConfirmed"):
      self._experimental_toggle.set_checked(False)

      def on_confirm():
        ui_state.params.put_bool("ExperimentalModeConfirmed", True)
        ui_state.params.put_bool("ExperimentalMode", True)
        self._experimental_toggle.set_checked(True)

      gui_app.push_widget(ExperimentalModeConfirmPage(on_confirm))
    else:
      ui_state.params.put_bool("ExperimentalMode", state)
