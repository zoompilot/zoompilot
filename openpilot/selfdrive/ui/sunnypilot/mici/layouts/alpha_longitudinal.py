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
from openpilot.selfdrive.ui.mici.widgets.button import BigToggle
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


class AlphaLongitudinalLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._alpha_long_toggle = BigToggle(tr("alpha longitudinal"), toggle_callback=self._on_alpha_long,
                                        description=tr("Lets zoompilot control the gas and brakes instead of stock cruise.\n" +
                                                       "This may turn off Automatic Emergency Braking (AEB)."))
    # upstream's row (TogglesLayoutMici), text and icon
    self._experimental_toggle = BigToggle(tr("experimental mode"), toggle_callback=self._on_experimental_mode,
                                          description_icon=gui_app.texture("icons_mici/experimental_mode.png", 64, 64),
                                          description=tr("Let the driving model control gas and brakes.\n" +
                                                         "Includes stopping for red lights and stop signs.\n" +
                                                         "Set speed is a maximum, not a target.\n" +
                                                         "These are alpha features. Expect mistakes.\n" +
                                                         "The path colors show acceleration and braking."))
    self._dec_toggle = BigParamControlSP(tr("dynamic experimental control"), "DynamicExperimentalControl",
                                         description=tr("Switches between chill and Experimental Mode on its own.\n" +
                                                        "Experimental for stops and slowdowns ahead, chill otherwise."))
    self._set_speed_toggle = BigParamControlSP(tr("speed assist"), "ExperimentalModeSetSpeed",
                                               description=tr("Experimental Mode often cruises below your set speed.\n" +
                                                              "It gently raises your speed toward it when the road ahead is clear.\n" +
                                                              "Does nothing while Dynamic Experimental Control is on."))
    self._lead_gap_toggle = BigParamControlSP(tr("lead follow assist"), "ExperimentalModeLeadGap",
                                              description=tr("Experimental Mode often follows farther back than your following distance.\n" +
                                                             "It closes the gap behind a steady car, never closer than chill mode.\n" +
                                                             "Does nothing while Dynamic Experimental Control is on."))

    self._refresh_toggles = (
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ExperimentalMode", self._experimental_toggle),
      ("ExperimentalModeSetSpeed", self._set_speed_toggle),
      ("ExperimentalModeLeadGap", self._lead_gap_toggle),
      ("DynamicExperimentalControl", self._dec_toggle),
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
    # these only act in experimental mode: locked without it, keeping their values
    idle = not has_long or not self._experimental_toggle._checked
    self._dec_toggle.set_superseded(idle)
    # DEC decides the mode on its own, and the e2e assists never act under it
    for toggle in (self._set_speed_toggle, self._lead_gap_toggle):
      toggle.set_superseded(idle or self._dec_toggle._checked)

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
