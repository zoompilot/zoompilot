"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise > alpha longitudinal: the alpha switch and the modes that run on openpilot longitudinal
control. Developer drops its copy of the alpha switch (DeveloperLayoutMiciSP) so it appears once.
"""
from openpilot.selfdrive.ui.mici.layouts.settings.developer import AlphaLongConfirmPage
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
    self._set_speed_toggle = BigParamControlSP(tr("set speed assist"), "ExperimentalModeSetSpeed",
                                               description=tr("The driving model often cruises below your set speed and follows farther back.\n" +
                                                              "This drives toward them whenever the model holds back.\n" +
                                                              "The model still makes every slowdown.\n" +
                                                              "Long press home or hold the distance button to switch it."))

    self._refresh_toggles = (
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ExperimentalModeSetSpeed", self._set_speed_toggle),
    )
    self._scroller.add_widgets([item for _, item in self._refresh_toggles])

    # onroad too, for devices powered only with the car on; never mid-engagement
    self._alpha_long_toggle.set_enabled(lambda: not ui_state.engaged)
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
    # only acts on openpilot longitudinal control: locked without it, keeping its value
    self._set_speed_toggle.set_superseded(not has_long)

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
    ui_state.set_alpha_longitudinal(state)
    self._alpha_long_toggle.set_checked(state)
