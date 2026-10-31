"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The alpha switch and the modes that run on openpilot longitudinal control, at the top of the tizi
Cruise panel. Developer drops its copy of the alpha switch (DeveloperLayoutSP) so it appears once.
"""
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog

# upstream's warning (developer.py), with our name; its advice to turn on experimental mode is moot
# now the driving model drives the gas and brakes by default
ALPHA_LONGITUDINAL_DESCRIPTION = tr_noop(
  "<b>WARNING: zoompilot longitudinal control is in alpha for this car and may disable Automatic Emergency Braking (AEB).</b><br><br>" +
  "On this car, zoompilot defaults to the car's built-in ACC instead of zoompilot's longitudinal control. " +
  "Enable this to switch to zoompilot longitudinal control. " +
  "Changing this setting will restart zoompilot if the car is powered on."
)

# the opt-out from the driving model (UIStateZP.legacy_chill)
LEGACY_CHILL_DESCRIPTION = tr_noop(
  "The older way to drive the gas and brakes: hold your set speed and follow the car ahead. " +
  "It won't stop for red lights or stop signs.<br><br>" +
  "Off, the default, lets the driving model drive the gas and brakes as it thinks a human would, " +
  "including stopping for red lights and stop signs. The set speed is then a maximum."
)


class AlphaLongitudinalToggles:
  """Not a panel: CruiseLayout lists `items` in its own scroller and drives the two hooks."""
  def __init__(self):
    self._alpha_long_toggle = toggle_item_sp(
      title=lambda: tr("Alpha Longitudinal"),
      description=lambda: tr(ALPHA_LONGITUDINAL_DESCRIPTION),
      callback=self._on_alpha_long,
      enabled=lambda: not ui_state.engaged)

    self._set_speed_toggle = toggle_item_sp(
      title=lambda: tr("Set Speed Assist"),
      description=lambda: tr("Drive toward your set speed and following distance as legacy chill mode does " +
                             "whenever the driving model holds back. The model still makes every slowdown."),
      param="ExperimentalModeSetSpeed")

    self._chill_toggle = toggle_item_sp(
      title=lambda: tr("Legacy Chill Mode"),
      description=lambda: tr(LEGACY_CHILL_DESCRIPTION),
      callback=ui_state.set_legacy_chill)

    self._refresh_toggles = (
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ExperimentalModeSetSpeed", self._set_speed_toggle),
    )
    self.items = [item for _, item in self._refresh_toggles] + [self._chill_toggle]

    ui_state.add_offroad_transition_callback(self.refresh)
    ui_state.add_engaged_transition_callback(self.refresh)

  def update_state(self):
    CP = ui_state.CP
    has_long = CP is not None and ui_state.has_longitudinal_control
    self._alpha_long_toggle.set_visible(CP is not None and CP.alphaLongitudinalAvailable)
    self._chill_toggle.action_item.set_enabled(CP is None or has_long)
    # only acts on the driving model's gas and brakes: locked in chill, keeping its value
    self._set_speed_toggle.action_item.set_enabled(has_long and not self._chill_toggle.action_item.get_state())

  def refresh(self):
    for key, item in self._refresh_toggles:
      item.action_item.set_state(ui_state.params.get_bool(key))
    self._chill_toggle.action_item.set_state(ui_state.legacy_chill)

  @staticmethod
  def _confirm(item, callback) -> None:
    content = f"<h1>{item.title}</h1><br><p>{item.description}</p>"
    gui_app.push_widget(ConfirmDialog(content, tr("Enable"), rich=True, callback=callback))

  def _on_alpha_long(self, state: bool):
    if not state:
      ui_state.set_alpha_longitudinal(False)
      return

    def on_result(result: DialogResult):
      if result == DialogResult.CONFIRM:
        ui_state.set_alpha_longitudinal(True)
      else:
        self._alpha_long_toggle.action_item.set_state(False)

    self._confirm(self._alpha_long_toggle, on_result)
