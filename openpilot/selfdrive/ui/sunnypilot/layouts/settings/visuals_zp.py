"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane Center Gauge, for the tizi Visuals panel: lane center assist's gauge, beside the steering arc
it sits under.
"""
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp

LANE_CENTER_UI_DESCRIPTION = {
  "enabled": tr_noop("Shows lane center assist's nudge as a gauge under the steering arc."),
  "disabled": tr_noop("This feature requires Lane Center Assist, in Models, to be on.")
}


class LaneCenterUI:
  """Not a panel: VisualsLayout splices `item` into its list and drives update_state."""
  def __init__(self):
    self.item = toggle_item_sp(
      title=lambda: tr("Lane Center Gauge"),
      description="",
      param="LaneCenterUI",
      initial_state=ui_state.params.get_bool("LaneCenterUI"),
    )
    self._enabled: bool | None = None

  def splice(self, toggles: dict, items: list) -> list:
    """`items` with the row after the steering arc's toggle."""
    after_arc = items.index(toggles["TorqueBar"]) + 1
    return items[:after_arc] + [self.item] + items[after_arc:]

  def update_state(self):
    self.item.action_item.set_state(ui_state.params.get_bool("LaneCenterUI"))
    enabled = ui_state.lane_center
    if enabled != self._enabled:
      self._enabled = enabled
      desc = LANE_CENTER_UI_DESCRIPTION["enabled" if enabled else "disabled"]
      self.item.set_description(lambda: tr(desc))
      self.item.action_item.set_enabled(enabled)
