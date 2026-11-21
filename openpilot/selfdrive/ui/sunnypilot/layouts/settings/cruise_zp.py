"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Curve Speed, for the tizi Cruise panel: how fast Slow for Curves takes turns, under its two toggles.
"""
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp


class CurveSpeed:
  """Not a panel: CruiseLayout lists `item` after the Slow for Curves toggles and drives update_state."""
  def __init__(self, scc_v_toggle, scc_m_toggle):
    self._scc_toggles = (scc_v_toggle, scc_m_toggle)
    # stored value is the index into curve_speed_profile.SCALES
    self.item = multiple_button_item_sp(
      title=lambda: tr("Curve Speed"),
      description=lambda: tr("How fast Slow for Curves takes turns. Gentle slows more, Sporty slows less."),
      buttons=[lambda: tr("Gentle"), lambda: tr("Normal"), lambda: tr("Sporty")],
      param="CurveSpeedProfile",
      button_width=280,
      inline=False)

  def update_state(self, cruise_available: bool):
    # nothing reads it with both curve features off
    scc_on = any(toggle.action_item.get_state() for toggle in self._scc_toggles)
    self.item.action_item.set_enabled(cruise_available and scc_on)
