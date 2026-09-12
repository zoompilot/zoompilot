"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


# Only params the MICI onroad path actually consumes. BlindSpot + RainbowMode render on MICI;
# GreenLightAlert + LeadDepartAlert gate alert emission in the longitudinal planner (controls-side).
# Toggles that only drove the TICI onroad renderers (TorqueBar, StandstillTimer, RoadNameToggle,
# TrueVEgoUI, HideVEgoUI, ShowTurnSignals, RocketFuel, ChevronInfo, DevUIInfo) were no-ops here and
# were removed.
TOGGLE_PARAMS = [
  (tr("blind spot"), "BlindSpot"),
  (tr("rainbow mode"), "RainbowMode"),
  (tr("green light alert"), "GreenLightAlert"),
  (tr("lead depart alert"), "LeadDepartAlert"),
]


class VisualsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._toggles: dict[str, BigParamControl] = {}
    items = []
    for label, param in TOGGLE_PARAMS:
      toggle = BigParamControl(label, param)
      self._toggles[param] = toggle
      items.append(toggle)

    self._scroller.add_widgets(items)

  def _update_state(self):
    super()._update_state()
    for _param, toggle in self._toggles.items():
      toggle.refresh()
