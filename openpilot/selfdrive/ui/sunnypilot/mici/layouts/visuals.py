"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller


# Only params the MICI onroad path actually consumes. BlindSpot + RainbowMode render on MICI;
# GreenLightAlert + LeadDepartAlert gate alert emission in the longitudinal planner (controls-side).
# Toggles that only drove the TICI onroad renderers (TorqueBar, StandstillTimer, RoadNameToggle,
# TrueVEgoUI, HideVEgoUI, ShowTurnSignals, RocketFuel, ChevronInfo, DevUIInfo) were no-ops here and
# were removed.
TOGGLE_PARAMS = [
  (tr("blind spot"), "BlindSpot", tr("Shows a warning on screen when a car is in your blind spot.")),
  (tr("rainbow mode"), "RainbowMode", tr("Draws the driving path as a rainbow.\n" +
                                         "It does not change how the car drives.")),
  (tr("green light alert"), "GreenLightAlert", tr("Chimes when the light you're stopped at turns green with no car ahead.\n" +
                                                  "Only while cruise is not engaged. Check the road before you go.")),
  (tr("lead depart alert"), "LeadDepartAlert", tr("Chimes when you're stopped and the car ahead pulls away.\n" +
                                                  "Only while cruise is not engaged. Check the road before you go.")),
]


class VisualsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._toggles: dict[str, BigParamControlSP] = {}
    items = []
    for label, param, description in TOGGLE_PARAMS:
      toggle = BigParamControlSP(label, param, description=description)
      self._toggles[param] = toggle
      items.append(toggle)

    # the indicators need the car's blind spot monitor
    self._toggles["BlindSpot"].set_superseded(lambda: ui_state.CP is not None and not ui_state.CP.enableBsm)
    self._scroller.add_widgets(items)

  def _update_state(self):
    super()._update_state()
    for _param, toggle in self._toggles.items():
      toggle.refresh()
