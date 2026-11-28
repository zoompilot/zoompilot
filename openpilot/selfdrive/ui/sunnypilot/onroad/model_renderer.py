"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful_zp import BlendfulRendererZP
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.system.ui.lib.application import gui_app


class ModelRendererSP:
  def __init__(self):
    self.rainbow_path = RainbowPath()
    self.chevron_metrics = ChevronMetrics()
    self.blendful = BlendfulRendererZP()
    self._width_filter = FirstOrderFilter(0.9, 0.1, 1 / gui_app.target_fps)

  # Upstream assigns carParams' openpilotLongitudinalControl. Read back, it also waits for a stock
  # ECU takeover (Mazda's radar): no lead chevron or acceleration colors on a car still on stock.
  @property
  def _longitudinal_control(self) -> bool:
    return self._cp_longitudinal_control and ui_state.stock_ecu_owned

  @_longitudinal_control.setter
  def _longitudinal_control(self, value: bool) -> None:
    self._cp_longitudinal_control = value

  @property
  def _lateral_active(self) -> bool:
    return UIStateSP.lateral_active(ui_state)

  def _get_path_half_width(self) -> float:
    target = 0.9 if self._lateral_active else 0.40
    return self._width_filter.update(target)
