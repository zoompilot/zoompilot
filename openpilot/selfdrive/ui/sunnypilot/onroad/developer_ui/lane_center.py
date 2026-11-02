"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful_zp import BLUE, nudge_share, pull_at_lookahead
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.elements import UiElement


class LaneCenterElement:
  """The 3X developer UI's lane center item: the readout box's number, the push at the lookahead in cm,
  smoothed the same way, with its side, and blue as the nudge grows."""
  LABEL, UNIT = "LANE CTR", "cm"

  def __init__(self):
    self._pull = FirstOrderFilter(0.0, 0.5, DT_MDL)

  def update(self, sm, is_metric: bool) -> UiElement:
    blendful = sm['modelDataV2SP']
    if blendful.blendfulTrust <= 0.0:
      return UiElement("-", self.LABEL, "", rl.WHITE)  # no unit on no number
    if sm.updated['modelDataV2SP']:
      self._pull.update(pull_at_lookahead(blendful.blendfulCurvature, sm['carState'].vEgo))
    pull = self._pull.x
    cm = round(abs(pull) * 100)
    side = "" if cm == 0 else "L " if pull < 0 else "R "
    return UiElement(f"{side}{cm}", self.LABEL, self.UNIT, rl.color_lerp(rl.WHITE, BLUE, nudge_share(pull)))
