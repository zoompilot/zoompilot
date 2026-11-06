"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mici's driver monitor (the dmoji) on tizi, at twice its size, in the bottom corner on the driver's
side. Its behaviour is mici's unchanged: hidden under alerts, and while disengaged unless AlwaysOnDM.
"""
from __future__ import annotations

import raylib_nb as rl

from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.mici.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus

SIZE = 2 * DriverStateRenderer.BASE_SIZE


def driver_monitor_rect(rect: rl.Rectangle, is_rhd: bool) -> rl.Rectangle:
  """Where the monitor sits in the onroad content rect; the long indicator stacks on it."""
  x = rect.x + (rect.width - UI_BORDER_SIZE - SIZE if is_rhd else UI_BORDER_SIZE)
  y = rect.y + rect.height - UI_BORDER_SIZE - SIZE - get_bottom_dev_ui_offset()
  return rl.Rectangle(x, y, SIZE, SIZE)


class DriverStateRendererSP(DriverStateRenderer):
  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, SIZE, SIZE))
    self.load_icons()  # sized by the rect

  def render(self, rect: rl.Rectangle | None = None):
    # upstream's tizi view hands over the whole content rect; mici's renderer draws into its own
    assert rect is not None
    self.set_should_draw(ui_state.status != UIStatus.DISENGAGED or ui_state.always_on_dm)
    return super().render(driver_monitor_rect(rect, self.is_rhd))
