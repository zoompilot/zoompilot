"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The home footer's longitudinal marks: alpha longitudinal as upstream's onroad car over three bars,
tightened to the footer's 40 px, and Set Speed Assist as the onroad dial beside it, white like the
footer's other icons.
"""
import raylib_nb as rl

from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_dial import DIAL_HEIGHT, DIAL_WIDTH, draw_dial
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

STATUS_ICON_HEIGHT = 40  # upstream's footer icons (network, chestnut, usb) are 40 px of ink
STATUS_ICON_ALPHA = 0.9  # settings and network draw at 90%
HOME_DIAL_SCALE = 1.25  # 44 px wide and 26 tall, centred like the gear and network icons beside it


class SpeedAssistIcon(Widget):
  """The dial, white, centred in a footer icon's 40 px."""
  def __init__(self, height: int = STATUS_ICON_HEIGHT):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, round(DIAL_WIDTH * HOME_DIAL_SCALE), float(height)))
    self.set_enabled(False)

  def _render(self, _) -> None:
    white = rl.Color(255, 255, 255, round(255 * STATUS_ICON_ALPHA))
    top = self._rect.y + (self._rect.height - DIAL_HEIGHT * HOME_DIAL_SCALE) / 2
    draw_dial(self._rect.x + self._rect.width / 2, top, HOME_DIAL_SCALE, track=white, needle=white)


# alpha longitudinal: upstream's onroad car over three bars (car 35 wide, bars 32/40/48), on whole
# pixels for 40 px: car 25x19, bars 23x3 / 29x4 / 33x5, 3 px apart
LONG_CAR = (25, 19)
LONG_BARS = ((23, 3), (29, 4), (33, 5))
LONG_GAP = 3


class LongitudinalIcon(Widget):
  """Alpha longitudinal on: the car and three bars the onroad view shows while it drives."""
  def __init__(self):
    super().__init__()
    self._car = gui_app.texture('icons_mici/longitudinal/car.png', *LONG_CAR, keep_aspect_ratio=False)
    self._bars = [gui_app.texture(f'icons_mici/longitudinal/distance_{i + 1}.png', w, h, keep_aspect_ratio=False)
                  for i, (w, h) in enumerate(LONG_BARS)]
    width = max(w for w, _ in (LONG_CAR, *LONG_BARS))
    height = LONG_CAR[1] + sum(h + LONG_GAP for _, h in LONG_BARS)
    self.set_rect(rl.Rectangle(0, 0, width, height))
    self.set_enabled(False)

  def _render(self, _) -> None:
    color = rl.fade(rl.WHITE, STATUS_ICON_ALPHA)
    x, y = self._rect.x, self._rect.y
    for texture in (self._car, *self._bars):
      # whole pixels: the bars are a few px tall and blur on a half-pixel offset
      rl.draw_texture_ex(texture, rl.Vector2(round(x + (self._rect.width - texture.width) / 2), round(y)), 0.0, 1.0, color)
      y += texture.height + LONG_GAP
