"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.system.ui.lib.application import gui_app

# a glow travelling around the zoompilot logo, prebuilt as a square sprite-sheet atlas
ASSET = "../../sunnypilot/selfdrive/assets/images/spinner_zoompilot_glow.png"
FRAMES = 75
GRID = 9
FPS = 30.0


class GlowLogo:
  """The boot spinner's logo. The frame comes from the global clock, so the loop keeps no
  per-frame state and skips frames rather than slowing when the UI drops one."""

  def __init__(self):
    self._atlas = gui_app.texture(ASSET)
    self._cell = self._atlas.width // GRID

  def draw(self, center: rl.Vector2, size: float) -> None:
    frame = int(rl.get_time() * FPS) % FRAMES
    src = rl.Rectangle((frame % GRID) * self._cell, (frame // GRID) * self._cell, self._cell, self._cell)
    dst = rl.Rectangle(center.x - size / 2.0, center.y - size / 2.0, size, size)
    rl.draw_texture_pro(self._atlas, src, dst, rl.Vector2(0, 0), 0.0, rl.WHITE)
