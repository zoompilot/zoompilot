"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The big onroad view's bottom fade. The UI renders on the CPU, so the fade is a gradient over the
# lower half of the view; stretching onroad_fade.png over the whole view took the comma 3X past its
# frame time whenever the car was engaged, and the UI then starved radard and plannerd on their core.

import os
os.environ["BIG"] = "1"  # the 3X's view: the cost is by the pixel

import time
from types import SimpleNamespace

import numpy as np
import pytest
import raylib_nb as rl
from PIL import Image

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.onroad import augmented_road_view as arv
from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.lib import polygon
from openpilot.system.ui.lib.application import ASSETS_DIR

BG = 200  # grey the fade is drawn over
MARGIN = 30  # the border the content rect sits inside


def _view(alpha: float = 1.0):
  return SimpleNamespace(_fade_alpha_filter=FirstOrderFilter(alpha, 0.1, 0.05))


class _Frame:
  """A frame drawn the way the device draws one: a point is a framebuffer pixel. A PC's window is
  scaled, and a HiDPI one doubled again, while a gradient is laid out in framebuffer pixels from
  gui_app.height, so this undoes the window's scale and gives the gradient the framebuffer's height."""

  def __init__(self, gui, monkeypatch, background: rl.Color):
    self._background = background
    self.height, self.width = rl.cpu.screen().shape[:2]
    self._window_scale = self.width / gui._scaled_width  # 2 on a HiDPI display
    monkeypatch.setattr(polygon, 'gui_app', SimpleNamespace(height=self.height))
    monkeypatch.setattr(arv, 'gui_app', SimpleNamespace(height=self.height))
    self.content_rect = rl.Rectangle(MARGIN, MARGIN, self.width - 2 * MARGIN, self.height - 2 * MARGIN)

  def __enter__(self):
    rl.begin_drawing()
    rl.clear_background(self._background)
    rl.rl_push_matrix()
    rl.rl_scalef(1 / self._window_scale, 1 / self._window_scale, 1.0)
    return self

  def __exit__(self, *exc):
    rl.rl_pop_matrix()
    rl.end_drawing()


def _draw(gui, monkeypatch, status=UIStatus.ENGAGED, torque_bar=True, alpha: float = 1.0) -> np.ndarray:
  """One frame of the fade over grey: the content rect's red channel, top row first."""
  monkeypatch.setattr(ui_state, 'status', status)
  monkeypatch.setattr(ui_state, 'torque_bar', torque_bar)
  with _Frame(gui, monkeypatch, rl.Color(BG, BG, BG, 255)) as frame:
    arv.AugmentedRoadViewSP.update_fade_out_bottom_overlay(_view(alpha), frame.content_rect)
  screen = rl.cpu.screen()[::-1]  # its rows are bottom up
  assert not (screen[:MARGIN, :, 0] != BG).any() and not (screen[-MARGIN:, :, 0] != BG).any(), "drew outside the content rect"
  assert not (screen[:, :MARGIN, 0] != BG).any() and not (screen[:, -MARGIN:, 0] != BG).any(), "drew outside the content rect"
  return screen[MARGIN:-MARGIN, MARGIN:-MARGIN, 0].astype(float)


class TestBottomFade:
  def test_darkens_only_the_lower_part(self, gui, monkeypatch):
    px = _draw(gui, monkeypatch)
    rows = px.mean(axis=1)
    clear = int(len(rows) * (1 - arv.FADE_HEIGHT)) - 1
    assert (px[:clear] == BG).all()
    assert (np.diff(rows[clear:]) <= 0.5).all(), "darkens toward the bottom"
    # each stop where it belongs: a gradient laid out upside down or off the rect fills it one color
    for stop, alpha in zip(arv.FADE_STOPS, arv.FADE_ALPHAS, strict=True):
      row = round((len(rows) - 1) * (1 - arv.FADE_HEIGHT * stop))
      assert rows[row] == pytest.approx(BG * (1 - alpha / 255), abs=5), f"stop {stop}"
    assert np.ptp(px, axis=1).max() <= 1, "the same across the view"

  @pytest.mark.parametrize("status, torque_bar", [(UIStatus.DISENGAGED, True), (UIStatus.ENGAGED, False)])
  def test_not_drawn(self, gui, monkeypatch, status, torque_bar):
    assert (_draw(gui, monkeypatch, status, torque_bar, alpha=0.0) == BG).all()

  def test_fades_in(self, gui, monkeypatch):
    full = _draw(gui, monkeypatch)[-1].mean()
    half = _draw(gui, monkeypatch, alpha=0.5)[-1].mean()  # the filter moves it a little toward 1
    assert full < half < BG

  def test_follows_the_texture(self):
    # the look it replaces: the texture's mean alpha by row, over its lower FADE_HEIGHT
    alpha = np.array(Image.open(ASSETS_DIR / "icons_mici/onroad/onroad_fade.png").convert("RGBA"))[..., 3].mean(axis=1)
    lower = alpha[int(len(alpha) * (1 - arv.FADE_HEIGHT)):]
    t = np.linspace(1.0, 0.0, len(lower))  # 0 at the bottom edge, like FADE_STOPS
    assert np.abs(np.interp(t, arv.FADE_STOPS, arv.FADE_ALPHAS) - lower).max() < 12
    assert alpha[:len(alpha) - len(lower)].max() < 8, "nothing to see above the gradient"

  def test_cheaper_than_the_texture(self, gui, monkeypatch):
    if not gui.big_ui():  # another module's import chose the window first
      pytest.skip("timed at the 3X's size, where there are pixels enough to time")
    monkeypatch.setattr(ui_state, 'status', UIStatus.ENGAGED)
    monkeypatch.setattr(ui_state, 'torque_bar', True)
    texture = gui.texture("icons_mici/onroad/onroad_fade.png")
    view = _view()

    def stretched_texture(rect):
      rl.draw_texture_pro(texture, rl.Rectangle(0, 0, texture.width, texture.height), rect, rl.Vector2(0, 0), 0.0, rl.WHITE)

    def cost(draw) -> float:
      times = []
      for _ in range(7):
        start = time.perf_counter()
        with _Frame(gui, monkeypatch, rl.BLACK) as frame:  # a texture is only drawn as the frame ends
          draw(frame.content_rect)
        times.append(time.perf_counter() - start)
      return float(np.median(times))

    frame = cost(lambda rect: None)  # an empty frame
    gradient = cost(lambda rect: arv.AugmentedRoadViewSP.update_fade_out_bottom_overlay(view, rect)) - frame
    assert gradient * 4 < cost(stretched_texture) - frame
