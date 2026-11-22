"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The comma 4's camera view starts right of the side panel, so its overlays draw at an offset. The rainbow path
# skipped it and sat a side panel's width left of the lane lines.

import numpy as np
import pytest
import raylib_nb as rl

from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH, model_renderer
from openpilot.selfdrive.ui.onroad.model_renderer import ModelPoints
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath
from openpilot.selfdrive.ui.ui_state import ui_state

PATH = np.array([[100, 200], [110, 100], [150, 100], [160, 200]], dtype=np.float32)


@pytest.fixture
def draw(monkeypatch):
  renderer = object.__new__(model_renderer.ModelRenderer)  # _draw_path alone, without a window
  renderer._rect = rl.Rectangle(SIDE_PANEL_WIDTH, 0, 464, 240)
  renderer._path = ModelPoints(projected_points=PATH)
  renderer.rainbow_path = RainbowPath()
  renderer._longitudinal_control = False
  calls = []
  monkeypatch.setattr(model_renderer, 'draw_polygon', lambda rect, points, color=None, gradient=None: calls.append((np.array(points), gradient)))

  def _draw(rainbow: bool, visible: float = 1.0):
    monkeypatch.setattr(ui_state, 'rainbow_path', rainbow)
    renderer._visible = visible
    calls.clear()
    renderer._draw_path()
    assert len(calls) == 1
    return calls[0]
  return _draw


def test_rainbow_path_draws_where_the_path_does(draw):
  points, _ = draw(rainbow=True)
  np.testing.assert_array_equal(points, draw(rainbow=False)[0])
  np.testing.assert_array_equal(points, PATH + (SIDE_PANEL_WIDTH, 0))


def test_rainbow_path_fades_with_the_path(draw):
  full, half = draw(rainbow=True)[1], draw(rainbow=True, visible=0.5)[1]
  assert [c.a for c in half.colors] == [round(c.a * 0.5) for c in full.colors]
