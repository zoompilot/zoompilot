"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from collections.abc import Callable


def read_scaled_param(params, param: str, float_param: bool, min_value: int) -> int:
  """Read a param in the picker's integer domain.

  Float params use the TICI control's x100 scaling. Rounding prevents binary float
  representation from reducing the value on each open and commit cycle.
  """
  val = params.get(param, return_default=True)
  try:
    return round(float(val) * (100 if float_param else 1)) if val is not None else min_value
  except (ValueError, TypeError):
    return min_value

from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.scroll_panel2 import ScrollState
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller import _Scroller as _BaseScroller


class _Scroller(_BaseScroller):
  """Skip snapping until items have been laid out after show_event."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._snap_ready = False

  def show_event(self):
    super().show_event()
    self._snap_ready = False

  def _get_scroll(self, visible_items, content_size):
    if self._snap_items and not self._snap_ready:
      # Item positions are stale on the first layout after show_event.
      self._snap_ready = True
      self.scroll_panel.update(self._rect, content_size)
      return self.scroll_panel.get_offset()
    return super()._get_scroll(visible_items, content_size)

try:
  from openpilot.common.params import Params
except ImportError:
  Params = None

ITEM_WIDTH = 100
ITEM_HEIGHT = 140
SCREEN_WIDTH = 536
TITLE_HEIGHT = 46
CAROUSEL_TOP = 36   # carousel overlaps title zone by 10px for tighter layout
CAROUSEL_HEIGHT = 180

# Precompute the selection-band gradient on first use.
BAND_COLOR = rl.Color(255, 255, 255, int(255 * 0.25))
BAND_TOP = CAROUSEL_TOP + 14
BAND_FADE_LEN = 40


def _lerp(dist, tiers, values):
  """Piecewise linear interpolation for small tier arrays."""
  i = min(int(dist), len(tiers) - 2)
  t = max(0.0, min(1.0, dist - tiers[i]))
  return values[i] + (values[i + 1] - values[i]) * t


class PickerItem(Widget):
  """A lightweight label widget for a single picker value."""

  _DIST_TIERS = [0, 1, 2, 3]
  _FONT_SIZES = [56, 40, 32, 28]   # center, adjacent, far, edge
  _ALPHAS = [0.9, 0.4, 0.2, 0.1]

  def __init__(self, raw_value: int, display_label: str, on_tap: Callable[[int], None] | None = None,
               item_width: int = ITEM_WIDTH):
    super().__init__()
    self.raw_value = raw_value
    self.display_label = display_label
    self._on_tap = on_tap
    self._item_width = item_width
    self.set_rect(rl.Rectangle(0, 0, item_width, ITEM_HEIGHT))

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    if self._on_tap:
      self._on_tap(self.raw_value)

  def _render(self, rect):
    # Measure distance from the viewport center in item widths.
    if self._parent_rect is not None:
      parent_center_x = self._parent_rect.x + self._parent_rect.width / 2
    else:
      parent_center_x = SCREEN_WIDTH / 2

    item_center_x = self._rect.x + self._rect.width / 2
    dist = abs(item_center_x - parent_center_x) / self._item_width

    font_size = int(_lerp(dist, self._DIST_TIERS, self._FONT_SIZES))
    alpha = _lerp(dist, self._DIST_TIERS, self._ALPHAS)
    font_weight = FontWeight.BOLD if dist < 0.5 else FontWeight.ROMAN

    font = gui_app.font(font_weight)
    color = rl.Color(255, 255, 255, int(255 * alpha))
    padding = 8
    max_width = self._rect.width - padding

    if '\n' in self.display_label:
      lines = self.display_label.split('\n', 1)

      # Some text labels exceed the item width at the default font size.
      main_size = measure_text_cached(font, lines[0], font_size)
      if main_size.x > max_width and main_size.x > 0:
        font_size = max(int(font_size * max_width / main_size.x), 14)
        main_size = measure_text_cached(font, lines[0], font_size)

      sub_font_size = max(int(font_size * 0.42), 14)
      sub_font = gui_app.font(FontWeight.ROMAN)
      sub_size = measure_text_cached(sub_font, lines[1], sub_font_size)

      # Keep the main line aligned with single-line items.
      main_y = self._rect.y + (self._rect.height - main_size.y) / 2
      main_x = self._rect.x + (self._rect.width - main_size.x) / 2
      rl.draw_text_ex(font, lines[0], rl.Vector2(main_x, main_y), font_size, 0, color)

      gap = -2
      sub_color = rl.Color(255, 255, 255, int(255 * alpha * 0.55))
      sub_x = self._rect.x + (self._rect.width - sub_size.x) / 2
      rl.draw_text_ex(sub_font, lines[1], rl.Vector2(sub_x, main_y + main_size.y + gap), sub_font_size, 0, sub_color)
    else:
      # Some text labels exceed the item width at the default font size.
      text_size = measure_text_cached(font, self.display_label, font_size)
      if text_size.x > max_width and text_size.x > 0:
        font_size = max(int(font_size * max_width / text_size.x), 14)
        text_size = measure_text_cached(font, self.display_label, font_size)
      text_x = self._rect.x + (self._rect.width - text_size.x) / 2
      text_y = self._rect.y + (self._rect.height - text_size.y) / 2
      rl.draw_text_ex(font, self.display_label, rl.Vector2(text_x, text_y), font_size, 0, color)


def _build_band_texture():
  """Build a two-pixel vertical gradient strip for a selection-band edge."""
  band_h = CAROUSEL_HEIGHT - 14 * 2
  fade = min(BAND_FADE_LEN, band_h // 3)
  img = rl.gen_image_color(2, band_h, rl.Color(0, 0, 0, 0))
  for y in range(band_h):
    if y < fade:
      a = int(BAND_COLOR.a * y / fade)
    elif y >= band_h - fade:
      a = int(BAND_COLOR.a * (band_h - 1 - y) / fade)
    else:
      a = BAND_COLOR.a
    c = rl.Color(BAND_COLOR.r, BAND_COLOR.g, BAND_COLOR.b, a)
    rl.image_draw_rectangle(img, 0, y, 2, 1, c)
  tex = rl.load_texture_from_image(img)
  rl.unload_image(img)
  return tex


# Texture creation requires an active OpenGL context.
_band_texture = None


def _get_band_texture():
  global _band_texture
  if _band_texture is None:
    _band_texture = _build_band_texture()
  return _band_texture


class NumberPickerScreen(Widget):
  """Full picker sub-screen with title and horizontal snap-scrolling carousel."""

  def __init__(self, title: str, param: str, min_value: int, max_value: int,
               step: int = 1, label_callback: Callable | None = None,
               value_map: dict[int, int] | None = None, float_param: bool = False,
               unit: str | Callable[[], str] = "", item_width: int = ITEM_WIDTH):
    super().__init__()
    self._title = title
    self._param = param
    self._min_value = min_value
    self._float_param = float_param
    self._unit = unit
    self._item_width = item_width
    assert step > 0, "step must be positive"
    self._params = Params()
    self._last_center_value: int | None = None
    self._was_settled = True

    self._picker_items: list[PickerItem] = []
    val = min_value
    while val <= max_value:
      display_val = value_map[val] if value_map and val in value_map else val
      display = label_callback(display_val) if label_callback else str(display_val)
      self._picker_items.append(PickerItem(val, display, on_tap=self._on_item_tap, item_width=item_width))
      val += step

    pad = (SCREEN_WIDTH - item_width) // 2
    self._scroller = self._child(_Scroller(
      self._picker_items,
      horizontal=True,
      snap_items=True,
      scroll_indicator=False,
      pad=pad,
      spacing=0,
      edge_shadows=False,
    ))
    self._scroller.set_reset_scroll_at_show(False)

    self.set_rect(rl.Rectangle(0, 0, SCREEN_WIDTH, TITLE_HEIGHT + CAROUSEL_HEIGHT))

  @property
  def _scroll_panel(self):
    return self._scroller.scroll_panel

  def _center_index(self) -> int:
    """Find which item is between the selection bars using actual layout positions."""
    center_x = self._scroller.rect.x + self._scroller.rect.width / 2
    closest_idx = 0
    closest_dist = float('inf')
    for idx, item in enumerate(self._picker_items):
      dist = abs(item.rect.x + item.rect.width / 2 - center_x)
      if dist < closest_dist:
        closest_dist = dist
        closest_idx = idx
    return closest_idx

  def _scroll_to_index(self, idx: int):
    """Scroll so item at idx is centered."""
    self._scroll_panel.set_offset(-(idx * self._item_width))

  def _on_item_tap(self, raw_value: int):
    for idx, item in enumerate(self._picker_items):
      if item.raw_value == raw_value:
        self._scroll_to_index(idx)
        self._commit_value()
        break

  def _read_value(self) -> int:
    return read_scaled_param(self._params, self._param, self._float_param, self._min_value)

  def show_event(self):
    super().show_event()  # propagates to _scroller via _child()
    current = self._read_value()
    self._last_center_value = current
    for idx, item in enumerate(self._picker_items):
      if item.raw_value == current:
        self._scroll_to_index(idx)
        break

  def _commit_value(self):
    """Write the current center item's value to params."""
    if not self._picker_items:
      return
    center = self._picker_items[self._center_index()]
    if center.raw_value != self._last_center_value:
      self._last_center_value = center.raw_value
      # Params.put is non-blocking by default.
      self._params.put(self._param, center.raw_value / 100.0 if self._float_param else center.raw_value)

  def _update_state(self):
    super()._update_state()
    settled = self._scroll_panel.state == ScrollState.STEADY
    if settled and not self._was_settled:
      self._commit_value()
    self._was_settled = settled

  def _render(self, rect):
    font = gui_app.font(FontWeight.BOLD)
    title_size = measure_text_cached(font, self._title, 36)
    title_x = rect.x + (rect.width - title_size.x) / 2
    title_y = rect.y + (TITLE_HEIGHT - title_size.y) / 2
    rl.draw_text_ex(font, self._title, rl.Vector2(title_x, title_y), 36, 0,
                    rl.Color(255, 255, 255, int(255 * 0.9)))

    carousel_rect = rl.Rectangle(rect.x, rect.y + CAROUSEL_TOP, SCREEN_WIDTH, CAROUSEL_HEIGHT)
    self._scroller.render(carousel_rect)

    # Hide units for options represented by non-numeric labels.
    center = self._picker_items[self._center_index()] if self._picker_items else None
    unit_text = self._unit() if not isinstance(self._unit, str) else self._unit
    if unit_text and center is not None:
      try:
        float(center.display_label.replace('\n', ''))
      except ValueError:
        unit_text = ""

    band_tex = _get_band_texture()
    band_left = rect.x + (SCREEN_WIDTH - self._item_width) / 2 - 1
    band_right = band_left + self._item_width
    for x in (band_left, band_right):
      rl.draw_texture_ex(band_tex, (x, rect.y + BAND_TOP), 0, 1.0, rl.WHITE)

    if unit_text:
      unit_font = gui_app.font(FontWeight.BOLD)
      unit_font_size = 32
      unit_color = rl.Color(255, 255, 255, int(255 * 0.5))
      unit_size = measure_text_cached(unit_font, unit_text, unit_font_size)
      unit_x = rect.x + (SCREEN_WIDTH - unit_size.x) / 2
      unit_y = rect.y + rect.height - unit_size.y - 2
      rl.draw_text_ex(unit_font, unit_text, rl.Vector2(unit_x, unit_y), unit_font_size, 0, unit_color)
