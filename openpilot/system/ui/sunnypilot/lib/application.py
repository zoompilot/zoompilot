"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os

import numpy as np
import raylib_nb as rl

SHOW_MOUSE_COORDS = os.getenv("SHOW_MOUSE_COORDS") == "1"
SUNNYPILOT_UI = os.getenv("SUNNYPILOT_UI", "1") == "1"


class GuiApplicationExt:
  def __init__(self):
    self._show_mouse_coords = SHOW_MOUSE_COORDS

  @staticmethod
  def sunnypilot_ui() -> bool:
    return SUNNYPILOT_UI

  def _draw_mouse_coordinates(self, font):
    mouse_pos = rl.get_mouse_position()
    coords_text = f"X:{int(mouse_pos.x)}, Y:{int(mouse_pos.y)}"

    green_color = rl.Color(0, 159, 47, 255)  # Match the green color of FPS counter

    # Calculate text width to position it at the right edge; estimate width based on text length
    # Each character is approximately 10-12 pixels wide at font size 20
    estimated_text_width = len(coords_text) * 11

    # Position text at the top right corner, 10px from the top
    screen_width = self._scaled_width if self._scale != 1.0 else self._width
    text_pos = rl.Vector2(screen_width - estimated_text_width - 10, 6)

    # Draw the text
    rl.draw_text_ex(font, coords_text, text_pos, 20, 0, green_color)

  def set_show_mouse_coords(self, show: bool):
    self._show_mouse_coords = show

  def grayscale_texture(self, asset_path: str, width: int, height: int) -> rl.Texture:
    """texture(), in greyscale with its alpha kept: a control that is on but greyed out."""
    cache_key = f"grayscale:{asset_path}_{width}_{height}"
    if cache_key in self._textures:
      return self._textures[cache_key]

    from importlib.resources import as_file
    from openpilot.system.ui.lib.application import ASSETS_DIR
    with as_file(ASSETS_DIR.joinpath(asset_path)) as fspath:
      image = self._load_image_from_path(fspath.as_posix(), width, height)
    # raylib's ImageColorGrayscale weights, alpha kept
    assert image.format == rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, f"{asset_path}: expected RGBA8"
    w, h = image.width, image.height
    rgba = np.frombuffer(image.to_bytes(w * h * 4), dtype=np.uint8).reshape(h, w, 4).copy()
    rgba[..., :3] = (rgba[..., :3] @ np.array([0.299, 0.587, 0.114]))[..., None].astype(np.uint8)
    texture = self._load_texture_from_image(image)
    rl.update_texture(texture, rgba)
    if self._scale != 1.0:
      texture.width, texture.height = width, height   # logical size, as texture() sets it

    self._textures[cache_key] = texture
    return texture
