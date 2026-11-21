"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful_zp import BlendfulRendererZP
from openpilot.selfdrive.ui.sunnypilot.onroad.rainbow_path import RainbowPath

# green lane lines mean openpilot is steering: not with only longitudinal engaged
class ModelRendererSP:
  def __init__(self):
    self.rainbow_path = RainbowPath()
    self.blendful = BlendfulRendererZP(rect_relative=True)
