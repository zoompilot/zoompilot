#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Render every mici settings screen to PNG for PR documentation.

Requires a display context (run from a regular terminal, not SSH).

Usage:
  cd /path/to/sunnypilot
  PYTHONPATH=. SCALE=1 .venv/bin/python openpilot/sunnypilot/selfdrive/ui/tests/screenshot_layouts.py
"""
import os

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")

import pyray as rl

from pathlib import Path
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix


OUTPUT_DIR = Path(__file__).parent / "screenshots"
SETTLE_FRAMES = 30


def setup_params():
  params = Params()

  cp = car.CarParams.new_message(
    enableBsm=True,
    brand="debug",
    openpilotLongitudinalControl=True,
    steerControlType="torque",
  )
  params.put("CarParamsPersistent", cp.to_bytes())

  cp_sp = custom.CarParamsSP.new_message(
    intelligentCruiseButtonManagementAvailable=True,
  )
  params.put("CarParamsSPPersistent", cp_sp.to_bytes())
  params.put_bool("IntelligentCruiseButtonManagement", True)

  # Visuals (only the toggles the MICI visuals layout renders)
  params.put_bool("BlindSpot", True)
  params.put_bool("RainbowMode", True)
  params.put_bool("GreenLightAlert", True)
  params.put_bool("LeadDepartAlert", True)

  # Display
  params.put("OnroadScreenOffBrightness", 10)
  params.put("OnroadScreenOffTimer", 2)
  params.put("InteractivityTimeout", 30)

  # Steering
  params.put_bool("Mads", True)
  params.put_bool("MadsMainCruiseAllowed", True)
  params.put_bool("MadsUnifiedEngagementMode", False)
  params.put("MadsSteeringMode", 1)
  params.put_bool("AutoLaneChangeTimer", True)
  params.put_bool("AutoLaneChangeBsmDelay", True)
  params.put_bool("BlinkerPauseLateralControl", True)
  params.put("BlinkerMinLateralControlSpeed", 25)
  params.put("BlinkerLateralReengageDelay", 3)
  params.put_bool("EnforceTorqueControl", True)
  params.put("TorqueControlTune", 1.0)
  params.put_bool("LiveTorqueParamsToggle", True)
  params.put_bool("LiveTorqueParamsRelaxedToggle", True)
  params.put_bool("CustomTorqueParams", True)
  params.put_bool("TorqueParamsOverrideEnabled", True)
  # The picker scales physical float values into its integer domain.
  params.put("TorqueParamsOverrideLatAccelFactor", 2.5)
  params.put("TorqueParamsOverrideFriction", 0.5)

  # Cruise
  params.put_bool("CustomAccIncrementsEnabled", True)
  params.put("CustomAccShortPressIncrement", 3)
  params.put("CustomAccLongPressIncrement", 2)
  params.put("SpeedLimitMode", 2)
  params.put("SpeedLimitPolicy", 1)
  params.put("SpeedLimitOffsetType", 1)
  params.put("SpeedLimitValueOffset", 5)

  # Trips
  params.put("ApiCache_DriveStats", {
    "all": {"routes": 142, "distance": 3456, "minutes": 8760},
    "week": {"routes": 7, "distance": 89, "minutes": 420},
  })


def setup_ui_state():
  from openpilot.selfdrive.ui.ui_state import ui_state

  ui_state.params = Params()
  ui_state.CP = car.CarParams.new_message(
    enableBsm=True,
    brand="debug",
    openpilotLongitudinalControl=True,
    steerControlType="torque",
  )
  ui_state.CP_SP = custom.CarParamsSP.new_message(
    intelligentCruiseButtonManagementAvailable=True,
  )
  ui_state.started = False
  ui_state.has_longitudinal_control = True
  ui_state.has_icbm = True
  ui_state.is_metric = False
  ui_state.is_sp_release = False


def capture(widget, filename, frames=SETTLE_FRAMES):
  from openpilot.system.ui.lib.application import gui_app

  rt = rl.load_render_texture(gui_app.width, gui_app.height)

  if hasattr(widget, '_trigger_animate_in'):
    widget._trigger_animate_in = False
  if hasattr(widget, '_pos_filter'):
    widget._pos_filter.x = 0.0

  # Keep alpha opaque: blend RGB normally, but force alpha to stay at dst (1.0 from clear)
  GL_FUNC_ADD = 0x8006
  rl.rl_set_blend_factors_separate(
    0x0302, 0x0303,  # RGB: GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA
    0x0000, 0x0001,  # Alpha: GL_ZERO, GL_ONE (preserves dst alpha = 1.0)
    GL_FUNC_ADD, GL_FUNC_ADD,
  )

  rect = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
  for _ in range(frames):
    rl.begin_texture_mode(rt)
    rl.clear_background(rl.BLACK)
    rl.begin_blend_mode(rl.BLEND_CUSTOM_SEPARATE)
    widget.render(rect)
    rl.end_blend_mode()
    rl.end_texture_mode()
    rl.begin_drawing()
    rl.end_drawing()

  image = rl.load_image_from_texture(rt.texture)
  rl.image_flip_vertical(image)

  OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
  path = str(OUTPUT_DIR / filename)
  rl.export_image(image, path)

  rl.unload_image(image)
  rl.unload_render_texture(rt)
  print(f"  {filename}")


def capture_sub_view(view, filename):
  """Capture a pre-built sub-view NavScroller directly."""
  view.show_event()
  capture(view, filename)


def capture_picker(option, filename):
  """Create and capture a picker screen for a BigParamOption."""
  from openpilot.system.ui.widgets.scroller import NavScroller

  picker = option.create_picker_screen()
  # NavScroller doesn't pass kwargs to _Scroller (NavWidget rejects them)
  view = NavScroller()
  view._scroller._show_scroll_indicator = False
  view._scroller._pad = 0
  view._scroller.add_widgets([picker])
  view._scroller.set_scrolling_enabled(False)
  view.show_event()
  capture(view, filename)


def capture_display():
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.display import DisplayLayoutMici

  d = DisplayLayoutMici()
  d.show_event()
  capture(d, "display.png")

  capture_picker(d._brightness, "display_brightness_picker.png")
  capture_picker(d._brightness_timer, "display_timer_picker.png")
  capture_picker(d._ui_timeout, "display_timeout_picker.png")


def capture_visuals():
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.visuals import VisualsLayoutMici

  v = VisualsLayoutMici()
  v.show_event()
  capture(v, "visuals.png")


def capture_cruise():
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.cruise import CruiseLayoutMici

  c = CruiseLayoutMici()
  c.show_event()
  capture(c, "cruise.png")

  # Custom ACC sub-panel
  capture_sub_view(c._acc_view, "cruise_custom_acc.png")
  capture_picker(c._acc_short, "cruise_acc_short_picker.png")
  capture_picker(c._acc_long, "cruise_acc_long_picker.png")

  # Speed limit sub-panel
  capture_sub_view(c._sl_view, "cruise_speed_limit.png")
  capture_picker(c._sl_offset_value, "cruise_sl_offset_picker.png")


def capture_steering():
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

  s = SteeringLayoutMici()
  s.show_event()
  capture(s, "steering.png")

  # MADS sub-panel
  capture_sub_view(s._mads_view, "steering_mads.png")

  # Lane change sub-panel
  capture_sub_view(s._lc_view, "steering_lane_change.png")

  # Blinker sub-panel
  capture_sub_view(s._blinker_view, "steering_blinker.png")
  capture_picker(s._blinker_speed, "steering_blinker_speed_picker.png")
  capture_picker(s._blinker_delay, "steering_blinker_delay_picker.png")

  # Torque sub-panel
  capture_sub_view(s._tq_view, "steering_torque.png")

  # Self-tune sub-panel (nested under torque)
  capture_sub_view(s._tq_self_tune_view, "steering_self_tune.png")

  # Custom tune sub-panel (nested under torque)
  capture_sub_view(s._tq_custom_view, "steering_custom_tune.png")
  capture_picker(s._tq_lat_accel, "steering_lat_accel_picker.png")
  capture_picker(s._tq_friction, "steering_friction_picker.png")


def capture_trips():
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.trips import TripsLayoutMici

  t = TripsLayoutMici()
  t.show_event()
  capture(t, "trips.png")


def main():
  with OpenpilotPrefix():
    setup_params()

    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)

    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("screenshot_layouts", fps=30)

    setup_ui_state()

    print("Capturing screenshots...")
    capture_display()
    capture_visuals()
    capture_cruise()
    capture_steering()
    capture_trips()

    gui_app.close()
    print(f"\nDone — {OUTPUT_DIR}/")


if __name__ == "__main__":
  main()
