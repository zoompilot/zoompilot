"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's curve speed planners (vision and map). On openpilot long they run on every brand:
the planner drives the cruise candidate, and the budget and actuation lead come from the planner
and CarParams, not from the car's stock ACC. On stock ACC they run on the brands listed here,
whose ACC response has been measured; every other brand keeps sunnypilot's own controllers,
wrapped only for the Curve Speed profile (curve_speed_profile.py).
See docs/zoompilot/scc-curve-planning.md.
"""

# brands whose stock ACC response and planning limits have been measured from their logs
TUNED_BRANDS = ('mazda',)


def runs_zoompilot_planners(CP) -> bool:
  return bool(CP.openpilotLongitudinalControl) or CP.brand in TUNED_BRANDS


def make_smart_cruise_control(CP):
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
  scc = SmartCruiseControl()
  if runs_zoompilot_planners(CP):
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.map_controller import SmartCruiseControlMap
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
    scc.vision = SmartCruiseControlVision(CP)
    scc.map = SmartCruiseControlMap(CP)
  else:
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import curve_speed_profile
    scc.vision = curve_speed_profile.SmartCruiseControlVision()
    scc.map = curve_speed_profile.SmartCruiseControlMap()
  return scc
