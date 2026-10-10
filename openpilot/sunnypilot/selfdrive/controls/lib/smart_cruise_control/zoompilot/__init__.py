"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's curve speed planners (vision and map), tuned from measurements of the brands
listed here. Every other brand runs sunnypilot's own controllers, unchanged.
See docs/zoompilot/scc-curve-planning.md.
"""

# brands whose stock ACC response and planning limits have been measured from their logs
TUNED_BRANDS = ('mazda',)


def make_smart_cruise_control(CP):
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
  scc = SmartCruiseControl()
  if CP.brand in TUNED_BRANDS:
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.map_controller import SmartCruiseControlMap
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
    scc.vision = SmartCruiseControlVision(CP)
    scc.map = SmartCruiseControlMap(CP)
  return scc
