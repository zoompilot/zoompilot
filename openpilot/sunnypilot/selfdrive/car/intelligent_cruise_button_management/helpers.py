"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
def get_minimum_set_speed(is_metric: bool) -> int:
  return 30 if is_metric else 20


def icbm_applicable(CP, CP_SP) -> bool:
  """ICBM drives a set speed the car's own ECU keeps. That is every button-actuated platform
  under stock cruise, and under openpilot longitudinal only the ports where the ECU still
  keeps the setpoint (pcmCruise, e.g. Mazda alpha long with its body cruise); where openpilot
  owns the setpoint outright there is nothing for the buttons to move."""
  return bool(CP_SP.intelligentCruiseButtonManagementAvailable and (CP.pcmCruise or not CP.openpilotLongitudinalControl))
