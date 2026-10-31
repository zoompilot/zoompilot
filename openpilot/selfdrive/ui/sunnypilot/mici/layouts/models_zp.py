"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The tici models panel's driving options, for the comma 4 models panel.
"""
from collections.abc import Callable

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP, BigParamOption, speed_unit
from openpilot.system.ui.sunnypilot.lib.switch import Switch
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr


def _per_mph() -> float:
  """Display units per mph: LaneTurnValue is stored in mph."""
  return CV.MPH_TO_KPH if ui_state.is_metric else 1


def _hundredths(v: int) -> str:
  return f"{v / 100:.2f}"


def _per_ms() -> float:
  """Display units per m/s."""
  return CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH


def _speed(v: int) -> str:
  """A speed stored in m/s, x100 in the picker, in the display unit; 0 is off."""
  return str(round(v / 100 * _per_ms())) if v else tr("off")


def _option(title: str, param: str, fmt: Callable[[int], str], unit: str | Callable[[], str], **kwargs) -> BigParamOption:
  """A float param, x100 in the picker, whose button reads the picker label and the unit."""
  unit_text = unit if callable(unit) else lambda: unit
  return BigParamOption(title, param, float_param=True, label_callback=lambda v: f"{fmt(v)} {unit_text()}",
                        picker_label_callback=fmt, picker_unit=unit, **kwargs)


class ModelOptions:
  def __init__(self):
    # modeld_v2 runs Blendful and the camera offset; stock modeld, which runs the Default model, does neither
    self.lane_center = BigParamControlSP(tr("lane center assist"), "Blendful", depends_on=lambda: ui_state.active_runner_tinygrad,
                                         description=tr("Alpha. Nudges the car toward the middle of the lane, more when the lines are clear.\n" +
                                                        "Yields around obstacles. Not with the Default model."))

    # LaneTurnDesire is the picker's off. A metric step of 0.62 mph lands on each whole km/h, as the tici panel steps it
    self.lane_turn = _option(tr("lane turn desires"), "LaneTurnValue", lambda v: str(round(v / 100 * _per_mph())), speed_unit,
                             min_value=500, max_value=2000, value_change_step=lambda: round(100 / _per_mph()),
                             switch=Switch("LaneTurnDesire", False, tr("off")),
                             description=tr("With the blinker on below this speed, plans the turn that way.\n" +
                                            "Stops the car picking the wrong turn at a red light."))

    # off, then every whole display unit up to 20 mph (32 km/h)
    self.small_steering = BigParamOption(tr("small model steer"), "SmallModelSteering", float_param=True, min_value=0, max_value=900,
                                         value_change_step=lambda: round(100 / _per_ms()), picker_label_callback=_speed,
                                         picker_unit=speed_unit,
                                         label_callback=lambda v: tr("under {} {}").format(_speed(v), speed_unit()) if v else tr("off"),
                                         description=tr("With a big model, the small model steers below this speed. It is steadier when slow.\n" +
                                                        "Default is 11 mph (18 km/h). Not with the Default model."))
    self.small_steering.set_enabled(lambda: ui_state.active_runner_tinygrad)

    # LagdToggle is the picker's live
    self.steer_delay = _option(tr("steer delay"), "LagdToggleDelay", _hundredths, "s", min_value=5, max_value=50,
                               switch=Switch("LagdToggle", True, tr("live")),
                               description=tr("Live learns how long your car takes to respond to steering, as stock openpilot does.\n" +
                                              "A number is a fixed software delay instead. Default is live."))

    self.camera_offset = _option(tr("camera offset"), "CameraOffset", _hundredths, "m", min_value=-35, max_value=35,
                                 description=tr("Shifts where the model thinks the camera sits.\n" +
                                                "Positive moves the car left, negative right. Not with the Default model."))
    self.camera_offset.set_enabled(lambda: ui_state.active_runner_tinygrad)

    # between the panel's lane center and jetlink rows; small_steering sits after jetlink
    self.items = [self.lane_turn, self.steer_delay, self.camera_offset]

  def refresh(self):
    for item in (*self.items, self.small_steering):
      item.refresh()
