"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import custom, log
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.cruise_ext import update_manual_button_timers
from openpilot.sunnypilot.selfdrive.controls.lib.relc import RoadEdgeMode, read_road_edge_mode

ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameZP = custom.OnroadEventZP.EventName

# selfdrived runs at 100 Hz
DISTANCE_LONG_PRESS = 50  # frames the distance button is held to switch Set Speed Assist
PARAMS_READ_FRAMES = 50  # also how soon a switch from the UI is announced
SPEED_ASSIST = "ExperimentalModeSetSpeed"


class SelfdrivedZP:
  """zoompilot's additions to selfdrived: Set Speed Assist's switch on a distance button long press
  (upstream's experimental/chill switch, which went with chill) and its alert, the wheel's "farther"
  distance button, ICBM's activation following card and the road edge alert's sound."""

  # Set Speed Assist's state after the last switch, for the alert (one selfdrived per process)
  switched_to: bool = True
  # RoadEdgeLaneChangeEnabled is silent: the road edge alert drops its chime
  road_edge_silent: bool = False

  def __init__(self, CP: structs.CarParams, params: Params | None = None):
    self.CP = CP
    self.params = params or Params()
    self.frame = 0

    self.speed_assist = self.params.get_bool(SPEED_ASSIST)
    self._read_road_edge_silent()
    self.gap_timers = {ButtonType.gapAdjustCruise: 0}
    self.speed_assist_switched = False
    # upstream steps the personality down on a gapAdjustCruise release unless it ended a long press
    self.gap_release_steps = True
    self.distance_farther_prev = False

  def _read_road_edge_silent(self) -> None:
    SelfdrivedZP.road_edge_silent = read_road_edge_mode(self.params) == RoadEdgeMode.SILENT

  def update(self, CS, CS_SP, events, events_sp, icbm, personality: int) -> int:
    """Run just before selfdrived's own personality step; returns the personality."""
    self.frame += 1
    if self.frame % PARAMS_READ_FRAMES == 0:
      self._read_speed_assist(events_sp)
      self._read_road_edge_silent()

    self.gap_release_steps = True
    if self.CP.openpilotLongitudinalControl:
      gap_released = any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents)
      if CS.cruiseState.available:
        update_manual_button_timers(CS, self.gap_timers)
        # switch Set Speed Assist once per distance button hold
        if self.gap_timers[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.speed_assist_switched:
          self.params.put_bool(SPEED_ASSIST, not self.speed_assist)
          self._switched(not self.speed_assist, events_sp)
          self.speed_assist_switched = True
      if gap_released:
        self.gap_release_steps = not self.speed_assist_switched
        self.speed_assist_switched = False

      # the farther button steps the personality up, unless upstream's gapAdjustCruise release
      # steps it down this frame
      if self.distance_farther_released(CS_SP) and not gap_released:
        personality = (personality + 1) % 3
        self.params.put('LongitudinalPersonality', personality)
        events.add(EventName.personalityChanged)
    else:
      self.distance_farther_released(CS_SP)

    icbm.update_activation(CS_SP)
    return personality

  def _read_speed_assist(self, events_sp) -> None:
    """Announces a switch made elsewhere (the UI's quick toggles and settings, sunnylink)."""
    speed_assist = self.params.get_bool(SPEED_ASSIST)
    if speed_assist != self.speed_assist:
      self._switched(speed_assist, events_sp)

  def _switched(self, speed_assist: bool, events_sp) -> None:
    self.speed_assist = speed_assist
    if self.CP.openpilotLongitudinalControl:
      SelfdrivedZP.switched_to = speed_assist
      events_sp.zp.add(EventNameZP.speedAssistSwitched)

  def distance_farther_released(self, CS_SP) -> bool:
    """The release edge of the wheel's "farther" distance button (carStateSP.zoompilot), the
    opposite direction to upstream's gapAdjustCruise cycle. False on cars without one."""
    pressed = bool(CS_SP.zoompilot.distanceFarther)
    released = self.distance_farther_prev and not pressed
    self.distance_farther_prev = pressed
    return released
