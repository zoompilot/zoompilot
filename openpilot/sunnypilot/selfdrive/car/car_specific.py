"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.mads import SET_SPEED_BUTTONS
from opendbc.sunnypilot.car.stock_ecu import ENGAGES_NORMALLY
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
StockEcuState = custom.CarStateZP.StockEcuState



class CarSpecificEventsSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.low_speed_alert = False
    self.stock_ecu_prev = StockEcuState.notNeeded

  def update(self, CS: structs.CarState, events: Events, CS_SP):
    events_sp = EventsSP()

    if self.CP.brand == 'chrysler':
      if self.CP.carFingerprint in RAM_DT:
        # remove belowSteerSpeed event from CarSpecificEvents as RAM_DT uses a different logic
        if events.has(EventName.belowSteerSpeed):
          events.remove(EventName.belowSteerSpeed)

        # TODO-SP: use if/elif to have the gear shifter condition takes precedence over the speed condition
        # TODO-SP: add 1 m/s hysteresis
        if CS.vEgo >= self.CP.minEnableSpeed:
          self.low_speed_alert = False
        if self.CP.minEnableSpeed >= 14.5 and CS.gearShifter != GearShifter.drive:
          self.low_speed_alert = True
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    elif self.CP.brand == 'mazda':
      if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS and events.has(EventName.steerTempUnavailable):
        # steerFaultTemporary on this EPS is the non-delivery latch reporting a sustained
        # road-speed block. The latch has already zeroed the command, so there is nothing
        # for a soft disable to protect; it only costs MADS its lateral after 3 s and
        # shouts at the driver. Keep the banner, drop the escalation.
        events.remove(EventName.steerTempUnavailable)
        events.add(EventName.steerTempUnavailableSilent)
      if CS.stockLkas:
        # carstate pulses stockLkas once per arming episode when the controller's presses on the
        # camera bus left the camera's own TJA/CTS armed. Upstream's alert is a no-entry for a
        # lane-departure nudge; this is a one-shot warning naming the button, openpilot keeps
        # steering (the panda blocks the camera's command).
        events.remove(EventName.stockLkas)
        events_sp.add(EventNameSP.mazdaStockCtsActive)

    # A SET/RES press before the stock ECU openpilot stands in for is owned lands on a body
    # that will not engage (Mazda route 0000020d: six presses, nothing shown): name what the
    # driver has to do. Alert-only; the press itself does nothing.
    stock_ecu = CS_SP.zoompilot.stockEcu
    if str(stock_ecu) not in ENGAGES_NORMALLY and any(be.pressed and be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents):
      events_sp.add(EventNameSP.stockEcuNotReady)
    # Unprompted only where waiting changes the outcome: parked with the takeover still
    # starting (route 0000021b pulled away 5 s short of it). Rolling, the press alert carries
    # "park to engage"; a standing banner would nag a whole drive. One line on the ready edge.
    if stock_ecu == StockEcuState.starting and CS.standstill:
      events_sp.add(EventNameSP.stockEcuInitializing)
    if stock_ecu == StockEcuState.ready and self.stock_ecu_prev != StockEcuState.ready:
      events_sp.add(EventNameSP.stockEcuReady)
    self.stock_ecu_prev = stock_ecu

    return events_sp
