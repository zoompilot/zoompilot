"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log
from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET

EventName = log.OnroadEvent.EventName


def _car_events(brand: str, flags: int = 0) -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = brand
  CP.flags = int(flags)
  return CarSpecificEventsSP(CP, structs.CarParamsSP())


def _events(*names: int) -> Events:
  events = Events()
  for name in names:
    events.add(name)
  return events


class TestMazdaSteerFaultEvents:
  """The 2022 EPS non-delivery latch reports through steerFaultTemporary. The latch already
  zeroes the command, so the report is a warning and must not escalate to a soft disable."""

  def test_steer_to_zero_eps_downgrades_to_the_silent_warning(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerTempUnavailable)
    car_events.update(structs.CarState(), events)
    assert not events.has(EventName.steerTempUnavailable)
    assert events.has(EventName.steerTempUnavailableSilent)
    assert events.contains(ET.WARNING)
    assert not events.contains(ET.SOFT_DISABLE)
    assert not events.contains(ET.NO_ENTRY)

  def test_older_eps_keeps_upstream_escalation(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    events = _events(EventName.steerTempUnavailable)
    car_events.update(structs.CarState(), events)
    assert events.has(EventName.steerTempUnavailable)
    assert not events.has(EventName.steerTempUnavailableSilent)
    assert events.contains(ET.SOFT_DISABLE)

  def test_other_brands_are_untouched(self):
    car_events = _car_events('honda', MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerTempUnavailable)
    car_events.update(structs.CarState(), events)
    assert events.has(EventName.steerTempUnavailable)
    assert not events.has(EventName.steerTempUnavailableSilent)

  def test_other_events_survive_the_swap(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerTempUnavailable, EventName.steerUnavailable, EventName.steerSaturated)
    car_events.update(structs.CarState(), events)
    assert events.has(EventName.steerUnavailable)
    assert events.has(EventName.steerSaturated)
    assert events.has(EventName.steerTempUnavailableSilent)
    assert not events.has(EventName.steerTempUnavailable)

  def test_nothing_to_downgrade_is_a_no_op(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerUnavailable)
    car_events.update(structs.CarState(), events)
    assert events.names == [EventName.steerUnavailable]
