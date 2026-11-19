"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's events beside sunnypilot's (EventsSP.zp): what selfdrived does with all of them at once.
"""
from openpilot.cereal import custom, messaging
from openpilot.sunnypilot.selfdrive.car.car_specific import CarEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET

EventNameSP = custom.OnroadEventSP.EventName
EventNameZP = custom.OnroadEventZP.EventName


def both():
  events_sp = EventsSP()
  events_sp.add(EventNameSP.lkasEnable)
  events_sp.zp.add(EventNameZP.stockLkasOff)
  return events_sp


def test_the_enums_share_ints_but_not_names():
  events_sp = both()
  assert int(EventNameZP.silentPedalPressed) == int(EventNameSP.lkasDisable)
  assert not events_sp.has(EventNameSP.lkasDisable)
  assert not events_sp.zp.has(EventNameZP.silentPedalPressed)
  assert events_sp.names == [EventNameSP.lkasEnable, 'stockLkasOff']


def test_selfdrived_queries_cover_both():
  events_sp = both()
  assert events_sp.contains(ET.ENABLE) and events_sp.contains(ET.NO_ENTRY)
  alerts = events_sp.create_alerts([ET.ENABLE, ET.NO_ENTRY])
  assert [a.alert_type for a in alerts] == ['lkasEnable/enable', 'stockLkasOff/noEntry']
  events_sp.clear()
  assert events_sp.names == []
  assert events_sp.event_counters[EventNameSP.lkasEnable] == 1 and events_sp.zp.event_counters[EventNameZP.stockLkasOff] == 1


def test_published_lists_are_each_enums_own():
  events_sp = both()
  msg = messaging.new_message('onroadEventsSP')
  msg.onroadEventsSP.events = events_sp.to_msg()
  msg.onroadEventsSP.zoompilot.events = events_sp.zp.to_msg()
  reader = msg.as_reader().onroadEventsSP
  assert [str(e.name) for e in reader.events] == ['lkasEnable']
  assert [str(e.name) for e in reader.zoompilot.events] == ['stockLkasOff']


def test_car_specific_round_trip_keeps_ours():
  car_events = CarEventsSP()
  car_events.add(EventNameSP.lkasEnable)
  car_events.zp.add(EventNameZP.stockEcuReady)
  events_sp = EventsSP()
  events_sp.add_from_msg(car_events.to_msg())
  assert events_sp.has(EventNameSP.lkasEnable) and events_sp.zp.has(EventNameZP.stockEcuReady)
  assert events_sp.names == [EventNameSP.lkasEnable, 'stockEcuReady']  # ours not read as sunnypilot's same int
