"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.car.card_ext import CardExt
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams


def _card_ext(CS: SimpleNamespace) -> CardExt:
  CI = SimpleNamespace(CS=CS, CC=SimpleNamespace())
  return CardExt(structs.CarParams(), structs.CarParamsSP(), FakeParams(), None, None, CI)


def _car_state_sp():
  return custom.CarStateSP.new_message()


class TestFillCylinderDeactivation:
  def test_car_state_without_cylinder_fields_publishes_the_default(self):
    # only the Mazda CarStateExt carries cyl_state; every other brand (and the brands
    # with no CarStateExt at all) must publish normal/0.0 instead of raising, or card
    # dies on the first frame
    ext = _card_ext(SimpleNamespace())
    CS_SP = _car_state_sp()
    ext.fill_cylinder_deactivation(CS_SP)
    cyl = CS_SP.zoompilot.cylinderDeactivation
    assert cyl.state == 'normal'
    assert cyl.entryProgress == 0.0

  def test_mazda_fields_publish_through(self):
    ext = _card_ext(SimpleNamespace(cyl_state='entry', cyl_entry_progress=3 / 7))
    CS_SP = _car_state_sp()
    ext.fill_cylinder_deactivation(CS_SP)
    cyl = CS_SP.zoompilot.cylinderDeactivation
    assert cyl.state == 'entry'
    assert abs(cyl.entryProgress - 3 / 7) < 1e-6
