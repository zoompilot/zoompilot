"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest
from opendbc.car.structs import car

from openpilot.selfdrive.ui.sunnypilot.longitudinal_mode import alpha_longitudinal_reachable, longitudinal_mode_labels
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams


@pytest.mark.parametrize(("has_long", "experimental", "dec", "nudge", "labels"), [
  (False, True, True, True, []),
  (True, False, True, True, ["chill"]),
  (True, True, True, True, ["experimental", "dec"]),  # the nudge never acts under DEC
  (True, True, False, False, ["experimental"]),
  (True, True, False, True, ["experimental", "nudge"]),
])
def test_cruise_shows_the_mode_that_drives(has_long, experimental, dec, nudge, labels):
  ui_state = SimpleNamespace(has_longitudinal_control=has_long, experimental_mode=experimental,
                             params=FakeParams(DynamicExperimentalControl=dec, ExperimentalModeSetSpeed=nudge))
  assert longitudinal_mode_labels(ui_state) == labels


@pytest.mark.parametrize(("cp", "has_long", "reachable"), [
  (None, False, True),  # before the first drive
  (car.CarParams.new_message(alphaLongitudinalAvailable=True), False, True),
  (car.CarParams.new_message(), True, True),
  (car.CarParams.new_message(), False, False),
])
def test_greyed_only_without_any_longitudinal(cp, has_long, reachable):
  assert alpha_longitudinal_reachable(SimpleNamespace(CP=cp, has_longitudinal_control=has_long)) == reachable
