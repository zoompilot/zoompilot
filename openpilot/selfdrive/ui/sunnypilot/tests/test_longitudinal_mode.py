"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest
from opendbc.car.structs import car

from openpilot.selfdrive.ui.sunnypilot.longitudinal_mode import longitudinal_reachable


@pytest.mark.parametrize(("cp", "has_long", "reachable"), [
  (None, False, True),  # before the first drive
  (car.CarParams.new_message(alphaLongitudinalAvailable=True), False, True),
  (car.CarParams.new_message(), True, True),
  (car.CarParams.new_message(), False, False),
])
def test_hidden_only_without_any_longitudinal(cp, has_long, reachable):
  assert longitudinal_reachable(SimpleNamespace(CP=cp, has_longitudinal_control=has_long)) == reachable
