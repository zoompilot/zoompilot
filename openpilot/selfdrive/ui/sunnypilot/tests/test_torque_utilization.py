"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The torque bar and lane lines reach full scale at the EPS rail, where a torque tune saturates.

from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.sunnypilot.car.lateral_tune import get_steer_rail_schedule
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state

MAZDA = structs.CarParams(brand='mazda', carFingerprint=CAR.MAZDA_CX5_2022, flags=int(MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS))


def _utilization(monkeypatch, CP, torque, v_ego):
  monkeypatch.setattr(ui_state, '_steer_rail_schedule', get_steer_rail_schedule(CP))
  monkeypatch.setattr(ui_state, 'sm', {
    'controlsState': SimpleNamespace(lateralControlState=SimpleNamespace(which=lambda: 'torqueState')),
    'carOutput': SimpleNamespace(actuatorsOutput=SimpleNamespace(torque=torque)),
    'carState': SimpleNamespace(vEgo=v_ego),
  })
  ui_state._update_torque_utilization()
  return ui_state.torque_utilization


class TestTorqueUtilization:
  # route 21d103861daeed11/000003db--f4124f0aa8/8, t=10.39: 620 counts at 19.7 m/s, pinned there
  # 0.7 s before steerSaturated; the upstream bar read 0.775 of that build's 800 scale
  @pytest.mark.parametrize("v_ego, torque", [(19.69, 620 / 1200), (13.9, 676 / 1200), (10.3, 1048 / 1200)])
  def test_rail_is_full_scale(self, monkeypatch, v_ego, torque):
    assert _utilization(monkeypatch, MAZDA, torque, v_ego) == pytest.approx(1.0, abs=1e-3)

  def test_below_rail_is_proportional(self, monkeypatch):
    assert _utilization(monkeypatch, MAZDA, -310 / 1200, 20.0) == pytest.approx(-0.5, abs=1e-3)

  def test_no_ceiling_is_applied_torque(self, monkeypatch):
    assert _utilization(monkeypatch, structs.CarParams(brand='toyota'), 0.6, 20.0) == 0.6

  def test_torque_bar_reads_it(self, monkeypatch):
    _utilization(monkeypatch, MAZDA, 620 / 1200, 19.69)
    bar = TorqueBar()
    for _ in range(200):
      bar._update_state()
    assert bar._torque_filter.x == pytest.approx(-1.0, abs=1e-3)
