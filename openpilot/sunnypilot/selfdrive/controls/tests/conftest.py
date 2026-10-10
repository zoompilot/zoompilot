"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from opendbc.sunnypilot.car import interfaces as sp_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_torque_ext_override as override_module
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_zp import LatControlTorqueExtZP
from openpilot.sunnypilot.selfdrive.controls.tests.speed_dep_helpers import FakeParams, make_cp


class TorqueParamsOverride(LatControlTorqueExtZP, LatControlTorqueExtOverride):
  """The torque-params half of LatControlTorqueExt: upstream's manual override under the fork's
  layer, without NNLC, which needs model files to init. The host only carries steer_max."""

  def __init__(self, CP):
    LatControlTorqueExtOverride.__init__(self, CP)
    LatControlTorqueExtZP.__init__(self, CP)
    self.lac_torque = SimpleNamespace(steer_max=1.0)


@pytest.fixture
def make_override(monkeypatch):
  """Factory for a TorqueParamsOverride whose Params is a FakeParams; the fake is reachable
  afterwards as ovr.params."""
  def _make(enforce=False, manual_override=False, manual_lat_accel_factor='200', manual_friction='15', CP=None):
    fake = FakeParams(enforce, manual_override, manual_lat_accel_factor, manual_friction)
    monkeypatch.setattr(override_module, "Params", lambda: fake)
    return TorqueParamsOverride(CP or make_cp())
  return _make


@pytest.fixture
def set_speed_dep_config(monkeypatch):
  """Replaces the speed_dependent.toml contents get_speed_dep_config_for_car reads."""
  def _set(cfg):
    monkeypatch.setattr(sp_interfaces, "get_speed_dep_config", lambda: cfg)
  return _set
