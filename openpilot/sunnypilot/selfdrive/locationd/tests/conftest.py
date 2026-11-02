"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.selfdrive.locationd import torqued
from openpilot.sunnypilot.selfdrive.locationd import torqued_ext
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams


def _route_params(monkeypatch, fake):
  # both Params sites: torqued reads the caches, torqued_ext reads the toggles
  monkeypatch.setattr(torqued, "Params", lambda: fake)
  monkeypatch.setattr(torqued_ext, "Params", lambda: fake)
  return fake


@pytest.fixture
def fake_params(monkeypatch):
  """One FakeParams behind both Params sites, Enforce Torque Control and Self-Tune on, caches
  empty. Speed-dep learning runs wherever self-tune does, and make_cp's brand is not one
  upstream self-tunes, so both toggles are needed."""
  return _route_params(monkeypatch, FakeParams(EnforceTorqueControl=True, LiveTorqueParamsToggle=True))


@pytest.fixture
def fake_params_off(monkeypatch):
  """Same, with Enforce Torque Control and Self-Tune off."""
  return _route_params(monkeypatch, FakeParams())
