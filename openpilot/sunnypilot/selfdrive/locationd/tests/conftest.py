"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.selfdrive.locationd import torqued
from openpilot.sunnypilot.selfdrive.locationd import torqued_ext
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import FakeParams


def _route_params(monkeypatch, fake):
  # both Params sites: torqued reads the caches, torqued_ext reads the toggles
  monkeypatch.setattr(torqued, "Params", lambda: fake)
  monkeypatch.setattr(torqued_ext, "Params", lambda: fake)
  return fake


@pytest.fixture
def fake_params(monkeypatch):
  """One FakeParams behind both Params sites, speed-dep toggle chain on, caches empty."""
  return _route_params(monkeypatch, FakeParams())


@pytest.fixture
def fake_params_off(monkeypatch):
  """Same, with the speed-dep toggle chain off."""
  return _route_params(monkeypatch, FakeParams(speed_dep_on=False))
