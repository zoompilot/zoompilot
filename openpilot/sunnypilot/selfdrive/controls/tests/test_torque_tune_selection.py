"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Which torque controller an unset TorqueControlTune selects. This is easy to get wrong by
# dropping `return_default=True` from the params read: params_keys.h declares a default, but
# a bare params.get() returns None for an unset param, and float(None) raises, or, guarded,
# can otherwise fall through to the upstream controller without an explicit error.
# The declared default is upstream's 0.0; the steer-to-zero Mazdas are seeded to 2.0 by
# _seed_mazda_torque_defaults instead, so other brands never inherit a tune fitted to that EPS.
#
# The v0 constructor is patched out: these tests pin the branch that gets taken, not the
# controller's behavior, and building the real one pulls in NNLC model loading.

import re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls import controlsd_ext
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

V0 = "v0"
V1 = "v1"  # stands in for the `lac` upstream controller controlsd passes in
V2 = "v2"
BY_VERSION = {0.0: V0, 1.0: V1, 2.0: V2}
PARAMS_KEYS_H = Path(__file__).resolve().parents[5] / "openpilot" / "common" / "params_keys.h"


def declared_default() -> float:
  """The default params_keys.h declares, read from the header text so the test does not
  depend on the compiled params library being current."""
  m = re.search(r'\{"TorqueControlTune", \{[^}]*, "([0-9.]+)"\}\}', PARAMS_KEYS_H.read_text())
  assert m, "TorqueControlTune must declare a default in params_keys.h"
  return float(m.group(1))


def compiled_default(params) -> float:
  return float(params.get("TorqueControlTune", return_default=True))


@pytest.fixture
def ctx(monkeypatch):
  monkeypatch.setattr(controlsd_ext, "LatControlTorqueV0", lambda *a, **k: V0)
  monkeypatch.setattr(controlsd_ext, "LatControlTorqueV2", lambda *a, **k: V2)
  with OpenpilotPrefix():
    params = Params()
    CP = car.CarParams.new_message(steerControlType="torque")
    CP.lateralTuning.init('torque')
    controls = SimpleNamespace(params=params, CP=CP.as_reader(),
                               CP_SP=custom.CarParamsSP.new_message().as_reader())
    yield params, controls


def select(controls):
  return ControlsExt.initialize_lateral_control(controls, V1, MagicMock(), 0.01)


class TestTorqueTuneSelection:
  def test_declared_default_is_upstreams(self):
    """Other brands must not inherit the Mazda tune through the param default; the Mazdas
    are seeded explicitly (test_torque_defaults_seed.py)."""
    assert declared_default() == 0.0

  def test_unset_selects_the_declared_default(self, ctx):
    """An unset param must resolve through params_keys.h (v0 today), not through None."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.remove("TorqueControlTune")
    if compiled_default(params) != declared_default():
      pytest.skip("libparams_c is stale against params_keys.h; rebuild with scons openpilot/common")
    assert select(controls) == BY_VERSION[declared_default()]

  @pytest.mark.parametrize(("version", "expected"), [(0.0, V0), (1.0, V1), (2.0, V2)])
  def test_explicit_version_is_honored(self, ctx, version, expected):
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == expected

  def test_every_declared_version_is_wired(self, ctx):
    """The versions file is what the UI selectors and the sunnylink schema offer, while
    initialize_lateral_control decides what is constructible. A version added to the file
    but not wired here would surface in every selector and silently run v1."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import load_versions

    wired = {0.0: V0, 1.0: V1, 2.0: V2}
    declared = {float(info["version"]) for info in load_versions().values()}
    assert declared == set(wired), "declared tune versions must match the wired controllers"

    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    for version, expected in wired.items():
      params.put("TorqueControlTune", version, block=True)
      assert select(controls) == expected

  @pytest.mark.parametrize("version", [1.0, 2.0])
  def test_torque_control_not_enforced_still_uses_v0_for_torque_cars(self, ctx, version):
    """Pre-existing behavior worth pinning: torque-tuned cars get v0 even with the toggle off.
    For 2.0 this is also the structural NNLC exclusion: enabling NNLC disables
    EnforceTorqueControl (ui_state/_cleanup_unsupported_params), so a stored v2 selection can
    never construct the v2 controller alongside NNLC."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == V0

  def test_ui_default_matches_what_controls_runs(self, ctx):
    """For an unset param the MICI selector lights up the declared default (the widget itself
    is pinned by test_torque_tune_unset_shows_declared_default); that version must be the one
    initialize_lateral_control picks, or the UI claims a tune the car isn't running."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.remove("TorqueControlTune")

    shown = float(params.get("TorqueControlTune", return_default=True))
    assert shown in set(SteeringLayoutMici._load_torque_versions().values()), \
      "the declared default must be a version the selectors offer"
    assert BY_VERSION[shown] == select(controls)
