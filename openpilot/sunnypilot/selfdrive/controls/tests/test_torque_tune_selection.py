"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Which torque controller an unset TorqueControlTune selects. This is easy to get wrong by
# dropping `return_default=True` from the params read: params_keys.h declares a default, but
# a bare params.get() returns None for an unset param, and float(None) raises, or, guarded,
# can otherwise fall through to the upstream controller without an explicit error.
# The small model's declared default is upstream's 0.0; the steer-to-zero Mazdas are seeded to
# 2.0 by _seed_mazda_torque_defaults instead, so other brands never inherit a tune fitted to
# that EPS. A big model's is v1 for every brand.
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

class FakeLaC(str):
  """A controller stand-in that compares as its version label and counts resets."""
  resets = 0

  def reset(self):
    self.resets += 1


V0 = "v0"
V1 = "v1"  # stands in for the `lac` upstream controller controlsd passes in
V2 = "v2"
BY_VERSION = {0.0: V0, 1.0: V1, 2.0: V2}
PARAMS_KEYS_H = Path(__file__).resolve().parents[5] / "openpilot" / "common" / "params_keys.h"


def declared_default(key: str = "TorqueControlTune") -> float:
  """The default params_keys.h declares, read from the header text so the test does not
  depend on the compiled params library being current."""
  m = re.search(r'\{"' + key + r'", \{[^}]*, "([0-9.]+)"\}\}', PARAMS_KEYS_H.read_text())
  assert m, f"{key} must declare a default in params_keys.h"
  return float(m.group(1))


def require_current_libparams(params, *keys: str) -> None:
  for key in keys:
    if float(params.get(key, return_default=True)) != declared_default(key):
      pytest.skip("libparams_c is stale against params_keys.h; rebuild with scons openpilot/common")


@pytest.fixture
def ctx(monkeypatch):
  monkeypatch.setattr(controlsd_ext, "LatControlTorqueV0", lambda *a, **k: FakeLaC(V0))
  monkeypatch.setattr(controlsd_ext, "LatControlTorqueV2", lambda *a, **k: FakeLaC(V2))
  with OpenpilotPrefix():
    params = Params()
    params.put_bool("EnforceTorqueControl", True, block=True)  # the enforce-off tests flip it
    CP = car.CarParams.new_message(steerControlType="torque")
    CP.lateralTuning.init('torque')
    controls = SimpleNamespace(params=params, CP=CP.as_reader(),
                               CP_SP=custom.CarParamsSP.new_message().as_reader())
    yield params, controls


def select(controls):
  """What controlsd does at startup: build and install the small model's controller."""
  controls.LaC = ControlsExt.initialize_lateral_control(controls, FakeLaC(V1), MagicMock(), 0.01)
  return controls.LaC


def swap(controls, big: bool):
  ControlsExt.select_lateral_control(controls, {'modelV2': SimpleNamespace(big=big)})


class TestTorqueTuneSelection:
  def test_declared_defaults(self):
    """Upstream's v0 for the small model, so other brands never inherit the Mazda tune through
    the default (the Mazdas are seeded, test_torque_defaults_seed.py); v1 for a big one."""
    assert declared_default("TorqueControlTune") == 0.0
    assert declared_default("TorqueControlTuneBig") == 1.0

  def test_unset_selects_the_declared_default(self, ctx):
    """An unset param must resolve through params_keys.h (v0 today), not through None."""
    params, controls = ctx
    params.remove("TorqueControlTune")
    require_current_libparams(params, "TorqueControlTune")
    assert select(controls) == BY_VERSION[declared_default()]

  @pytest.mark.parametrize(("version", "expected"), [(0.0, V0), (1.0, V1), (2.0, V2)])
  def test_explicit_version_is_honored(self, ctx, version, expected):
    params, controls = ctx
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

  def test_unset_big_tune_selects_its_declared_default(self, ctx):
    """An unset TorqueControlTuneBig resolves through params_keys.h (v1), not through the
    small tune and not through None."""
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.remove("TorqueControlTuneBig")
    require_current_libparams(params, "TorqueControlTuneBig")
    select(controls)
    swap(controls, big=True)
    assert controls.LaC == BY_VERSION[declared_default("TorqueControlTuneBig")]

  def test_same_tune_for_both_sizes_never_swaps(self, ctx):
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    select(controls)
    swap(controls, big=True)
    assert controls.LaC == V2 and controls.LaC.resets == 0

  @pytest.mark.parametrize(("small", "big"), [(2.0, 1.0), (0.0, 2.0), (1.0, 0.0)])
  def test_lateral_control_follows_the_model_size(self, ctx, small, big):
    """Every drive starts on the small model's tune; the frame after modelV2.big flips runs
    the other size's controller, reset once, and a repeated frame does not churn it."""
    params, controls = ctx
    params.put("TorqueControlTune", small, block=True)
    params.put("TorqueControlTuneBig", big, block=True)
    assert select(controls) == BY_VERSION[small]

    swap(controls, big=True)
    assert controls.LaC == BY_VERSION[big] and controls.LaC.resets == 1
    swap(controls, big=True)
    assert controls.LaC.resets == 1

    swap(controls, big=False)
    assert controls.LaC == BY_VERSION[small] and controls.LaC.resets == 1

  def test_enforce_off_ignores_the_big_tune(self, ctx):
    """The enforce-off v0 forcing applies to both sizes."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put("TorqueControlTune", 1.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    assert select(controls) == V0
    swap(controls, big=True)
    assert controls.LaC == V0 and controls.LaC.resets == 0

  def test_ui_default_matches_what_controls_runs(self, ctx):
    """For an unset param the MICI selector lights up the declared default (the widget itself
    is pinned by test_torque_tune_unset_shows_declared_default); that version must be the one
    initialize_lateral_control picks, or the UI claims a tune the car isn't running."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import versions_by_label

    params, controls = ctx
    params.remove("TorqueControlTune")

    shown = float(params.get("TorqueControlTune", return_default=True))
    assert shown in set(versions_by_label().values()), \
      "the declared default must be a version the selectors offer"
    assert BY_VERSION[shown] == select(controls)

  @pytest.mark.parametrize(("small", "big", "enforce", "expected"), [
    (2.0, 2.0, True, False), (2.0, 1.0, True, True), (0.0, 2.0, True, True), (2.0, 2.0, False, True)])
  def test_jerk_aware_has_effect_unless_every_size_runs_v2(self, ctx, small, big, enforce, expected):
    """v2 forces the jerk-aware controller off, so its toggle only locks when both sizes run
    v2; with Enforce Torque Control off both run v0."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import jerk_aware_has_effect

    params, _ = ctx
    params.put_bool("EnforceTorqueControl", enforce, block=True)
    params.put("TorqueControlTune", small, block=True)
    params.put("TorqueControlTuneBig", big, block=True)
    assert jerk_aware_has_effect(params) is expected
