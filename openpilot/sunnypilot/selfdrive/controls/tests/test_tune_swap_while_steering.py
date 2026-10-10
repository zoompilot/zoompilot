"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The torque tune swap on a model change, against the real controllers: the big model's v1
# steering and the small model's v2 idle with state from an earlier curve, as after a join.
# On the 2026-09-29 drives a hand-back while steering swapped to the idle v2 at once and the
# commanded torque stepped by 0.09 to 0.19 in one tick. The tune that steers now carries the
# small model to the first frame lateral is inactive.

from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque as LatControlTorqueV1
from openpilot.sunnypilot.selfdrive.controls.controls_lateral_zp import TUNE_SWAP_INACTIVE_FRAMES, ControlsLateralZP
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import LatControlTorque as LatControlTorqueV2
from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v2 import FRICTION, make_cs, make_lac, step

V_EGO = 10.0
LAT_ACCEL = 0.8           # a steady curve, and the car following it
HAND_BACK = 200           # the frame the small model's first modelV2 arrives, v1 settled


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


def controls():
  """controlsd's two controllers, the big one steering, the small one idle with the state it
  had when it last steered, on another curve at another speed."""
  small, big = make_lac(LatControlTorqueV2, friction=FRICTION), make_lac(LatControlTorqueV1, friction=FRICTION)
  for _ in range(300):
    step(small, make_cs(v_ego=20.0, lat_accel=-1.5), -1.5 / 20.0 ** 2)
  return SimpleNamespace(_lacs=(small, big), _lac_by_size={False: small, True: big}, LaC=big, _inactive_frames=0)


def drive(ctl, hand_back: bool, inactive_at: int | None = None, frames: int = 300,
          inactive: set[int] | None = None) -> list[float]:
  """Commanded torque every frame, as controlsd runs it: state_control's update, then the
  swap at the end of the frame. Lateral is inactive from `inactive_at` on, and on the
  frames in `inactive`."""
  out = []
  for i in range(frames):
    active = (inactive_at is None or i < inactive_at) and i not in (inactive or ())
    torque, _, _ = ctl.LaC.update(active, make_cs(V_EGO, LAT_ACCEL), *DRIVE_ARGS)
    ControlsLateralZP.note_lat_active(ctl, active)
    out.append(torque)
    ControlsLateralZP.select_lateral_control(ctl, {'modelV2': SimpleNamespace(big=not (hand_back and i >= HAND_BACK))})
  return out


def _drive_args():
  from openpilot.sunnypilot.selfdrive.controls.tests.test_latcontrol_torque_v2 import LAT_DELAY, LP, VM
  return (VM, LP, False, LAT_ACCEL / V_EGO ** 2, None, False, LAT_DELAY)


DRIVE_ARGS = _drive_args()


def largest_tick(out: list[float], start: int, end: int) -> float:
  return max(abs(b - a) for a, b in zip(out[start:end], out[start + 1:end], strict=False))


class TestSwapWhileSteering:
  def test_a_hand_back_while_steering_leaves_the_torque_as_it_was(self, params):
    steady = drive(controls(), hand_back=False)
    handed_back = drive(controls(), hand_back=True)
    assert handed_back == pytest.approx(steady, abs=1e-9)

  def test_swapping_at_once_would_have_stepped_it(self, params):
    # what the switch did before: the idle v2 takes over on the hand-back frame
    ctl = controls()
    steady = drive(controls(), hand_back=False)
    out = []
    for i in range(HAND_BACK + 20):
      if i == HAND_BACK:
        ctl.LaC = ctl._lac_by_size[False]
        ctl.LaC.reset()
      torque, _, _ = ctl.LaC.update(True, make_cs(V_EGO, LAT_ACCEL), *DRIVE_ARGS)
      out.append(torque)
    assert abs(out[HAND_BACK] - out[HAND_BACK - 1]) > 0.05
    assert abs(out[HAND_BACK] - out[HAND_BACK - 1]) > 10 * largest_tick(steady, HAND_BACK - 50, HAND_BACK)

  def test_a_short_drop_keeps_the_tune_and_the_torque(self, params):
    # a steer fault flicker, and a 0.3 s gap: neither swaps, so the resume is the same
    # controller as without the hand-back
    gaps = {HAND_BACK + 20} | set(range(HAND_BACK + 40, HAND_BACK + 70))
    steady = drive(controls(), hand_back=False, inactive=gaps)
    ctl = controls()
    handed_back = drive(ctl, hand_back=True, inactive=gaps)
    assert handed_back == pytest.approx(steady, abs=1e-9)
    assert ctl.LaC is ctl._lac_by_size[True]

  def test_half_a_second_off_swaps(self, params):
    off = HAND_BACK + 40
    ctl = controls()
    drive(ctl, hand_back=True, inactive_at=off, frames=off + TUNE_SWAP_INACTIVE_FRAMES - 1)
    assert ctl.LaC is ctl._lac_by_size[True], "swapped before lateral had been off for half a second"
    ctl = controls()
    drive(ctl, hand_back=True, inactive_at=off, frames=off + TUNE_SWAP_INACTIVE_FRAMES)
    assert ctl.LaC is ctl._lac_by_size[False], "not swapped after half a second off"
