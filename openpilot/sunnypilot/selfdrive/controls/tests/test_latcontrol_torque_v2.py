"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Behavior tests for the v2 torque tune, run against the real controllers with a toy
# steering geometry (curvature proportional to steering angle) and a mocked car interface
# (torque == lat_accel / latAccelFactor). The load-bearing property is the parity test: with
# KD zeroed, the deadzone zeroed and the jerk filter bypassed, v2 IS v0 frame for frame on a
# moving request and a moving measurement. Everything else is a deliberate, tested delta.

import math
from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_torque_v2 as v2_module
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import (
  LatControlTorque as LatControlTorqueV2,
  get_center_chatter_jerk_deadzone,
  MAX_FRICTION_JERK,
  RELEASE_ERROR_RAMP_T,
  STEER_RELEASE_I_DECAY,
)

DT = 0.01
LAT_DELAY = 0.3
DELAY_FRAMES = int(LAT_DELAY / DT)
LAF = 2.5
FRICTION = 0.25
CURV_PER_DEG = 2e-4  # toy geometry: curvature = -steeringAngleDeg * CURV_PER_DEG

VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)


def make_cp(friction=0.0):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = LAF
  CP.lateralTuning.torque.friction = friction
  return CP.as_reader()


def make_ci():
  CI = MagicMock()
  CI.torque_from_lateral_accel.return_value = lambda lataccel, tp: lataccel / tp.latAccelFactor
  CI.lateral_accel_from_torque.return_value = lambda torque, tp: torque * tp.latAccelFactor
  return CI


def make_lac(cls, friction=0.0):
  return cls(make_cp(friction=friction), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT)


def make_pair(friction=0.0):
  return make_lac(LatControlTorqueV0, friction=friction), make_lac(LatControlTorqueV2, friction=friction)


def make_cs(v_ego=15.0, lat_accel=0.0, pressed=False):
  """CarState whose measured lateral accel equals lat_accel at v_ego."""
  angle = -lat_accel / (CURV_PER_DEG * v_ego ** 2)
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=pressed)


def step(lac, cs, desired_curvature, active=True, lp=LP, lat_delay=LAT_DELAY, sls=False):
  _, _, pid_log = lac.update(active, cs, VM, lp, sls, desired_curvature, None, False, lat_delay)
  return pid_log


def friction_term(pid_log, v_ego, desired_curvature):
  """roll and latAccelOffset are 0 in these tests, so f minus the request is the friction term"""
  return pid_log.f - desired_curvature * v_ego ** 2


class NoKD(LatControlTorqueV2):
  KD_SCHEDULE = 0.0


def make_v0_equivalent(monkeypatch, friction=0.0):
  """v2 with its three shaping deltas neutralized: KD 0, deadzone 0, jerk filter identity."""
  monkeypatch.setattr(v2_module, 'get_center_chatter_jerk_deadzone', lambda v, s: 0.0)
  v2 = make_lac(NoKD, friction=friction)
  v2.jerk_filter.update = lambda x: x
  return v2


class LatAccelBuffer:
  """v0 buffer semantics behind v2's curvature-buffer interface (store lat accel at append
  time, undo the reader's live v^2 scaling). The owner's _v_now is set per frame by the test."""
  def __init__(self, owner, n):
    self.owner = owner
    self.buf = deque([0.0] * n, maxlen=n)

  def append(self, k):
    self.buf.append(k * self.owner._v_now ** 2)

  def __getitem__(self, idx):
    return self.buf[idx] / self.owner._v_now ** 2


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


class TestV0Parity:
  def test_v2_is_v0_with_the_deltas_off(self, params, monkeypatch):
    """The setpoint is v0's, so with KD, the deadzone and the jerk filter neutralized the two
    controllers must agree frame for frame on a moving request AND a moving measurement,
    friction included (the 0.3 Hz request keeps the differencer under the 2.5 m/s^3 clip)."""
    v0 = make_lac(LatControlTorqueV0, friction=FRICTION)
    v2 = make_v0_equivalent(monkeypatch, friction=FRICTION)
    v_ego = 15.0
    for i in range(600):
      t = i * DT
      desired = 1.0 * math.sin(2 * math.pi * 0.3 * t) / v_ego ** 2
      measured = 0.8 * math.sin(2 * math.pi * 0.3 * (t - 0.4))  # lagged, so the error moves too
      out0, _, log0 = v0.update(True, make_cs(v_ego, measured), VM, LP, False, desired, None, False, LAT_DELAY)
      out2, _, log2 = v2.update(True, make_cs(v_ego, measured), VM, LP, False, desired, None, False, LAT_DELAY)
      assert out2 == pytest.approx(out0, abs=1e-6), f"frame {i}"
      for field in ('error', 'p', 'i', 'd', 'f', 'desiredLateralAccel', 'desiredLateralJerk'):
        assert getattr(log2, field) == pytest.approx(getattr(log0, field), abs=1e-6), f"frame {i} {field}"
    assert abs(v2.pid.i) > 1e-3  # the integrator was live, not frozen, through the comparison
    assert log0.version == 0
    assert log2.version == 2

  def test_extension_output_overrides_disabled(self, params):
    """v2 owns the friction shaping and KD, so extension controllers that override the shared
    PID (jerk-aware, NNLC, any future sibling) are disabled no matter what the params say,
    and the PID limits stay in lat-accel space. v0 constructed with the same params keeps
    jerk-aware on (torque-space limits), pinning that the disable is v2's."""
    params.put_bool("LateralJerkTorqueController", True, block=True)
    params.put_bool("NeuralNetworkLateralControl", True, block=True)
    v2 = make_lac(LatControlTorqueV2)
    assert not v2.extension.overrides_output
    assert v2.pid.pos_limit == pytest.approx(LAF)
    v2.update_limits()  # must stay a no-op on the extension, or the per-frame override path restores torque-space limits
    assert v2.pid.pos_limit == pytest.approx(LAF)

    v0 = make_lac(LatControlTorqueV0)
    assert v0.extension.overrides_output
    assert v0.pid.pos_limit == pytest.approx(v0.steer_max)

  def test_pid_limits_follow_steer_max(self, params):
    """The rail arrives from the extension as steer_max; v0's update_limits (not overridden)
    must put the PID limits there."""
    v2 = make_lac(LatControlTorqueV2)
    v2.steer_max = 0.6
    v2.update_limits()
    assert v2.pid.pos_limit == pytest.approx(0.6 * LAF)
    assert v2.pid.neg_limit == pytest.approx(-0.6 * LAF)


class TestFrictionInput:
  def _tracking_run(self, v0, v2, v_ego, request_fn, frames):
    """The wheel tracks the delayed command, so the friction input is the jerk contribution
    alone. Returns (v0 friction terms, v2 friction terms)."""
    history = deque([0.0] * (DELAY_FRAMES + 1), maxlen=DELAY_FRAMES + 1)
    f0, f2 = [], []
    for i in range(frames):
      desired = request_fn(i * DT) / v_ego ** 2
      measured = history[-DELAY_FRAMES] * v_ego ** 2
      history.append(desired)
      log0 = step(v0, make_cs(v_ego, measured), desired)
      log2 = step(v2, make_cs(v_ego, measured), desired)
      f0.append(friction_term(log0, v_ego, desired))
      f2.append(friction_term(log2, v_ego, desired))
    return f0, f2

  def test_filter_attenuates_a_5hz_dither(self, params, monkeypatch):
    """A 5 Hz request dither (2 m/s^3 differencer amplitude) reaches the friction term through
    the 1.2 Hz filter at ~0.23 gain; deadzone off so the filter alone is measured."""
    monkeypatch.setattr(v2_module, 'get_center_chatter_jerk_deadzone', lambda v, s: 0.0)
    v0, v2 = make_pair(friction=FRICTION)
    f0, f2 = self._tracking_run(v0, v2, 25.0, lambda t: 0.3 * math.sin(2 * math.pi * 5.0 * t), 600)
    var0, var2 = float(sum(x * x for x in f0[200:])), float(sum(x * x for x in f2[200:]))
    assert var0 > 0.0
    assert var2 < 0.5 * var0

  def test_filter_passes_a_turn_in(self, params):
    """A 1 s ramp into a 1 m/s^2 turn (0.3 Hz quarter wave) reaches the friction term within
    10% of v0's peak: the filter shapes chatter, not turn-in."""
    v0, v2 = make_pair(friction=FRICTION)
    f0, f2 = self._tracking_run(v0, v2, 25.0, lambda t: min(1.0, max(0.0, t - 0.5)), 250)
    peak0, peak2 = max(abs(x) for x in f0), max(abs(x) for x in f2)
    assert peak0 > 0.2
    assert peak2 >= 0.9 * peak0

  def test_deadzone_only_acts_in_the_center_band(self, params, monkeypatch):
    """Above 0.35 m/s^2 of setpoint the friction term is identical with and without the
    deadzone; inside the band it is not."""
    def run(lo, hi, deadzone_off):
      v_ego = 25.0
      if deadzone_off:
        monkeypatch.setattr(v2_module, 'get_center_chatter_jerk_deadzone', lambda v, s: 0.0)
      lac = make_lac(LatControlTorqueV2, friction=FRICTION)
      fs = []
      for i in range(200):
        desired = (lo + (hi - lo) * min(1.0, i / 100)) / v_ego ** 2  # 1 s ramp between the two levels
        fs.append(step(lac, make_cs(v_ego, 0.0), desired).f)
      monkeypatch.undo()
      return fs
    def max_diff(lo, hi):
      return max(abs(a - b) for a, b in zip(run(lo, hi, False), run(lo, hi, True), strict=True))
    assert max_diff(0.5, 1.0) == pytest.approx(0.0, abs=1e-9)
    assert max_diff(0.0, 0.2) > 0.01
    assert get_center_chatter_jerk_deadzone(25.0, 0.0) == pytest.approx(0.18)
    assert get_center_chatter_jerk_deadzone(25.0, 0.35) == pytest.approx(0.0)
    assert get_center_chatter_jerk_deadzone(0.0, 0.0) == pytest.approx(0.08)

  def test_friction_jerk_input_is_clipped(self, params):
    """A 2 m/s^2 step is a 6.7 m/s^3 differencer; the filter must see it clipped at +-2.5
    while the logged desired jerk stays raw."""
    v2 = make_lac(LatControlTorqueV2)
    seen = []
    real_update = v2.jerk_filter.update
    v2.jerk_filter.update = lambda x: (seen.append(x), real_update(x))[1]
    v_ego = 25.0
    for i in range(80):
      desired = (2.0 if 20 <= i < 50 else 0.0) / v_ego ** 2
      log = step(v2, make_cs(v_ego), desired)
      if i in (20, 50):
        assert abs(log.desiredLateralJerk) > 6.0
    assert max(seen) == pytest.approx(MAX_FRICTION_JERK)
    assert min(seen) == pytest.approx(-MAX_FRICTION_JERK)


class TestSharedGainSchedule:
  """v2 keeps v0's gain schedule verbatim: the low-speed KP flatten (KP ~7 below 7.5 m/s,
  2026-08-30 routes 129-12c) tripled the felt vehicle weave at 2.5-5 m/s and was reverted.
  Any low-speed retune must come from a system-ID of the loop, not a schedule guess."""

  def test_kp_schedule_matches_v0_everywhere(self, params):
    v0, v2 = make_pair()
    for i in range(400):
      v0.pid.speed = v2.pid.speed = i * 0.1
      assert v2.pid.k_p == pytest.approx(v0.pid.k_p), f"{i * 0.1} m/s"


class TestLowSpeedDamping:
  """kd = 0.3 s * KP(v), capped below 7.5 m/s and faded out by 14.5 m/s (route 12e steps:
  closed-loop sim overshoot 1.63 -> 1.21). v0 keeps KD = 0; the shared -measurement_rate
  argument is dead there."""

  def test_kd_schedule_pins(self, params):
    _, v2 = make_pair()
    for v_ego, kd in [(2.0, 1.65), (7.5, 1.65), (9.0, 1.29), (10.0, 1.05), (14.5, 0.0), (25.0, 0.0)]:
      v2.pid.speed = v_ego
      assert v2.pid.k_d == pytest.approx(kd), f"{v_ego} m/s"

  def test_v0_kd_stays_zero(self, params):
    v0, _ = make_pair()
    for v_ego in [2.0, 9.0, 25.0]:
      v0.pid.speed = v_ego
      assert v0.pid.k_d == 0.0

  def _sweep(self, v_ego, pressed=False):
    v2 = make_lac(LatControlTorqueV2)
    desired = 0.5 / v_ego ** 2
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(v_ego), desired, active=False)
    log = None
    for i in range(50):
      log = step(v2, make_cs(v_ego, min(i * 0.02, 0.5), pressed=pressed), desired)  # measurement sweeping up at 2 m/s^3
    return v2, log

  def test_damping_opposes_measurement_motion_at_9mps(self, params):
    v2, log = self._sweep(9.0)
    assert v2.pid.d < -0.1
    assert log.d == pytest.approx(v2.pid.d, abs=1e-6)

  def test_no_damping_at_16mps(self, params):
    v2, _ = self._sweep(16.0)
    assert v2.pid.d == 0.0

  def test_no_damping_while_pressed(self, params):
    """While the driver moves the wheel the measurement rate is theirs: D must be exactly zero
    even though the measurement is moving and KD is nonzero at this speed."""
    v2, _ = self._sweep(9.0, pressed=True)
    assert v2.pid.d == 0.0


class TestReleaseHandling:
  def test_release_decay_is_one_shot(self, params):
    """Handing back the wheel decays the integrator once (x0.8), not every frame; v0 does not."""
    v2 = make_lac(LatControlTorqueV2)
    v2.pid.i = 1.0
    step(v2, make_cs(pressed=True), 0.0)  # frozen while pressed
    assert v2.pid.i == pytest.approx(1.0)
    step(v2, make_cs(pressed=False), 0.0)  # falling edge: one-shot decay, error is 0
    assert v2.pid.i == pytest.approx(STEER_RELEASE_I_DECAY)
    step(v2, make_cs(pressed=False), 0.0)
    assert v2.pid.i == pytest.approx(STEER_RELEASE_I_DECAY)

    v0 = make_lac(LatControlTorqueV0)
    v0.pid.i = 1.0
    step(v0, make_cs(pressed=True), 0.0)
    step(v0, make_cs(pressed=False), 0.0)
    assert v0.pid.i == pytest.approx(1.0)


class TestReleaseErrorRamp:
  """The release-edge error ramp: the P term used to land the whole hand-off error in one
  frame (P-step p90 2.24 within 100 ms measured on the 2026-08-29 override drive); the ramp
  eases the PID error in over RELEASE_ERROR_RAMP_T while feedforward stays immediate."""

  def _steady_error_run(self, v2, v_ego, lat_accel, press_frames, desired=0.0):
    """Standing measurement offset; pressed for press_frames, then released. Returns
    (errors, ps, fs) for 40 frames after the release edge."""
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(v_ego), desired, active=False)
    for _ in range(press_frames):
      step(v2, make_cs(v_ego, lat_accel, pressed=True), desired)
    errors, ps, fs = [], [], []
    for _ in range(40):
      log = step(v2, make_cs(v_ego, lat_accel), desired)
      errors.append(log.error)
      ps.append(log.p)
      fs.append(log.f)
    return errors, ps, fs

  def test_release_error_ramp_slopes_the_p_step(self, params):
    v2 = make_lac(LatControlTorqueV2)
    errors, ps, _ = self._steady_error_run(v2, v_ego=15.0, lat_accel=-0.5, press_frames=50)
    full = errors[-1]
    assert abs(full) > 0.4  # the standing error survives the run
    # first post-release frame carries dt/RAMP_T of the error, not all of it
    assert errors[0] == pytest.approx(full * DT / RELEASE_ERROR_RAMP_T, rel=1e-3)
    assert abs(ps[0]) < 0.1 * abs(ps[-1])
    # monotone ramp-in, complete within RELEASE_ERROR_RAMP_T
    assert abs(errors[9]) < abs(errors[19]) < abs(errors[29])
    assert errors[int(RELEASE_ERROR_RAMP_T / DT) + 1] == pytest.approx(full, rel=1e-6)

  def test_release_ramp_leaves_feedforward_alone(self, params):
    """The ramp eases the PID error only: at the release frame the feedforward already equals
    the non-ramped value (the request here, with roll, offset and friction all zero)."""
    v2 = make_lac(LatControlTorqueV2)
    request = 2e-3 * 15.0 ** 2
    _, _, fs = self._steady_error_run(v2, v_ego=15.0, lat_accel=request, press_frames=50, desired=2e-3)
    assert fs[0] == pytest.approx(request, abs=1e-6)
    assert fs[0] == pytest.approx(fs[-1], abs=1e-9)

  def test_no_ramp_without_a_release_edge(self, params):
    """A steady active run never engages the ramp: error is full-scale from the start."""
    v2 = make_lac(LatControlTorqueV2)
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(15.0), 0.0, active=False)
    log = step(v2, make_cs(15.0, -0.5), 0.0)
    assert abs(log.error) > 0.4


class TestInactivePriming:
  """Re-engaging with a wound wheel. v0's setpoint is the live request whatever the buffer
  holds, so its stale buffer only shows in the logged jerk; in v2 the friction input reads
  the shaped jerk and the D term reads the measurement rate, so a stale buffer or a stale
  previous_measurement would command against the held wheel. Priming while inactive is
  what keeps the first active frame clean."""

  V_EGO = 9.0  # inside the KD band, so the rate state matters too
  HOLD = 1.0  # m/s^2

  def _engage_after(self, lac, inactive_frames):
    desired = self.HOLD / self.V_EGO ** 2
    lac._v_now = self.V_EGO
    for _ in range(inactive_frames):
      step(lac, make_cs(self.V_EGO, self.HOLD), desired, active=False)
    return step(lac, make_cs(self.V_EGO, self.HOLD), desired, active=True), desired

  def test_primed_v2_engages_without_a_command(self, params):
    v2 = make_lac(LatControlTorqueV2, friction=FRICTION)
    v2.pid.i = 0.3
    log, desired = self._engage_after(v2, 100)
    assert log.error == pytest.approx(0.0, abs=1e-6)
    assert log.desiredLateralJerk == pytest.approx(0.0, abs=1e-6)
    assert friction_term(log, self.V_EGO, desired) == pytest.approx(0.0, abs=1e-6)
    assert log.d == pytest.approx(0.0, abs=1e-6)
    assert v2.pid.i == pytest.approx(0.3)  # deliberately not cleared while inactive

  def test_v0_shows_the_stale_buffer(self, params):
    v0 = make_lac(LatControlTorqueV0, friction=FRICTION)
    log, _ = self._engage_after(v0, 100)
    assert log.error == pytest.approx(0.0, abs=1e-6)  # setpoint == request regardless
    assert abs(log.desiredLateralJerk) > 1.0  # the buffered hold appears as jerk

  def test_unprimed_v2_would_push_against_the_held_wheel(self, params):
    """The counterfactual: without the inactive frames the stale buffer makes the shaped jerk
    ramp toward the clip and the friction term opposes the hold until the buffer refills."""
    v2 = make_lac(LatControlTorqueV2, friction=FRICTION)
    desired = self.HOLD / self.V_EGO ** 2
    worst = 0.0
    for _ in range(DELAY_FRAMES):
      log = step(v2, make_cs(self.V_EGO, self.HOLD), desired, active=True)
      worst = min(worst, friction_term(log, self.V_EGO, desired))
    assert worst < -0.1  # against a +1.0 hold


class TestCurvatureBuffer:
  """The buffer stores curvature and is rescaled by the live v^2 on read: braking through a
  constant-curvature arc produces no phantom jerk, so nothing reaches the friction input.
  v0's lat-accel buffer replays the old speed's values (visible in its jerk log only, since
  its setpoint collapses to the request), and a lat-accel buffer behind v2's friction input
  would put that phantom into the command."""

  def _brake_through_arc(self, lac, lataccel_buffer=False):
    desired = 2e-3
    v_ego = 20.0
    lac._v_now = v_ego
    if lataccel_buffer:
      lac.curvature_request_buffer = LatAccelBuffer(lac, lac.lat_accel_request_buffer_len)
    for _ in range(150):  # settle in the curve at constant speed first
      step(lac, make_cs(v_ego, desired * v_ego ** 2), desired)
    jerks, frictions = [], []
    for _ in range(250):
      v_ego = max(v_ego - 0.04, 10.0)  # -4 m/s^2, 20 -> 10 m/s
      lac._v_now = v_ego
      log = step(lac, make_cs(v_ego, desired * v_ego ** 2), desired)
      jerks.append(abs(log.desiredLateralJerk))
      frictions.append(abs(friction_term(log, v_ego, desired)))
    return max(jerks), max(frictions)

  def test_v2_sees_no_phantom(self, params):
    jerk, friction = self._brake_through_arc(make_lac(LatControlTorqueV2, friction=FRICTION))
    assert jerk < 1e-3
    assert friction < 0.05

  def test_v0_buffer_reads_a_phantom_jerk(self, params):
    jerk, _ = self._brake_through_arc(make_lac(LatControlTorqueV0, friction=FRICTION))
    assert jerk > 0.2

  def test_lat_accel_buffer_would_reach_the_friction_term(self, params):
    jerk, friction = self._brake_through_arc(make_lac(LatControlTorqueV2, friction=FRICTION), lataccel_buffer=True)
    assert jerk > 0.2
    assert friction > 0.05


class TestIntegratorFreeze:
  """v0's freeze semantics on the classified flag: steer_limited_by_safety (now driver-limited
  only), steeringPressed, or vEgo < 5 freeze the integrator; nothing else does."""

  def _run(self, v_ego=15.0, pressed=False, sls=False, frames=20):
    v2 = make_lac(LatControlTorqueV2)
    for _ in range(DELAY_FRAMES + 10):
      step(v2, make_cs(v_ego), 0.0, active=False)
    v2.pid.i = 0.1
    for _ in range(frames):
      step(v2, make_cs(v_ego, 0.5, pressed=pressed), 0.0, sls=sls)  # error -0.5: integrating would shrink i
    return v2.pid.i

  def test_frozen_when_limited_by_safety(self, params):
    assert self._run(sls=True) == pytest.approx(0.1)

  def test_frozen_when_pressed(self, params):
    assert self._run(pressed=True) == pytest.approx(0.1)

  def test_frozen_below_5mps(self, params):
    assert self._run(v_ego=4.0) == pytest.approx(0.1)

  def test_live_otherwise(self, params):
    assert self._run() < 0.1 - 1e-4
